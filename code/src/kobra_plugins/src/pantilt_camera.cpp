#include "kobra_plugins/pantilt_camera.h"


using namespace gazebo;

int sign(float x) {
    return (x > 0) - (x < 0);
}

const MapString PantTiltCameraPlugin::joints_name_tag = {{PAN, "panJoint"}, {TILT, "tiltJoint"}};
const MapString PantTiltCameraPlugin::velocities_name_tag = {{PAN, "panVelocity"}, {TILT, "tiltVelocity"}};
const MapString PantTiltCameraPlugin::radius_name_tag = {{PAN, "panJointRadius"}, {TILT, "tiltJointRadius"}};

/*
<panJoint> camera_support_joint orizzontale
<panVelocity>
<tiltJoint> camera_junction_sphere_joint verticale
<tiltVelocity>
<topicName>/kobra/ptz
<cameraName>
*/

void PantTiltCameraPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
    if (!ros::isInitialized()) {
        ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin.");
        return;
    }

    this->parent = _model;
    this->update_connection = event::Events::ConnectWorldUpdateBegin(boost::bind(&PantTiltCameraPlugin::update, this));
    
    if(!checkTags(_sdf)) {
        return;
    }

    extractNames(_sdf);
    extractJoints(_sdf);
    extractVelocities(_sdf);
    extractRadius(_sdf);

    rosnode = new ros::NodeHandle();

    rossub = rosnode->subscribe(topic_name, 10, &PantTiltCameraPlugin::pantiltCallback, this);

}

void PantTiltCameraPlugin::update() 
{
    joints[TILT]->SetVelocity(0, joint_velocity[TILT]);
    joints[PAN]->SetVelocity(0, joint_velocity[PAN]);
}

void PantTiltCameraPlugin::pantiltCallback(const kobra_plugins::ptz_msg::ConstPtr &msg)
{
    double pan_degree = fixAngle(PAN, msg->pan); 
    double tilt_degree = fixAngle(TILT, msg->tilt);
    double zoom = msg->zoom;

 
    double pan_time = abs(pan_degree / velocity[PAN] * radius[PAN]);    // convertion from v to omega
    double tilt_time = abs(tilt_degree / velocity[TILT] * radius[TILT]);

    if(tilt_time > 0){
        moveJoint(TILT, tilt_degree, tilt_time);
    }

    if(pan_time > 0){
        moveJoint(PAN, pan_degree, pan_time);
    }
}

void PantTiltCameraPlugin::moveJoint(std::string JOINT, double degree, double sleep_time)
{
    joint_velocity[JOINT] = velocity[JOINT] * sign(degree);
    ros::Duration(sleep_time).sleep();
    joint_velocity[JOINT] = 0;
}

double PantTiltCameraPlugin::fixAngle(std::string JOINT, double degree)
{

    math::Angle current_angle = joints[JOINT]->GetAngle(0);

    if(degree < 0) {
        return std::max((math::Angle::Pi - current_angle).Radian(), degree);
    } else {
        return std::min((math::Angle::Zero - current_angle).Radian(), degree);
    }

}

bool PantTiltCameraPlugin::checkTags(sdf::ElementPtr _sdf)
{
    return checkJointsTag(_sdf) && checkTopicTags(_sdf);
}

bool PantTiltCameraPlugin::checkJointsTag(sdf::ElementPtr _sdf)
{
    for(MapStrConstIterator it = joints_name_tag.begin(); it != joints_name_tag.end(); ++it) {
        if(!_sdf->HasElement(it->second)) {
            ROS_FATAL_STREAM("PantTiltCameraPlugin: <"+ it->first +"> missing!");
            return false;
        }
    }
    return true;
}

bool PantTiltCameraPlugin::checkTopicTags(sdf::ElementPtr _sdf) 
{
    if(!_sdf->HasElement("topicName")) {
        ROS_FATAL_STREAM("PantTiltCameraPlugin: <topicName> missing!");
        return false;
    }
    if(!_sdf->HasElement("cameraName")) {
        ROS_FATAL_STREAM("PantTiltCameraPlugin: <cameraName> missing!");
        return false;
    }
    return true;
}

void PantTiltCameraPlugin::extractNames(sdf::ElementPtr _sdf)
{
    topic_name = _sdf->GetElement("topicName")->Get<std::string>();
    camera_name = _sdf->GetElement("cameraName")->Get<std::string>();
}

void PantTiltCameraPlugin::extractVelocities(sdf::ElementPtr _sdf)
{
    for(MapStrConstIterator it = velocities_name_tag.begin(); it != velocities_name_tag.end(); ++it) {
        if(!_sdf->HasElement(it->second)) {
            velocity[it->first] = DEFAULT_VEL;
            ROS_WARN("PantTiltCameraPlugin: <%s> missing, default value: %f", it->second, DEFAULT_VEL);
        } else {
            velocity[it->first] = _sdf->GetElement(it->second)->Get<double>();
        }
    }
}

void PantTiltCameraPlugin::extractRadius(sdf::ElementPtr _sdf)
{
    for(MapStrConstIterator it = radius_name_tag.begin(); it != radius_name_tag.end(); ++it) {
        if(!_sdf->HasElement(it->second)) {
            radius[it->first] = DEFAULT_VEL;
            ROS_WARN("PantTiltCameraPlugin: <%s> missing, default value: %f", it->second, DEFAULT_RADIUS);
        } else {
            radius[it->first] = _sdf->GetElement(it->second)->Get<double>();
        }
    }
}

void PantTiltCameraPlugin::extractJoints(sdf::ElementPtr _sdf) 
{
    for(MapStrConstIterator it = joints_name_tag.begin(); it != joints_name_tag.end(); ++it) {
        //it->first PAN or TILT
        //it->second tag 
        joints_name[it->first] = _sdf->GetElement(it->second)->Get<std::string>();
        joints[it->first] = this->parent->GetJoint(joints_name[it->first]);
        joint_velocity[it->first] = 0.0;
    }
}

PantTiltCameraPlugin::~PantTiltCameraPlugin()
{
    delete rosnode;
}
