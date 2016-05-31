#include "kobra_plugins/pantilt_camera.h"


using namespace gazebo;

const MapString joints_name_tag = {{PAN, "panJoint"}, {TILT, "tiltJoint"}};

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

    rosnode = new ros::NodeHandle();

    rossub = rosnode->subscribe(topic_name, 10, &PantTiltCameraPlugin::pantiltCallback, this);
}

void PantTiltCameraPlugin::pantiltCallback(const kobra_plugins::ptz_msg::ConstPtr &msg)
{
    float pan_degree = msg->pan;
    float tilt_degree = msg->tilt;
    float zoom = msg->zoom;

    float pan_time = pan_degree / pan_velocity;
    if(pan_time > 0){
        joints[PAN]->SetVelocity(0, pan_velocity);
        ros::Duration(pan_time).sleep();
    }
    
    joints[PAN]->SetVelocity(0, 0);

    float tilt_time = tilt_degree / tilt_velocity;
    if(tilt_time > 0){
        joints[TILT]->SetVelocity(0, tilt_velocity);
        ros::Duration(tilt_time).sleep();
    }

    joints[TILT]->SetVelocity(0, 0); 
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
    if(!_sdf->HasElement("panVelocity")) {
        ROS_WARN("PantTiltCameraPlugin: <panVelocity> missing, default value: %f", DEFAULT_PAN_VEL);
    } else {
        pan_velocity = _sdf->GetElement("panVelocity")->Get<float>();
    }

    if(!_sdf->HasElement("tiltVelocity")) {
        ROS_WARN("PantTiltCameraPlugin: <tiltVelocity> missing, default value: %f", DEFAULT_TILT_VEL);
    } else {
        tilt_velocity = _sdf->GetElement("tiltVelocity")->Get<float>();
    }
}

void PantTiltCameraPlugin::extractJoints(sdf::ElementPtr _sdf) 
{
    for(MapStrConstIterator it = joints_name_tag.begin(); it != joints_name_tag.end(); ++it) {
        //it->first PAN or TILT
        //it->second tag 
        joints_name[it->first] = _sdf->GetElement(it->second)->Get<std::string>();
        joints[it->first] = this->parent->GetJoint(joints_name[it->first]);
        joints[it->first]->SetVelocity(0, 0); 
    }
}

PantTiltCameraPlugin::~PantTiltCameraPlugin()
{
    delete rosnode;
}
