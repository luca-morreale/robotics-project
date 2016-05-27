#include "kobra_plugins/pantilt_camera.h"

using namespace gazebo;

const std::string joints_name_tag[2] = {"PanJoint", "TiltJoint"};

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

    topic_name = _sdf->GetElement("topicName")->Get<std::string>();
    
    camera_name = _sdf->GetElement("cameraName")->Get<st_sdfd::string>();
    //camera = camera??

    rosnode = new ros::NodeHandle();
    sub = rosnode->subscribe(this->topic_name, 10, &PantTiltCameraPlugin::pantiltCallback, this);
}

void PantTiltCameraPlugin::update() 
{
    // TODO
}

void PantTiltCameraPlugin::pantiltCallback(const ptz_msgsConstPtr& msg)
{
    // TODO
    float pan = msg.pan;
    float tilt = msg.tilt;
    float zoom = msg.zoom;

    //pan_joint.setVelocity();
    //tilt_joint.setVelocity();

}

bool PantTiltCameraPlugin::checkTags(sdf::ElementPtr _sdf)
{
    

    return checkJointsTag(_sdf) && checkTopicTags(_sdf);
    
}

bool PantTiltCameraPlugin::checkJointsTag(sdf::ElementPtr _sdf)
{
    for(int i=0; i<N_JOINTS; i++) {
        if(!_sdf->HasElement(joints_name_tag[i])) {
            ROS_FATAL_STREAM("PantTiltCameraPlugin: <"+ joints_name_tag[i] +"> missing!");
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

void extractNames(sdf::ElementPtr _sdf)
{
    
}

void PantTiltCameraPlugin::extractJoints(sdf::ElementPtr _sdf) 
{
    for(int i=0; i<N_JOINTS; i++) {
        joints[joints_name[i]] = this->parent->GetJoint(joints_name[i]);
    }
}


PantTiltCameraPlugin::~PantTiltCameraPlugin()
{
    delete rosnode;
}
