#include "pantilt_camera.h"

using namespace gazebo;


/*
<panJoint> camera_support_joint orizzontale
<tiltJoint> camera_junction_sphere_joint verticale
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

    pan_joint_name = _sdf->GetElement("panJoint")->Get<std::string>();
    pan_joint = this->parent->GetJoint(pan_joint_name);

    tilt_joint_name = _sdf->GetElement("tiltJoint")->Get<std::string>();
    tilt_joint = this->parent->GetJoint(tilt_joint_name);

    topic_name = _sdf->GetElement("topicName")->Get<std::string>();
    
    camera_name = _sdf->GetElement("cameraName")->Get<std::string>();
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
    float = pan

}

bool PantTiltCameraPlugin::checkTags(sdf::ElementPtr _sdf)
{
    if(!_sdf->HasElement("panJoint")) {
        ROS_FATAL_STREAM("PantTiltCameraPlugin: <panJoint> missing!");
        return false;
    }
    if(!_sdf->HasElement("tiltJoint")) {
        ROS_FATAL_STREAM("PantTiltCameraPlugin: <tiltJoint> missing!");
        return false;
    }
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


PantTiltCameraPlugin::~PantTiltCameraPlugin()
{
    delete rosnode;
}
