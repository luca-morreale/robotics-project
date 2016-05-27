#include "kobra_plugins/steering_control.h"

using namespace gazebo;

const std::string joints_tag_names[4] = {"LeftFrontJoint", "RightFrontJoint", "LeftRearJoint", "RightRearJoint"};
const std::string odometry_topic_tag = "OdometryTopic";
const std::string odometry_frame_tag = "OdometryFrame";
const std::string command_tag = "CmdTpoic";

void SteeringControlPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
    if (!ros::isInitialized()) {
      ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin.");
      return;
    }

    this->parent = _model;  
    this->update_connection = event::Events::ConnectWorldUpdateBegin(boost::bind(&SteeringControlPlugin::update, this));

    if(!checkJoints(_sdf)) {
        return;
    }

    for(int i=0; i < N_JOINTS; i++) {
        joints_names[i] = _sdf->GetElement(joints_tag_names[i])->Get<std::string>();
        joints[i] = this->parent->GetJoint(joints_names[i]);
    }
    
    extractOdomInfo(_sdf);
    extractCmdTopic(_sdf);

    rosnode = new ros::NodeHandle();

    cmd_sub = rosnode->subscribe(this->command_topic, 10, &SteeringControlPlugin::cmdCallback, this);
    odom_pub = rosnode->advertise<nav_msgs::Odometry>(this->odometry_topic, 1);
    
}

void SteeringControlPlugin::update() 
{
    // TODO
}

void SteeringControlPlugin::cmdCallback(const geometry_msgs::Twist::ConstPtr& cmd_msg)
{
    // TODO
}


bool SteeringControlPlugin::checkJoints(sdf::ElementPtr _sdf)
{
    for(int i = 0; i < N_JOINTS; i++){
        if(!_sdf->HasElement(joints_tag_names[i])) {
            ROS_FATAL_STREAM("SteeringOdometry: joint "+ this->joints_names[i] +" missing!");
            return false;
        }
    }
    return true;
}

void SteeringControlPlugin::extractOdomInfo(sdf::ElementPtr _sdf)
{    
    if (!_sdf->HasElement(odometry_topic_tag)) {
        this->odometry_topic = "odom";
        ROS_WARN("SteeringOdometry: missing <%s>, defaults to %s.", this->odometry_topic_tag, this->odometry_topic);
    } else {
        this->odometry_topic = _sdf->GetElement(odometry_topic_tag)->Get<std::string>();
    }

    
    if (!_sdf->HasElement("odometryFrame")) {
        this->odometry_frame = "odom";
        ROS_WARN("SteeringOdometry: missing <%s>, defaults to %s.", this->odometry_frame_tag, this->odometry_frame);
    } else {
        this->odometry_frame = _sdf->GetElement(odometry_frame_tag)->Get<std::string>();
    }
}

void SteeringControlPlugin::extractCmdTopic(sdf::ElementPtr _sdf)
{
    if (!_sdf->HasElement("commandTopic")) {
        this->command_topic = "cmd_vel";
        ROS_WARN("SteeringOdometry missing <%s>, defaults to %s.", this->command_tag, this->command_topic);
    } else {
        this->command_topic = _sdf->GetElement(command_tag)->Get<std::string>();
    }
}


SteeringControlPlugin::~SteeringControlPlugin()
{
    delete rosnode;
}
