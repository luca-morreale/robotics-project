#include "../include/kobra_plugins/steering_control.h"

using namespace gazebo;

/* Tag names used in model.sdf */

const MapString SteeringControlPlugin::joints_name_tag = {{LEFT_FRONT, "LeftFrontJoint"}, {RIGHT_FRONT, "RightFrontJoint"}, {LEFT_REAR, "LeftRearJoint"}, {RIGHT_REAR, "RightRearJoint"}};
const std::string SteeringControlPlugin::odometry_topic_tag = "OdometryTopic";
const std::string SteeringControlPlugin::command_tag = "CmdTopic";
const std::string SteeringControlPlugin::wheel_separation_tag = "WheelSeparation";
const std::string SteeringControlPlugin::wheel_diameter_tag = "WheelDiameter";



void SteeringControlPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
    if (!ros::isInitialized()) {
      //ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin.");
      return;
    }

    this->parent = _model;  
    this->update_connection = event::Events::ConnectWorldUpdateBegin(boost::bind(&SteeringControlPlugin::update, this));

    setDefaultValues();

    //Extract informations from the sdf file
    if(!extractJoints(_sdf)) { return; }
    extractRobotInfo(_sdf);
    extractOdomInfo(_sdf);
    extractCmdTopic(_sdf);

    rosnode = new ros::NodeHandle();

    cmd_sub = rosnode->subscribe(this->command_topic, 10, &SteeringControlPlugin::commandCallback, this);
    odom_pub = rosnode->advertise<nav_msgs::Odometry>(this->odometry_topic, 1);
}

void SteeringControlPlugin::setDefaultValues()
{
    /* Kobra dimesnions */
    wheel_diameter = 0.145;
    wheel_separation = 0.495;

    /* Init velocities */
    wheel_speed[RIGHT_FRONT] = 0;
    wheel_speed[LEFT_FRONT] = 0;
    wheel_speed[RIGHT_REAR] = 0;
    wheel_speed[LEFT_REAR] = 0;
    yaw = 0;
    
    /* Initilize last update to current time */
    last_update_time = ros::Time::now();
}

void SteeringControlPlugin::commandCallback(const geometry_msgs::Twist::ConstPtr& cmd_msg)
{
    double linear = cmd_msg->linear.x;
    double angular = cmd_msg->angular.z;

    calculateWheelVelocity(linear, angular);
}

void SteeringControlPlugin::setWheelVelocity()
{
    for(MapJointIterator it = joints.begin(); it != joints.end(); ++it) {
        // transformation from linear velocity to speed of the wheel
        joints[it->first]->SetVelocity(0, wheel_speed[it->first] / (wheel_diameter / 2.0));
    }
}

void SteeringControlPlugin::calculateWheelVelocity(double linear, double angular)
{
    wheel_speed[RIGHT_FRONT] = linear + angular * (wheel_separation / 2);
    wheel_speed[RIGHT_REAR] = linear + angular * (wheel_separation / 2);
    wheel_speed[LEFT_FRONT] = linear - angular * (wheel_separation / 2);
    wheel_speed[LEFT_REAR] = linear - angular * (wheel_separation / 2);

    //ROS_DEBUG("RIGHT wheels speed is now %lf", wheel_speed[RIGHT_FRONT]);
    //ROS_DEBUG("LEFT wheels speed is now %lf", wheel_speed[LEFT_FRONT]);
}

void SteeringControlPlugin::getWheelVelocity(MapDouble *vel)
{
    for(MapJointIterator it = joints.begin(); it != joints.end(); ++it) {
        // transformation from speed of the wheel to the speed of the center
        (*vel)[it->first] = joints[it->first]->GetVelocity(0) * (wheel_diameter / 2.0);
    }
}

void SteeringControlPlugin::update() 
{
    current_time = ros::Time::now();
    double time_step = (current_time - last_update_time).toSec();

    publishOdometry(time_step);
    last_update_time = ros::Time::now();   // Update the time of the last update
    setWheelVelocity();
}

void SteeringControlPlugin::publishOdometry(double dt)
{
    
    MapDouble vel;
    getWheelVelocity(&vel);

    double linear = (vel[RIGHT_FRONT] + vel[LEFT_FRONT]) / 2;
    double angular = (vel[RIGHT_FRONT] - vel[LEFT_FRONT]) / wheel_separation;
    
    double dx = linear * cos(yaw) * dt;
    double dy = linear * sin(yaw) * dt;
    yaw = yaw + angular * dt;
    

    nav_msgs::Odometry odom_msg;
    odom_msg.header.stamp = ros::Time::now();
    
    odom_msg.twist.twist.linear.x = linear;
    odom_msg.twist.twist.angular.z = angular;

    odom_msg.pose.pose.position.x = dx;
    odom_msg.pose.pose.position.y = dy;
    odom_msg.pose.pose.position.z = 0;

    odom_pub.publish(odom_msg);
}

bool SteeringControlPlugin::extractJoints(sdf::ElementPtr _sdf)
{
    for(MapStrConstIterator it = joints_name_tag.begin(); it != joints_name_tag.end(); ++it) {
        if(!_sdf->HasElement(it->second)) {
            //ROS_FATAL_STREAM(ROS_NODE_NAME " missing "+ it->second);
            return false;
        } else {
            joints_name[it->first] = _sdf->GetElement(it->second)->Get<std::string>();
            joints[it->first] = this->parent->GetJoint(joints_name[it->first]);
            joints[it->first]->SetVelocityLimit(0, MAX_VELOCITY);
        }
    }
    return true;
}

void SteeringControlPlugin::extractRobotInfo(sdf::ElementPtr _sdf)
{
    if (!_sdf->HasElement(wheel_separation_tag)) {
    	//ROS_WARN(ROS_NODE_NAME " missing <%s>, defaults to %s.", this->wheel_separation_tag, this->wheel_separation);
    } else {
        this->wheel_separation= _sdf->GetElement(wheel_separation_tag)->Get<double>();
    }

    if (!_sdf->HasElement(wheel_diameter_tag)) {
    	//ROS_WARN(ROS_NODE_NAME " missing <%s>, defaults to %s.", this->wheel_diameter_tag, this->wheel_diameter);
    } else {
        this->wheel_diameter= _sdf->GetElement(wheel_diameter_tag)->Get<double>();
    }
}

void SteeringControlPlugin::extractOdomInfo(sdf::ElementPtr _sdf)
{    
    if (!_sdf->HasElement(odometry_topic_tag)) {
        this->odometry_topic = "odom";
        //ROS_WARN(ROS_NODE_NAME " missing <%s>, defaults to %s.", this->odometry_topic_tag, this->odometry_topic);
    } else {
        this->odometry_topic = _sdf->GetElement(odometry_topic_tag)->Get<std::string>();
    }
}

void SteeringControlPlugin::extractCmdTopic(sdf::ElementPtr _sdf)
{
    if (!_sdf->HasElement(command_tag)) {
        this->command_topic = "cmd_vel";
        //ROS_WARN(ROS_NODE_NAME " missing <%s>, defaults to %s.", this->command_tag, this->command_topic);
    } else {
        this->command_topic = _sdf->GetElement(command_tag)->Get<std::string>();
    }
}

SteeringControlPlugin::~SteeringControlPlugin()
{
    delete rosnode;
}
