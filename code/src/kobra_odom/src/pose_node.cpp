#include "kobra_odom/pose_node.h"
#include <tf/transform_broadcaster.h>

bool PoseNode::Prepare() 
{
    ROS_INFO("Preparing the node %s", ros::this_node::getName().c_str());
    initPoseValues();

    rosnode = new ros::NodeHandle();
    odomSub = rosnode->subscribe("/kobra/odom", 10, &PoseNode::odomCallback, this);
    odomSub = rosnode->subscribe("/robot_gt", 10, &PoseNode::odomCallback, this);
    posePub = rosnode->advertise<nav_msgs::Odometry>("odom", 10);
    ROS_INFO("Node %s ready to run.", ros::this_node::getName().c_str());
    return true;
}


void PoseNode::RunContinuously() 
{
  	ROS_INFO("Node %s running continuously.", ros::this_node::getName().c_str());
  	ros::spin();
}

void PoseNode::Shutdown() 
{
  	ROS_INFO("Node %s shutting down.", ros::this_node::getName().c_str());
  	delete rosnode;
}

void PoseNode::initPoseValues()
{
    rosnode->param<double>("x", x, 0.0);
    rosnode->param<double>("y", y, 0.0);
    rosnode->param<double>("yaw", yaw, 0.0);

    rosnode->param<std::string>("integration_type", type, EULER);

    last_msg_time = -1.0;
}

void PoseNode::odomCallback(const nav_msgs::Odometry::ConstPtr& msg) 
{
	// Init
    if(last_msg_time < 0) {
        last_msg_time = msg->header.stamp.toSec();
        return;
    }

    float linear = msg->twist.twist.linear.x;
    float angular = msg->twist.twist.angular.z;
    double time_step = msg->header.stamp.toSec() - last_msg_time;

    if(time_step < RUN_PERIOD_DEFAULT) {
        return;
    }

    nav_msgs::Odometry out;
    out.header = msg->header;
    initPoseMsg(out);

    // Perform Integration
    if(type == EULER) {
        eulerIntegration(linear, angular, time_step, out);
    } else if(type == KUTTA) {
        rungeKuttaIntegration(linear, angular, time_step, out);
    } else if(type == EXACT) {
        exactIntegration(linear, angular, time_step, out);
    }
    
    setPoseValues(out, linear, angular);
    posePub.publish(out);
    
   	last_msg_time = msg->header.stamp.toSec();
}

void PoseNode::eulerIntegration(double linear, double angular, double time_step, nav_msgs::Odometry &msg)
{
	x = x + linear * cos(yaw) * time_step;
    y = y + linear * sin(yaw) * time_step;
    yaw = yaw + angular * time_step;

    msg.pose.pose.position.x = x;
    msg.pose.pose.position.y = y;
    msg.pose.pose.orientation.z = yaw;
}

void PoseNode::rungeKuttaIntegration(double linear, double angular, double time_step, nav_msgs::Odometry &msg)
{
	x = x + linear * time_step * cos(yaw+(angular * time_step) / 2);
    y = y + linear * time_step * sin(yaw+(angular * time_step) / 2);
    yaw = yaw + angular * time_step;

    msg.pose.pose.position.x = x;
    msg.pose.pose.position.y = y;
    msg.pose.pose.orientation.z = yaw;
}
void PoseNode::exactIntegration(double linear, double angular, double time_step, nav_msgs::Odometry &msg)
{
	double yaw_old = yaw;
	double yaw_new = yaw_old + angular*time_step;
	x = x + (linear / angular) * (sin(yaw_new) - sin(yaw_old));
    y = y + (linear / angular) * (cos(yaw_new) - cos(yaw_old));
    yaw = yaw_new;
}

void PoseNode::initPoseMsg(nav_msgs::Odometry &msg)
{
    msg.child_frame_id = "base_link";
    msg.pose.pose.orientation.x = 0.0;
    msg.pose.pose.orientation.y = 0.0;
    msg.pose.pose.orientation.z = 0.0;
    msg.pose.pose.orientation.w = 0.0;
    msg.pose.pose.position.x = 0.0;
    msg.pose.pose.position.y = 0.0;
    msg.pose.pose.position.z = 0.0;   
}

void PoseNode::setPoseValues(nav_msgs::Odometry &msg, double linear, double angular)
{
    msg.pose.pose.position.x = x;
    msg.pose.pose.position.y = y;
    msg.pose.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);

    msg.twist.twist.linear.x = linear;
    msg.twist.twist.angular.z = angular;
}



using namespace std;

int main(int argc, char **argv) {
  	ros::init(argc, argv, NAME_OF_THIS_NODE);
  
  	PoseNode pose_node;
   
  	if(!pose_node.Prepare()){
  		return (-1);
  	}
  	pose_node.RunContinuously();
  	pose_node.Shutdown();
  
  	return 0;
}