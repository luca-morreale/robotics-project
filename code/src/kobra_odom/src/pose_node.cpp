#include "kobra_odom/pose_node.h"


bool PoseNode::Prepare() 
{
    ROS_INFO("Preparing the node %s", ros::this_node::getName().c_str());
    
    rosnode = new ros::NodeHandle();
    initPoseValues();

    poseSub = rosnode->subscribe("/amcl_pose", 10, &PoseNode::poseCallback, this);
    odomSub = rosnode->subscribe("/kobra/odom", 10, &PoseNode::odomCallback, this);
    posePub = rosnode->advertise<nav_msgs::Odometry>("/odom", 10);
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
    std::string type;
    rosnode->param<std::string>("integration_type", type, EULER);

    if(type == EULER) {
        integration = &PoseNode::eulerIntegration;
    } else if(type == KUTTA) {
        integration = &PoseNode::rungeKuttaIntegration;
    } else if(type == EXACT) {
        integration = &PoseNode::exactIntegration;
    }

    last_msg_time = -1.0;
    last_restore_pose = -1.0;
}

void PoseNode::poseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{
    double dt = msg->header.stamp.toSec() -last_restore_pose;
    if(last_restore_pose < 0 || dt < RUN_PERIOD_DEFAULT) {
        x = msg->pose.pose.position.x;
        y = msg->pose.pose.position.y;
        geometry_msgs::Quaternion orientation = msg->pose.pose.orientation;
        tf::Matrix3x3 mat(tf::Quaternion(orientation.x, orientation.y, orientation.z, orientation.w));
        double roll, pitch;
        mat.getRPY(roll, pitch, this->yaw);

        last_restore_pose = msg->header.stamp.toSec();
    }
}

void PoseNode::odomCallback(const nav_msgs::Odometry::ConstPtr& msg) 
{
    nav_msgs::Odometry in = *msg;
	// Init -- extract cmd_velocities from the msg
    if(last_msg_time < 0 || last_restore_pose < 0) {
        last_msg_time = msg->header.stamp.toSec();
        return;
    }

    double linear = msg->twist.twist.linear.x;
    double angular = msg->twist.twist.angular.z;
    double time_step = msg->header.stamp.toSec() - last_msg_time;

    if(time_step < RUN_PERIOD_DEFAULT) {
        return;
    }

    // Perform Integration
    (this->*integration)(linear, angular, time_step);
    
    //Publish the transformation over tf
    broadcastTransformation(msg);

    //Create Odometry message
    publishOdometryMessage(linear,angular, msg);
    
   	last_msg_time = msg->header.stamp.toSec();
}

void PoseNode::eulerIntegration(double linear, double angular, double time_step)
{
    x = x + linear * cos(yaw) * time_step;
    y = y + linear * sin(yaw) * time_step;
    yaw = yaw + angular * time_step;
}

void PoseNode::rungeKuttaIntegration(double linear, double angular, double time_step)
{
    x = x + linear * time_step * cos(yaw+(angular * time_step) / 2);
    y = y + linear * time_step * sin(yaw+(angular * time_step) / 2);
    yaw = yaw + angular * time_step;
}

void PoseNode::exactIntegration(double linear, double angular, double time_step)
{
    double yaw_old = yaw;
    double yaw_new = yaw_old + angular * time_step;
    x = x + (linear / angular) * (sin(yaw_new) - sin(yaw_old));
    y = y + (linear / angular) * (cos(yaw_new) - cos(yaw_old));
    yaw = yaw_new;
}

void PoseNode::publishOdometryMessage(double linear, double angular, const nav_msgs::Odometry::ConstPtr& old_msg)
{
    nav_msgs::Odometry msg;
    msg.header.stamp = old_msg->header.stamp;
    msg.header.frame_id="/map";
    msg.child_frame_id = "/odom";

    //set position
    msg.pose.pose.position.x = x;
    msg.pose.pose.position.y = y;
    msg.pose.pose.position.z = 0.0;
    msg.pose.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);

    //set velocity
    msg.twist.twist.linear.x = linear;
    msg.twist.twist.linear.y = 0.0;
    msg.twist.twist.angular.z = angular;

    posePub.publish(msg);
}

void PoseNode::broadcastTransformation(const nav_msgs::Odometry::ConstPtr& old_msg)
{
    geometry_msgs::TransformStamped odom_trans;

    odom_trans.header.stamp = old_msg->header.stamp;
    odom_trans.header.frame_id = "/map";
    odom_trans.child_frame_id = "/odom";

    odom_trans.transform.translation.x = x;
    odom_trans.transform.translation.y = y;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = tf::createQuaternionMsgFromYaw(yaw);

    odom_broadcaster.sendTransform(odom_trans);
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