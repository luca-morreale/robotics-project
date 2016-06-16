#include "kobra_odom/kobra_pose_node.h"

bool PoseNode::Prepare() {
    ROS_INFO("Preparing the node %s", ros::this_node::getName().c_str());
    initPoseValues();
    initPoseMsg();

    rosnode = new ros::NodeHandle();
    odomSub = rosnode->subscribe("/kobra/odom", 10, &PoseNode::odomCallback, this);
    posePub = rosnode->advertise<geometry_msgs::PoseStamped>("/kobra/pose", 10);
    ROS_INFO("Node %s ready to run.", ros::this_node::getName().c_str());
    return true;
}


void PoseNode::RunContinuously() {
  	ROS_INFO("Node %s running continuously.", ros::this_node::getName().c_str());  
  	ros::spin();
}

void PoseNode::Shutdown() {
  	ROS_INFO("Node %s shutting down.", ros::this_node::getName().c_str());
  	delete rosnode;
}


void PoseNode::odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
	//Init
    if(last_msg_time < 0) {
        last_msg_time = msg->header.stamp.toSec();
        return;
    }
    //Grab information from Odometry msg
    float linear = msg->twist.twist.linear.x;
    float angular = msg->twist.twist.angular.z;
    double time_step = msg->header.stamp.toSec() - last_msg_time;

    //Perform Integration
    pose_msg.header = msg->header;
    //TODO: choose method via parameters in launch file
    eulerIntegration(linear, angular, time_step);
    rungeKuttaIntegration(linear, angular, time_step);
    exactIntegration(linear, angular, time_step);
    
    posePub.publish(pose_msg);
    
   	last_msg_time = msg->header.stamp.toSec();
}

void PoseNode::eulerIntegration(double linear, double angular, double time_step){
	x = x + linear*cos(yaw)*time_step;
    y = y + linear*sin(yaw)*time_step;
    yaw = yaw + angular*time_step;

    pose_msg.pose.position.x = x;
    pose_msg.pose.position.y = y;
    pose_msg.pose.orientation.z = yaw;
}

void PoseNode::rungeKuttaIntegration(double linear, double angular, double time_step){
	x = x + linear*time_step*cos(yaw+(angular*time_step)/2);
    y = y + linear*time_step*sin(yaw+(angular*time_step)/2);
    yaw = yaw + angular*time_step;

    pose_msg.pose.position.x = x;
    pose_msg.pose.position.y = y;
    pose_msg.pose.orientation.z = yaw;
}
void PoseNode::exactIntegration(double linear, double angular, double time_step){
	double yaw_old=yaw;
	double yaw_new = yaw_old + angular*time_step;
	x = x + (linear/angular)*(sin(yaw_new)-sin(yaw_old));
    y = y + (linear/angular)*(cos(yaw_new)-cos(yaw_old));
    yaw=yaw_new;

    pose_msg.pose.position.x = x;
    pose_msg.pose.position.y = y;
    pose_msg.pose.orientation.z = yaw;
}

void PoseNode::initPoseValues(){
	/*GET VALUES FROM PARAM
	    if (!Handle.getParam(ros::this_node::getName()+"/x", x)) return false;
    if (!Handle.getParam(ros::this_node::getName()+"/y", y)) return false;
    if (!Handle.getParam(ros::this_node::getName()+"/yaw", yaw)) return false;
    get also value for the integration method
    */
    x = 0.0; //Temporary values...TO BE MODIFIED
    y = 0.0;
 	yaw = 0.0;
    roll = 0.0;
    pitch = 0.0;
    last_msg_time = -1.0;
}

void PoseNode::initPoseMsg(){
    pose_msg.header.frame_id = "/world";
    pose_msg.pose.orientation.x = 0.0;
    pose_msg.pose.orientation.y = 0.0;
    pose_msg.pose.orientation.z = 0.0;
    pose_msg.pose.orientation.w = 0.0;
    pose_msg.pose.position.x = 0.0;
    pose_msg.pose.position.y = 0.0;
    pose_msg.pose.position.z = 0.0;   
}

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