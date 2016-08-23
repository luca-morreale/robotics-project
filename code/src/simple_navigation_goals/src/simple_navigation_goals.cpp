#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <tf/transform_broadcaster.h>
#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <limits.h>
#include <unistd.h>
#include "../include/simple_navigation_goals/strtk.hpp"

using namespace std;

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int main(int argc, char** argv){
	ros::init(argc, argv, "simple_navigation_goals");

	//Print directory where the exec is running from -- Used for debug purpose only
	char result[ PATH_MAX ];
	ssize_t count = readlink( "/proc/self/exe", result, PATH_MAX );
	std::string path = std::string( result, (count > 0) ? count : 0 );
	cout << path;


	//tell the action client that we want to spin a thread by default
	MoveBaseClient ac("move_base", true);

	//wait for the action server to come up
	while(!ac.waitForServer(ros::Duration(5.0))){
		ROS_INFO("Waiting for the move_base action server to come up");
	}
	ROS_INFO("The move_base action server is up and running");

	move_base_msgs::MoveBaseGoal goal;

	while(true){
		//we'll send a goal to the robot reading from the txt file
		std::ifstream file("../../../src/simple_navigation_goals/path.txt");
		if(file.is_open()){
			std::string str;
			while(std::getline(file, str)){
				std::vector<double> double_vector;
				if(strtk::parse(str," ",double_vector)){
					//Print values
					cout<<"Received goal is: ";
					for(int i=0; i<double_vector.size(); i++){
						cout<<double_vector[i]<<" ";
					}
					cout<<endl;
				}
				else{
					ROS_ERROR("Cannot read from text file.");
					file.close();
					return -1;
				}
				goal.target_pose.header.frame_id = "map";
				goal.target_pose.header.stamp = ros::Time::now();

				goal.target_pose.pose.position.x = double_vector[0];
				goal.target_pose.pose.position.y = double_vector[1];
				goal.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(double_vector[3]);;

				ROS_INFO("Sending goal");
				ac.sendGoal(goal);

				ac.waitForResult();

				if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
					ROS_INFO("The goal has been reached, passing to the next one");
				else
					ROS_INFO("The robot failed to reach the goal for some reason");
			}
		}
		else{
			ROS_WARN("Cannot open the file path.txt");
			cerr << "Error: " << strerror(errno);
			return -1;
		}
		file.close();
	}
	return 0;
}
