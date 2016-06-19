#ifndef ODOM_NODE_H
#define ODOM_NODE_H

#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/PoseStamped.h"
#include "nav_msgs/Odometry.h"
#include <math.h>

#define RUN_PERIOD_DEFAULT 0.001
#define NAME_OF_THIS_NODE "pose_node"
#define EULER "euler"
#define KUTTA "kutta"
#define EXACT "exact"

class PoseNode {
public:
    bool Prepare();
    void RunContinuously();
    void Shutdown();
    ~PoseNode() { };

private: 
    ros::NodeHandle *rosnode;
    ros::Subscriber odomSub;
    ros::Publisher posePub;
    
    double x;
    double y;
    double yaw;
    double last_msg_time;

    std::string type;

    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);
    void eulerIntegration(double linear, double angular, double time_step, nav_msgs::Odometry &msg);
    void rungeKuttaIntegration(double linear, double angular, double time_step, nav_msgs::Odometry &msg);
    void exactIntegration(double linear, double angular, double time_step, nav_msgs::Odometry &msg);
    void initPoseMsg(nav_msgs::Odometry &msg);
    void setPoseValues(nav_msgs::Odometry &msg, double linear, double angular);
    void initPoseValues();

};

#endif /* ODOM_NODE_H */