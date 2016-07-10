#ifndef ODOM_NODE_H
#define ODOM_NODE_H

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <math.h>

#define RUN_PERIOD_DEFAULT 0.1
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
    tf::TransformBroadcaster odom_broadcaster;
    
    double x;
    double y;
    double yaw;
    double last_msg_time;

    typedef void(PoseNode::*IntegrationFunction)(double linear, double angular, double time_step);

    IntegrationFunction integration;

    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);
    void eulerIntegration(double linear, double angular, double time_step);
    void rungeKuttaIntegration(double linear, double angular, double time_step);
    void exactIntegration(double linear, double angular, double time_step);
    void initPoseValues();

    void publishOdometryMessage(double linear, double angular, const nav_msgs::Odometry::ConstPtr& msg);
    void broadcastTransformation(const nav_msgs::Odometry::ConstPtr& old_msg);

};

#endif /* ODOM_NODE_H */