#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/PoseStamped.h"
#include <math.h>

#define RUN_PERIOD_DEFAULT 0.1
#define NAME_OF_THIS_NODE "kobra_pose_node"

class PoseNode {
private: 
    ros::NodeHandle *rosnode;
    ros::Subscriber odomSub;
    ros::Publisher posePub;
    geometry_msgs::PoseStamped pose_msg;
    
    double x;
    double y;
    double yaw;
    double pitch;
    double roll;
    double last_msg_time;

    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);
    void eulerIntegration(double linear, double angular, double time_step);
    void rungeKuttaIntegration(double linear, double angular, double time_step);
    void exactIntegration(double linear, double angular, double time_step);
    void initPoseMsg();
    void initPoseValues();

public:
    bool Prepare();
    void RunContinuously();
    void Shutdown();
    ~PoseNode(){};
};