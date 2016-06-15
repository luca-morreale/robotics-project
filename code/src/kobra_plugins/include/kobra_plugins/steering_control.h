
#ifndef STEERING_CONTROL_H
#define STEERING_CONTROL_H

#include <algorithm>
#include <assert.h>

#include <map>

#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>
#include <sdf/sdf.hh>

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/GetMap.h>
#include <nav_msgs/Odometry.h>
#include <boost/bind.hpp>
#include <boost/thread/mutex.hpp>


namespace gazebo
{

    #define N_JOINTS 4
    #define ROS_NODE_NAME "SteeringControlPlugin" 
    #define RIGHT_FRONT "right_front"
    #define LEFT_FRONT "left_front"
    #define RIGHT_REAR "right_rear"
    #define LEFT_REAR "left_rear"

    typedef std::map<std::string, std::string> MapString;
    typedef MapString::const_iterator MapStrConstIterator;
    typedef std::map<std::string, physics::JointPtr> MapJoint;
    typedef MapJoint::const_iterator MapJointIterator;
    typedef std::map<std::string, double> MapDouble;

    class SteeringControlPlugin : public ModelPlugin {
    public:
        SteeringControlPlugin() : ModelPlugin() { }
        ~SteeringControlPlugin();

        void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);

    protected:
        virtual void update();
        virtual void commandCallback(const geometry_msgs::Twist::ConstPtr &cmd_msg);

    protected:
        //Gazebo
        physics::ModelPtr parent;
        event::ConnectionPtr update_connection;

        //ROS
        ros::NodeHandle *rosnode;
        ros::Subscriber cmd_sub;
        ros::Publisher odom_pub;
        nav_msgs::Odometry odom_msg;

        static const MapString joints_name_tag;
        static const std::string odometry_topic_tag;
        static const std::string command_tag;
        static const std::string wheel_separation_tag;
        static const std::string wheel_diameter_tag;

        MapString joints_name;
        MapJoint joints;

        std::string odometry_topic;
        std::string command_topic;

        MapDouble wheel_speed;
        double yaw;
        double wheel_separation;
        double wheel_diameter;

        ros::Time current_time;
        ros::Time last_update_time;

        void setDefaultValues();
        bool extractJoints(sdf::ElementPtr _sdf);
        void extractRobotInfo(sdf::ElementPtr _sdf);
        void extractOdomInfo(sdf::ElementPtr _sdf);
        void extractCmdTopic(sdf::ElementPtr _sdf);

        void setWheelVelocity();
        void calculateWheelVelocity(double linear, double angular);
        
        void publishOdometry(double step_time);
        void getWheelVelocity(MapDouble *vel);
    };

GZ_REGISTER_MODEL_PLUGIN(SteeringControlPlugin)
}

#endif /* STEERING_CONTROL_H */
