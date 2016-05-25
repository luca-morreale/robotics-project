#ifndef STEERING_CONTROL_H
#define STEERING_CONTROL_H

#include <algorithm>
#include <assert.h>

#include <gazebo_plugins/gazebo_ros_skid_steer_drive.h>

#include <gazebo/math/gzmath.hh>
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

    class SteeringControlPlugin : public ModelPlugin {
    public:
        SteeringControlPlugin() : ModelPlugin() { }
        ~SteeringControlPlugin();

        void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);

        void update();
        void cmdCallback(const geometry_msgs::Twist::ConstPtr& cmd_msg);

    private:
        physics::ModelPtr parent;
        event::ConnectionPtr update_connection;
        ros::NodeHandle *rosnode;
        ros::Subscriber cmd_sub;
        ros::Publisher odom_pub;

        static const std::string joints_tag_names[N_JOINTS];
        static const std::string odometry_topic_tag;
        static const std::string odometry_frame_tag;
        static const std::string command_tag;

        std::string joints_names[N_JOINTS];
        physics::JointPtr joints[N_JOINTS];

        std::string odometry_topic;
        std::string odometry_frame;
        std::string command_topic;


        bool checkJoints(sdf::ElementPtr _sdf);
        void extractOdomInfo(sdf::ElementPtr _sdf);
        void extractCmdTopic(sdf::ElementPtr _sdf);


    };

GZ_REGISTER_MODEL_PLUGIN(SteeringControlPlugin)
}

#endif /* STEERING_CONTROL_H */