#ifndef PANTILT_CAMERA_H
#define PANTILT_CAMERA_H

#include <algorithm>
#include <assert.h>

#include <gazebo_plugins/gazebo_ros_skid_steer_drive.h>

#include <gazebo/plugins/CameraPlugin.hh>

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

/*
<panJoint> camera_support_joint orizzontale
<tiltJoint> camera_junction_sphere_joint verticale
<topicName>/kobra/ptz
<cameraName>
*/


namespace gazebo
{
    class PantTiltCameraPlugin : public ModelPlugin {
    public:
        PantTiltCameraPlugin() : ModelPlugin() { }
        ~PantTiltCameraPlugin();

        void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);

        void update();
        void pantiltCallback(const geometry_msgs::Twist::ConstPtr& cmd_msg);

        virtual void setZoom(float z) { }

    private:
        physics::ModelPtr parent;
        event::ConnectionPtr update_connection;
        ros::NodeHandle *rosnode;
        ros::Subscriber sub;

        std::string pan_joint_name;
        std::string tilt_joint_name;
        std::string topic_name;
        std::string camera_name;
        rendering::CameraPtr camera;

        physics::JointPtr pan_joint;
        physics::JointPtr tilt_joint;


    
        bool checkTags(sdf::ElementPtr _sdf);

    };

GZ_REGISTER_MODEL_PLUGIN(PantTiltCameraPlugin)
}

#endif /* PANTILT_CAMERA_H */