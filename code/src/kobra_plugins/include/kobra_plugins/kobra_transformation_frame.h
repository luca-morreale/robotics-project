#ifndef KOBRA_TF_H
#define KOBRA_TF_H

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

#include <tf/transform_broadcaster.h>
#include <turtlesim/Pose.h>



namespace gazebo {

    #define PAN "pan"
    #define TILT "tilt"

    typedef std::map<std::string, std::string> MapString;
    typedef std::map<std::string, physics::JointPtr> MapJoint;
    typedef MapString::const_iterator MapStrConstIterator;

    class TFKobraPlugin : public ModelPlugin {
    public:
        TFKobraPlugin() : ModelPlugin() { }
        ~TFKobraPlugin();

        void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
        void update();
    
    protected:
        virtual void publishTF();
        virtual void publishDebugTF();


    private:
        static const MapString joints_name_tag;
        MapString joints_name;

        physics::ModelPtr model;
        event::ConnectionPtr update_connection;

        ros::NodeHandle *node;
        ros::Publisher pub;

        MapJoint joints;
        tf::TransformBroadcaster broadcaster;
        
        ros::Time current_time;
        ros::Time last_update_time;
        double update_period;

        void extractJoints(sdf::ElementPtr _sdf);

        tf::Transform buildTransform(math::Pose pose);
        tf::Transform buildRelativeTransform(math::Pose parent, math::Pose child);
        tf::Transform buildLaserTransform();
        tf::Transform buildCameraTransform();
        tf::Vector3 buildOrigin(math::Pose pose);
        tf::Vector3 buildOrigin(math::Vector3 pose);
        tf::Quaternion buildQuaternion(math::Pose pose);
        tf::Quaternion buildQuaternion(math::Quaternion rot);
    };

    GZ_REGISTER_MODEL_PLUGIN(TFKobraPlugin)
}

#endif /* KOBRA_TF_H */
