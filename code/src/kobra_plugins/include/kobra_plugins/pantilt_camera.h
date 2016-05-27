#ifndef PANTILT_CAMERA_H
#define PANTILT_CAMERA_H

#include <algorithm>
#include <map>

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

#include "kobra_plugins/ptz_msg.h"

/*
<panJoint> camera_support_joint orizzontale
<tiltJoint> camera_junction_sphere_joint verticale
<topicName>/kobra/ptz
<cameraName>
*/


namespace gazebo
{
    #define N_JOINTS 2
    #define DEFAULT_PAN_VEL 0.1
    #define DEFAULT_TILT_VEL 0.1
    #define PAN "pan"
    #define TILT "tilt"

    typedef std::map<std::string, std::string> MapString;
    typedef MapString::const_iterator MapStrConstIterator;

    class PantTiltCameraPlugin : public ModelPlugin {
    public:
        PantTiltCameraPlugin() : ModelPlugin() { }
        ~PantTiltCameraPlugin();

        void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);

    protected:
        void update();
        virtual void pantiltCallback(const kobra_plugins::ptz_msg::ConstPtr &msg);
        virtual void setZoom(float z) { } //advertise to camera setting and publish info! http://wiki.ros.org/camera_calibration

        void startTilting(const ros::TimerEvent& /* evt */, float tilt_degree);

    private:
        static const MapString joints_name_tag;

        physics::ModelPtr parent;
        event::ConnectionPtr update_connection;
        ros::NodeHandle *rosnode;
        ros::Subscriber sub;

        MapString joints_name;
        std::string topic_name;
        std::string camera_name;

        std::map<std::string, physics::JointPtr> joints;
        float pan_velocity = DEFAULT_PAN_VEL;
        float tilt_velocity = DEFAULT_TILT_VEL;


        bool checkTags(sdf::ElementPtr _sdf);
        bool checkJointsTag(sdf::ElementPtr _sdf);
        bool checkTopicTags(sdf::ElementPtr _sdf);
        
        void extractJoints(sdf::ElementPtr _sdf);
        void extractNames(sdf::ElementPtr _sdf);
        void extractVelocities(sdf::ElementPtr _sdf);

    };

GZ_REGISTER_MODEL_PLUGIN(PantTiltCameraPlugin)
}

#endif /* PANTILT_CAMERA_H */