#ifndef PANTILT_CAMERA_H
#define PANTILT_CAMERA_H


#include <map>
#include <algorithm>

#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <ros/advertise_options.h>


#include <boost/thread.hpp>
#include <boost/bind.hpp>

#include "kobra_plugins/ptz_msg.h"


namespace gazebo
{
    #define DEFAULT_VEL 0.7
    #define DEFAULT_RADIUS 0.3
    #define ANGLE_GAP 1.0
    #define PAN "pan"
    #define TILT "tilt"

    typedef std::map<std::string, std::string> MapString;
    typedef std::map<std::string, physics::JointPtr> MapJoint;
    typedef MapString::const_iterator MapStrConstIterator;
    typedef std::map<std::string, double> MapDouble;

    class PantTiltCameraPlugin : public ModelPlugin {
    public:
        PantTiltCameraPlugin() : ModelPlugin() { }
        ~PantTiltCameraPlugin();

        void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);

    protected:
        virtual void update();
        virtual void pantiltCallback(const kobra_plugins::ptz_msg::ConstPtr &msg);
        virtual void setZoom(float z) { } //advertise to camera setting and publish info! http://wiki.ros.org/camera_calibration

    private:
        static const MapString joints_name_tag;
        static const MapString velocities_name_tag;
        static const MapString radius_name_tag;

        physics::ModelPtr parent;
        event::ConnectionPtr update_connection;
        ros::NodeHandle *rosnode;
        ros::Subscriber rossub;

        MapString joints_name;
        std::string topic_name;
        std::string camera_name;


        MapJoint joints;
        MapDouble velocity;
        MapDouble radius;
        MapDouble joint_velocity;

        bool checkTags(sdf::ElementPtr _sdf);
        bool checkJointsTag(sdf::ElementPtr _sdf);
        bool checkTopicTags(sdf::ElementPtr _sdf);
        
        void extractJoints(sdf::ElementPtr _sdf);
        void extractNames(sdf::ElementPtr _sdf);
        void extractVelocities(sdf::ElementPtr _sdf);
        void extractRadius(sdf::ElementPtr _sdf);

        void moveJoint(std::string JOINT, double degree, double sleep_time);
        double fixAngle(std::string JOINT, double degree);
    };

GZ_REGISTER_MODEL_PLUGIN(PantTiltCameraPlugin)
}

#endif /* PANTILT_CAMERA_H */