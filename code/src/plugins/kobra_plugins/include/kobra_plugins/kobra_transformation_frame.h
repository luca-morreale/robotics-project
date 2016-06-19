#ifndef KOBRA_TF_H
#define KOBRA_TF_H

#include <map>

#include <gazebo/gazebo.hh>
#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>
#include <sdf/sdf.hh>

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <boost/bind.hpp>

#include <gazebo/transport/transport.hh>


namespace gazebo {

    #define PAN "pan"
    #define TILT "tilt"
    #define BASE "base"

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
        static const MapString frames_tag;
        MapString joints_name;
        MapString frames;
        MapJoint joints;

        physics::ModelPtr model;
        event::ConnectionPtr update_connection;

        ros::NodeHandle *node;
        ros::Publisher pub;
        tf::TransformBroadcaster broadcaster;
        
        ros::Time current_time;
        ros::Time last_update_time;
        double update_period;


        tf::Transform buildTransform(math::Pose pose);
        tf::Transform buildRelativeTransform(math::Pose parent, math::Pose child);
        tf::Transform buildLaserTransform();
        tf::Transform buildCameraTransform();
        tf::Vector3 buildOrigin(math::Pose pose);
        tf::Vector3 buildOrigin(math::Vector3 pose);
        tf::Quaternion buildQuaternion(math::Pose pose);
        tf::Quaternion buildQuaternion(math::Quaternion rot);

        void extractJoints(sdf::ElementPtr _sdf);
        void extractFrames(sdf::ElementPtr _sdf);
        bool existsTags(sdf::ElementPtr _sdf);
    };

    GZ_REGISTER_MODEL_PLUGIN(TFKobraPlugin)
}

#endif /* KOBRA_TF_H */