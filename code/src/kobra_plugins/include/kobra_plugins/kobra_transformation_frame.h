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
        physics::ModelPtr model;
        event::ConnectionPtr update_connection;

        ros::NodeHandle *node;
        ros::Publisher pub;

        tf::TransformBroadcaster broadcaster;
        
        ros::Time current_time;
        ros::Time last_update_time;
        double update_period;

    };

    GZ_REGISTER_MODEL_PLUGIN(TFKobraPlugin)
}

#endif /* KOBRA_TF_H */
