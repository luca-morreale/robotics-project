
#include "kobra_plugins/kobra_transformation_frame.h"

using namespace gazebo;

void TFKobraPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
    if (!ros::isInitialized()) {
        ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin.");
        return;
    }

    this->model = _model;
    this->update_connection = event::Events::ConnectWorldUpdateBegin(boost::bind(&TFKobraPlugin::update, this));
    
    update_period = 1.0;
    node = new ros::NodeHandle();
    pub = node->advertise<geometry_msgs::Pose>("/robot_gt", 1);
}


void TFKobraPlugin::update()
{
    current_time = ros::Time::now();
    double time_step = (current_time - last_update_time).toSec();

    if (time_step > update_period) {
        publishTF();
        publishDebugTF();
        last_update_time = ros::Time::now();
    }
}

void TFKobraPlugin::publishTF()
{    
    
    math::Pose pose = model->GetWorldPose();
    
    tf::Transform transform;
    transform.setOrigin(tf::Vector3(pose.pos.x, pose.pos.y, pose.pos.z));
    transform.setRotation(tf::Quaternion(pose.rot.w, pose.rot.x, pose.rot.y, pose.rot.z));

    broadcaster.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "/basis_link"));

    tf::Transform laser_transform;

    laser_transform.setOrigin(tf::Vector3(0.29, -0.15, 0.10));
    broadcaster.sendTransform(tf::StampedTransform(laser_transform, ros::Time::now(), "base_link", "/laser_sensor"));

    tf::Transform tilt_transform;
    tilt_transform.setOrigin(tf::Vector3(0.211, -0.15, 1.0975));
    // missing rotation
    broadcaster.sendTransform(tf::StampedTransform(tilt_transform, ros::Time::now(), "base_link", "/tilt_joint"));

    tf::Transform pan_transform;
    // missing position
    // missing rotation
    broadcaster.sendTransform(tf::StampedTransform(pan_transform, ros::Time::now(), "tilt_joint", "/pan_joint"));

    tf::Transform camera_transform;
    tf::Quaternion camera_quaternion;
    camera_transform.setOrigin(tf::Vector3(0.007, 0.0, -0.0025));
    camera_quaternion.setRPY(0.0, -1.57071, 0.0);
    camera_transform.setRotation(camera_quaternion);
    broadcaster.sendTransform(tf::StampedTransform(camera_transform, ros::Time::now(), "pan_joint", "/camera_sensor"));
}


void TFKobraPlugin::publishDebugTF()
{
    math::Pose pose = model->GetWorldPose();

    geometry_msgs::Pose out_pose;
    out_pose.position.x = pose.pos.x;
    out_pose.position.y = pose.pos.y;
    out_pose.position.z = pose.pos.z;

    out_pose.orientation.x = pose.rot.x;
    out_pose.orientation.y = pose.rot.y;
    out_pose.orientation.z = pose.rot.z;
    out_pose.orientation.w = pose.rot.w;

    pub.publish(out_pose);
}

TFKobraPlugin::~TFKobraPlugin()
{
    delete node;
}