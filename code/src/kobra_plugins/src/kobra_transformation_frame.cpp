
#include "kobra_plugins/kobra_transformation_frame.h"

using namespace gazebo;

const MapString TFKobraPlugin::joints_name_tag = {{PAN, "panJoint"}, {TILT, "tiltJoint"}};

void TFKobraPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
    if (!ros::isInitialized()) {
        ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin.");
        return;
    }

    this->model = _model;
    this->update_connection = event::Events::ConnectWorldUpdateBegin(boost::bind(&TFKobraPlugin::update, this));
    
    update_period = 2.0;
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

    broadcaster.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "/base_link"));

    tf::Transform laser_transform;
    laser_transform.setOrigin(tf::Vector3(0.29, -0.15, 0.10));
    broadcaster.sendTransform(tf::StampedTransform(laser_transform, ros::Time::now(), "base_link", "/laser_sensor"));

    math::Pose tilt_pose = joints[TILT]->GetWorldPose();
    math::Quaternion tilt_rot = tilt_pose.rot * pose.rot.GetInverse(); // missing rotation
    tf::Transform tilt_transform;
    tilt_transform.setOrigin(tf::Vector3(0.211, -0.15, 1.0975));
    tilt_transform.setRotation(tf::Quaternion(tilt_rot.w, tilt_rot.x, tilt_rot.y, tilt_rot.z));
    broadcaster.sendTransform(tf::StampedTransform(tilt_transform, ros::Time::now(), "base_link", "/tilt_joint"));

    math::Pose pan_pose = joints[PAN]->GetChild()->GetRelativePose();
    tf::Transform pan_transform;
    tf::Quaternion pan_quaternion;
    // missing position
    //pan_quaternion.setRPY(0.0, 2.0943, 0.0);
    //pan_transform.setRotation(pan_quaternion);
    pan_transform.setOrigin(tf::Vector3(pan_pose.pos.x, pan_pose.pos.y, pan_pose.pos.z));
    pan_transform.setRotation(tf::Quaternion(pan_pose.rot.w, pan_pose.rot.x, pan_pose.rot.y, pan_pose.rot.z));
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

void TFKobraPlugin::extractJoints(sdf::ElementPtr _sdf) 
{
    for(MapStrConstIterator it = joints_name_tag.begin(); it != joints_name_tag.end(); ++it) {
        //it->first PAN or TILT
        //it->second tag 
        joints_name[it->first] = _sdf->GetElement(it->second)->Get<std::string>();
        joints[it->first] = this->model->GetJoint(joints_name[it->first]);
    }
}

TFKobraPlugin::~TFKobraPlugin()
{
    delete node;
}