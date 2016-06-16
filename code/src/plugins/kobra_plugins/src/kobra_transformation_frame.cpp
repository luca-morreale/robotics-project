#include "../include/kobra_plugins/kobra_transformation_frame.h"

using namespace gazebo;

const MapString TFKobraPlugin::joints_name_tag = {{PAN, "panJoint"}, {TILT, "tiltJoint"}};
const MapString TFKobraPlugin::frames_tag = {{BASE, "baseFrame"}, {TILT, "tiltFrame"}, {PAN, "panFrame"}};


void TFKobraPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
    if (!ros::isInitialized()) {
        ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin.");
        return;
    }

    this->model = _model;
    this->update_connection = event::Events::ConnectWorldUpdateBegin(boost::bind(&TFKobraPlugin::update, this));
    
    update_period = 1.0 / 50.0;

    if(!existsTags(_sdf)) { return; }
    extractJoints(_sdf);
    extractFrames(_sdf);

    node = new ros::NodeHandle();
    pub = node->advertise<geometry_msgs::Pose>("/kobra_gt", 1);
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
    static tf::TransformBroadcaster broadcaster;
    math::Pose pose = model->GetWorldPose();
    broadcaster.sendTransform(tf::StampedTransform(this->buildTransform(pose), ros::Time::now(), "world", frames[BASE]));

    broadcaster.sendTransform(tf::StampedTransform(this->buildLaserTransform(), ros::Time::now(), frames[BASE], "laser_sensor"));

    math::Pose tilt_pose = joints[TILT]->GetWorldPose();
    broadcaster.sendTransform(tf::StampedTransform(this->buildRelativeTransform(pose, tilt_pose), ros::Time::now(), frames[BASE], frames[TILT]));

    math::Pose pan_pose = joints[PAN]->GetChild()->GetWorldPose();
    broadcaster.sendTransform(tf::StampedTransform(this->buildRelativeTransform(tilt_pose, pan_pose), ros::Time::now(), frames[TILT], frames[PAN]));

    broadcaster.sendTransform(tf::StampedTransform(this->buildCameraTransform(), ros::Time::now(), frames[PAN], "camera_sensor"));
}

tf::Transform TFKobraPlugin::buildTransform(math::Pose pose)
{
    tf::Transform transform;
    transform.setOrigin(buildOrigin(pose));
    transform.setRotation(buildQuaternion(pose));
    return transform;
}

tf::Transform TFKobraPlugin::buildRelativeTransform(math::Pose parent, math::Pose child)
{
    tf::Transform relative_transform;
    math::Quaternion relative_rot = child.rot * parent.rot.GetInverse();
    math::Vector3 relative_pose = child.pos - parent.pos;
    relative_pose.z *= -1;
    relative_rot.Normalize();
    relative_transform.setOrigin(buildOrigin(relative_pose));
    relative_transform.setRotation(buildQuaternion(relative_rot));
    return relative_transform;
}

tf::Transform TFKobraPlugin::buildLaserTransform()
{
    tf::Transform laser_transform;
    tf::Quaternion laser_quaternion;
    laser_transform.setOrigin(tf::Vector3(0.29, 0.0, 0.0));
    laser_quaternion.setRPY(0.0, 0.0, 0.0);
    laser_quaternion.normalize();
    laser_transform.setRotation(laser_quaternion);
    return laser_transform;
}

tf::Transform TFKobraPlugin::buildCameraTransform()
{
    tf::Transform camera_transform;
    tf::Quaternion camera_quaternion;
    camera_transform.setOrigin(tf::Vector3(0.0, 0.0, -0.0025));
    camera_quaternion.setRPY(0.0, -1.57071, 0.0);
    camera_quaternion.normalize();
    camera_transform.setRotation(camera_quaternion);
    return camera_transform;
}

tf::Vector3 TFKobraPlugin::buildOrigin(math::Pose pose)
{
    return tf::Vector3(pose.pos.x, pose.pos.y, pose.pos.z);
}

tf::Vector3 TFKobraPlugin::buildOrigin(math::Vector3 pose)
{
    return tf::Vector3(pose.x, pose.y, pose.z);
}

tf::Quaternion TFKobraPlugin::buildQuaternion(math::Pose pose)
{
    return tf::Quaternion(pose.rot.w, pose.rot.x, pose.rot.y, pose.rot.z);
}

tf::Quaternion TFKobraPlugin::buildQuaternion(math::Quaternion rot)
{
    return tf::Quaternion(rot.w, rot.x, rot.y, rot.z);
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
        joints_name[it->first] = _sdf->GetElement(it->second)->Get<std::string>();
        joints[it->first] = this->model->GetJoint(joints_name[it->first]);
    }
}

void TFKobraPlugin::extractFrames(sdf::ElementPtr _sdf) 
{
    for(MapStrConstIterator it = frames_tag.begin(); it != frames_tag.end(); ++it) {
        frames[it->first] = _sdf->GetElement(it->second)->Get<std::string>();
    }
}

bool TFKobraPlugin::existsTags(sdf::ElementPtr _sdf)
{
    for(MapStrConstIterator it = joints_name_tag.begin(); it != joints_name_tag.end(); ++it) {
        if(!_sdf->HasElement(it->second)) {
            ROS_FATAL_STREAM("TFKobraPlugin: <"+ it->first +"> missing!");
            return false;
        }
    }

    for(MapStrConstIterator it = frames_tag.begin(); it != frames_tag.end(); ++it) {
        if(!_sdf->HasElement(it->second)) {
            ROS_FATAL_STREAM("TFKobraPlugin: <"+ it->first +"> missing!");
            return false;
        }
    }
    return true;
}

TFKobraPlugin::~TFKobraPlugin()
{
    delete node;
}
