#include "kobra_plugins/steering_control.h"

using namespace gazebo;

/*Tag names used in model.sdf*/

const MapString joints_name_tag = {{LEFT_FRONT, "LeftFrontJoint"}, {RIGHT_FRONT, "RightFrontJoint"}, {LEFT_REAR, "LeftRearJoint"}, {RIGHT_REAR, "RightRearJoint"}};
const std::string odometry_topic_tag = "OdometryTopic";
const std::string odometry_frame_tag = "OdometryFrame";
const std::string robot_base_frame_tag = "robotBaseFrame";
const std::string command_tag = "CmdTopic";
const std::string wheel_separation_tag = "wheelSeparation";
const std::string update_rate_tag = "updateRate";

void SteeringControlPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
    if (!ros::isInitialized()) {
      ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin.");
      return;
    }

    this->parent = _model;  
    this->update_connection = event::Events::ConnectWorldUpdateBegin(boost::bind(&SteeringControlPlugin::update, this));

    setDefaultValues();

    //Extract informations from the sdf file
    if(!extractJoints(_sdf)) {
        return;
    }
    extractRobotInfo(_sdf);
    extractOdomInfo(_sdf);
    extractCmdTopic(_sdf);

    rosnode = new ros::NodeHandle();

    ROS_DEBUG("Starting " ROS_NODE_NAME " ...");

    cmd_sub = rosnode->subscribe(this->command_topic, 10, &SteeringControlPlugin::cmdCallback, this);
    odom_pub = rosnode->advertise<nav_msgs::Odometry>(this->odometry_topic, 1);
    
}

void SteeringControlPlugin::setDefaultValues()
{
    //Kobra dimesnions
    wheel_diameter=0.145;
    wheel_separation=0.495;
    update_rate = 100.0;
    //Init velocities
    wheel_speed[RIGHT_FRONT] = 0;
    wheel_speed[LEFT_FRONT] = 0;
    wheel_speed[RIGHT_REAR] = 0;
    wheel_speed[LEFT_REAR] = 0;
    linear_vel=0;
    angular_vel=0;
    //Time between two consecutive updates
    update_period = 1/update_rate;
    //Initilize last update to current time
    last_update_time = this->world->GetSimTime();
}

void SteeringControlPlugin::update() 
{
    current_time = this->world->GetSimTime();
    double seconds_since_last_update = (current_time - last_update_time).Double();

    if (seconds_since_last_update > update_period) {
    	publishOdometry(seconds_since_last_update);
    	//Update the time of the last update
    	last_update_time = this->world->GetSimTime();
    }
}

void SteeringControlPlugin::cmdCallback(const geometry_msgs::Twist::ConstPtr& cmd_msg)
{
    linear_vel = cmd_msg->linear.x;
    angular_vel = cmd_msg->angular.z;
}


bool SteeringControlPlugin::extractJoints(sdf::ElementPtr _sdf)
{
    for(MapStrConstIterator it = joints_name_tag.begin(); it != joints_name_tag.end(); ++it) {
        //it->first PAN or TILT (key)
        //it->second tag (value)
        if(!_sdf->HasElement(it->second)) {
            ROS_FATAL_STREAM(ROS_NODE_NAME " missing "+ it->second);
            return false;
        } else {
            joints_name[it->first] = _sdf->GetElement(it->second)->Get<std::string>();
            joints[it->first] = this->parent->GetJoint(joints_name[it->first]);
        }
    }
    return true;
}

void SteeringControlPlugin::extractRobotInfo(sdf::ElementPtr _sdf)
{
    if (!_sdf->HasElement(wheel_separation_tag)) {
    	ROS_WARN(ROS_NODE_NAME " missing <%s>, defaults to %s.", this->wheel_separation_tag, this->wheel_separation);
    } else {
      this->wheel_separation= _sdf->GetElement(wheel_separation_tag)->Get<double>();
    }

    if (!_sdf->HasElement(wheel_diameter_tag)) {
    	ROS_WARN(ROS_NODE_NAME " missing <%s>, defaults to %s.", this->wheel_diameter_tag, this->wheel_diameter);
    } else {
      this->wheel_diameter= _sdf->GetElement(wheel_diameter_tag)->Get<double>();
    }

    if (!_sdf->HasElement(robot_base_frame_tag)) {
        ROS_WARN(ROS_NODE_NAME " missing <%s>, defaults to \"%s\"", this->robot_base_frame_tag, this->robot_base_frame);
    } else {
      this->robot_base_frame = _sdf->GetElement(robot_base_frame_tag)->Get<std::string>();
    }

    if (!_sdf->HasElement(update_rate_tag)) {
      ROS_WARN(ROS_NODE_NAME " missing <%s>, defaults to %f", this->update_rate_tag, this->update_rate);
    } else {
      this->update_rate = _sdf->GetElement(update_rate_tag)->Get<double>();
    }
}

void SteeringControlPlugin::extractOdomInfo(sdf::ElementPtr _sdf)
{    
    if (!_sdf->HasElement(odometry_topic_tag)) {
        this->odometry_topic = "odom";
        ROS_WARN(ROS_NODE_NAME " missing <%s>, defaults to %s.", this->odometry_topic_tag, this->odometry_topic);
    } else {
        this->odometry_topic = _sdf->GetElement(odometry_topic_tag)->Get<std::string>();
    }

    
    if (!_sdf->HasElement("odometryFrame")) {
        this->odometry_frame = "odom";
        ROS_WARN(ROS_NODE_NAME " missing <%s>, defaults to %s.", this->odometry_frame_tag, this->odometry_frame);
    } else {
        this->odometry_frame = _sdf->GetElement(odometry_frame_tag)->Get<std::string>();
    }
}

void SteeringControlPlugin::extractCmdTopic(sdf::ElementPtr _sdf)
{
    if (!_sdf->HasElement("commandTopic")) {
        this->command_topic = "cmd_vel";
        ROS_WARN(ROS_NODE_NAME " missing <%s>, defaults to %s.", this->command_tag, this->command_topic);
    } else {
        this->command_topic = _sdf->GetElement(command_tag)->Get<std::string>();
    }
}

void SteeringControlPlugin::getWheelVelocitiesFromCmdVel()
{
	wheel_speed[RIGHT_FRONT] = linear_vel + angular_vel*(wheel_separation/2);
	wheel_speed[RIGHT_REAR] = linear_vel + angular_vel*(wheel_separation/2);
	wheel_speed[LEFT_FRONT] = linear_vel - angular_vel*(wheel_separation/2);
	wheel_speed[LEFT_REAR] = linear_vel - angular_vel*(wheel_separation/2);

	ROS_INFO("RIGHT wheels speed is now %lf", wheel_speed[RIGHT_FRONT]);
	ROS_INFO("LEFT wheels speed is now %lf", wheel_speed[LEFT_FRONT]);
}

void SteeringControlPlugin::getCmdVelocitiesFromWheels()
{

}

void SteeringControlPlugin::publishOdometry(double step_time)
{
    //TODO
    //Euler integration
    double dt = step_time;
    double d_x =  linear.x * dt;
    double d_y =  linar.y * dt;
    double d_th = angular.z * dt;

    double new_x=d_x;
    double new_y=;
    double new_th=;

    odom_msg->
    odom_pub.publish(odom_msg);
}


SteeringControlPlugin::~SteeringControlPlugin()
{
    delete rosnode;
}
