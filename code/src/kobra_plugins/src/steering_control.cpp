#include "kobra_plugin/steering_control.h"

using namespace gazebo;

/*Tag names used in model.sdf*/
const MapString joints_name_tag = {RIGHT_FRONT, 0}, {LEFT_FRONT, 1}, {RIGHT_REAR, 2}, {LEFT_REAR, 3}};
const std::string joints_tag_names[N_JOINTS] = {"LeftFrontJoint", "RightFrontJoint", "LeftRearJoint", "RightRearJoint"};
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

    // Set default values
    this->wheel_diameter=0.145;
    this->wheel_separation=0.495;
    this->robot_base_frame = "base_footprint";
    this->update_rate = 100.0;

    //Initilize last update to current time
    this->last_update_time = this->world->GetSimTime();

    //Extract informations from the sdf file
    if(!extractJoints(_sdf)) {
        return;
    }
    extractRobotInfo(_sdf);
    extractOdomInfo(_sdf);
    extractCmdTopic(_sdf);

    //Time between two consecutive updates
    this->update_period = 1/update_rate;

    rosnode = new ros::NodeHandle();

    ROS_INFO("Starting " ROS_NODE_NAME " ...");

    cmd_sub = rosnode->subscribe(this->command_topic, 10, &SteeringControlPlugin::cmdCallback, this);
    odom_pub = rosnode->advertise<nav_msgs::Odometry>(this->odometry_topic, 1);

    //Init velocities
    this->wheel_speed[RIGHT_FRONT] = 0;
    this->wheel_speed[LEFT_FRONT] = 0;
    this->wheel_speed[RIGHT_REAR] = 0;
    this->wheel_speed[LEFT_REAR] = 0;
    this->linear=0;
    this->angular=0;
    
}

void SteeringControlPlugin::update() 
{
    common::Time current_time = this->world->GetSimTime();
    double seconds_since_last_update = (current_time - last_update_time).Double();

    if (seconds_since_last_update > update_period) {
    	publishOdometry(seconds_since_last_update);
    	//Update the time of the last update
    	//last_update_time_+= common::Time(update_period);
    	last_update_time = this->world->GetSimTime();
    }
}

void SteeringControlPlugin::cmdCallback(const geometry_msgs::Twist::ConstPtr& cmd_msg)
{
    linear = cmd_msg->linear;
    angular = cmd_msg->angular;
}


bool SteeringControlPlugin::extractJoints(sdf::ElementPtr _sdf)
{
    for(int i = 0; i < N_JOINTS; i++){
    	if(!_sdf->HasElement(joints_tag_names[i])) {
            ROS_FATAL_STREAM(ROS_NODE_NAME " missing "+ this->joints_names[i]);
            return false;
        }
        else{
            joints_names[i] = _sdf->GetElement(joints_tag_names[i])->Get<std::string>();
            joints[i] = this->parent->GetJoint(joints_names[i]);
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

void SteeringControlPlugin::getWheelVelocities()
{
	wheel_speed[RIGHT_FRONT] = linear + angular*(wheel_separation/2);
	wheel_speed[RIGHT_REAR] = linear + angular*(wheel_separation/2);
	wheel_speed[LEFT_FRONT] = linear - angular*(wheel_separation/2);
	wheel_speed[LEFT_REAR] = linear - angular*(wheel_separation/2);

	ROS_INFO("RIGHT wheels speed is now %lf", wheel_speed[RIGHT_FRONT]);
	ROS_INFO("LEFT wheels speed is now %lf", wheel_speed[LEFT_FRONT]);
}

void SteeringControlPlugin::publishOdometry(double step_time)
{
    //TODO
    odom_pub.publish(odom_msg);
}


SteeringControlPlugin::~SteeringControlPlugin()
{
    delete rosnode;
}
