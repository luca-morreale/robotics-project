#include "kobra_joystick/kobra_joystick.h"

#include <iostream>

using namespace gazebo;

struct termios old_settings;

KeyboardReader::KeyboardReader()
{
        rosnode = new ros::NodeHandle();

        initScaleFactors();

        velocity_pub = rosnode->advertise<geometry_msgs::Twist>("kobra/cmd_vel", 1);
        pantilt_pub = rosnode->advertise<kobra_plugins::ptz_msg>("kobra/ptz", 1);
}

void KeyboardReader::initScaleFactors()
{
    rosnode->param("linear_scale", l_scale, DEFAULT_SCALE);
    rosnode->param("omega_scale", w_scale, DEFAULT_SCALE);
    rosnode->param("pan_scale", p_scale, DEFAULT_SCALE);
    rosnode->param("tilt_scale", t_scale, DEFAULT_SCALE);

    rosnode->param("linear_vel", velocities[LINEAR], DEFAULT_SCALE);
    rosnode->param("angular_vel", velocities[ANGULAR], DEFAULT_SCALE);
    rosnode->param("pan_degree", velocities[PAN], 0.5);
    rosnode->param("tilt_degree", velocities[TILT], 0.5);
}

void KeyboardReader::keyLoop()
{
    getRawKeyboard();

    ROS_INFO("Reading commands from your keyboard");
    ROS_INFO("---------------------------");
    ROS_INFO("Use A and D to pan, use W and S to tilt.");
    ROS_INFO("---------------------------");
    ROS_INFO("Use arrow keys to move the robot.");

    char c;

    while(ros::ok()) {

        if(read(KEYBOARD, &c, 1) < 0) {
            std::cerr << "read() error: " << c << std::endl;
            exit(-1);
        }

        Command cmd;
        transformCommand(c, &cmd);
     
        geometry_msgs::Twist twist_cmd;
        twist_cmd.angular.z = w_scale * cmd.kobra_omega;
        twist_cmd.linear.x = l_scale * cmd.kobra_linear;

        kobra_plugins::ptz_msg pantilt_cmd;
        pantilt_cmd.pan = cmd.camera_pan;
        pantilt_cmd.tilt = cmd.camera_tilt;

        if(cmd.isKobra()) {
            velocity_pub.publish(twist_cmd);            
        } else if(cmd.isPanTilt()) {
            pantilt_pub.publish(pantilt_cmd);
        }

    }
}

// DEBUG TO BE REMOVED AFTER TESTING
void KeyboardReader::transformCommand(char c, Command *cmd)
{
    switch(c) {
        case LEFT_ARROW:
            ROS_WARN("MOVE LEFT");
            cmd->kobra_omega = velocities[ANGULAR];
            break;
        case RIGHT_ARROW:
            ROS_WARN("MOVE RIGHT");
            cmd->kobra_omega = -velocities[ANGULAR];
            break;
        case UP_ARROW:
            ROS_WARN("MOVE FORWARD");
            cmd->kobra_linear = -velocities[LINEAR];
            break;
        case DOWN_ARROW:
            ROS_WARN("MOVE BACKWARD");
            cmd->kobra_linear = velocities[LINEAR];
            break;
        case TILT_UP:
            ROS_WARN("TILT UP");
            cmd->camera_tilt = velocities[TILT];
            break;
        case TILT_DOWN:
            ROS_WARN("TILT DOWN");
            cmd->camera_tilt = -velocities[TILT];
            break;
        case PAN_LEFT:
            ROS_WARN("PAN LEFT");
            cmd->camera_pan = velocities[PAN];
            break;
        case PAN_RIGHT:
            ROS_WARN("PAN RIGHT");
            cmd->camera_pan = -velocities[PAN];
            break;
        default:
            ROS_WARN("NOTHING");
            cmd->validity = false;
    }
}

void KeyboardReader::getRawKeyboard() 
{
    tcgetattr(KEYBOARD, &old_settings);                             /* Get the console settings                 */
    memcpy(&raw_settings, &old_settings, sizeof(struct termios));   /* Clone keyboard settings                  */
    raw_settings.c_lflag &= ~(ICANON | ECHO);                       /* Disable buffered i/o and set echo mode   */
                        
    raw_settings.c_cc[VEOL] = 1;                                    /* Set additional End-Of-Line character     */
    raw_settings.c_cc[VEOF] = 2;                                    /* Sets End-Of-File special character       */

    tcsetattr(KEYBOARD, TCSANOW, &raw_settings);                    /* Use these new terminal i/o settings      */
}

Command::Command() {
    kobra_omega = FZERO;
    kobra_linear = FZERO;
    camera_pan = FZERO;
    camera_tilt = FZERO;
    validity = true;
}

bool Command::isPanTilt() {
    return validity && (this->camera_pan != FZERO || this->camera_tilt != FZERO);
}
        
bool Command::isKobra() {
    return validity && (this->kobra_omega != FZERO || this->kobra_linear != FZERO);
}

using namespace std;

#include <signal.h>

void quit(int sig);

int main(int argc, char** argv)
{
    ros::init(argc, argv, "kobra_keyboard");
    KeyboardReader joystick;

    signal(SIGINT, quit);
    joystick.keyLoop();
    
    return 0;
}

void quit(int sig)
{
    (void)sig;
    ros::shutdown();
    tcsetattr(KEYBOARD, TCSANOW, &old_settings);                    /* Set console settings back to standard   */
    exit(0);
}
