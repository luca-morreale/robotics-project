#include "kobra_joystick/kobra_joystick.h"

#include <iostream>

using namespace gazebo;

KeyboardReader::KeyboardReader()
{
        rosnode = new ros::NodeHandle();

        initScaleFactors();

        velocity_pub = rosnode->advertise<geometry_msgs::Twist>("kobra/cmd_vel", 1);
        pantilt_pub = rosnode->advertise<geometry_msgs::Vector3>("kobra/camera_mv", 1);
}

void KeyboardReader::initScaleFactors()
{
    rosnode->param("linear_scale", l_scale, DEFAULT_SCALE);
    rosnode->param("omega_scale", w_scale, DEFAULT_SCALE);
    rosnode->param("pan_scale", p_scale, DEFAULT_SCALE);
    rosnode->param("tilt_scale", t_scale, DEFAULT_SCALE);

    rosnode->param("linear_vel", velocities[LINEAR], DEFAULT_SCALE);
    rosnode->param("angular_vel", velocities[ANGULAR], DEFAULT_SCALE);
    rosnode->param("pan_degree", velocities[PAN], DEFAULT_SCALE);
    rosnode->param("tilt_degree", velocities[TILT], DEFAULT_SCALE);
}

void KeyboardReader::keyLoop()
{
    getRawKeyboard();

    ROS_INFO("Reading commands from your keyboard");
    ROS_INFO("---------------------------");
    ROS_INFO("Use A and D to pan the camera, use W and S to tilt the camera.");
    ROS_INFO("---------------------------");
    ROS_INFO("Use arrow keys to move the robot.");

    char c;

    while(ros::ok()) {

        if(read(KEYBOARD, &c, 1) < 0) {
            std::cerr << "read() error: " << c << std::endl;
            exit(-1);
        }
      
        Command *cmd = transformCommand(c);
     
        geometry_msgs::Twist twist_cmd;
        twist_cmd.angular.z = w_scale * cmd->kobra_omega;
        twist_cmd.linear.x = l_scale * cmd->kobra_linear;

        geometry_msgs::Vector3 pantilt_cmd;
        pantilt_cmd.x = p_scale * cmd->camera_pan;
        pantilt_cmd.y = t_scale * cmd->camera_tilt;

        if(cmd->isKobra()) {
            velocity_pub.publish(twist_cmd);
            
        } else if(cmd->isPanTilt()) {
            pantilt_pub.publish(pantilt_cmd);
        }
    }

    return;
}

// DEBUG TO BE REMOVED AFTER TESTING
Command *KeyboardReader::transformCommand(char c)
{
    Command *cmd = (Command *) malloc(sizeof(Command));
    
    switch(c) {
        case LEFT_ARROW:
            ROS_DEBUG("MOVE LEFT");
            cmd->kobra_omega = velocities[ANGULAR];
            break;
        case RIGHT_ARROW:
            ROS_DEBUG("MOVE RIGHT");
            cmd->kobra_omega = -velocities[ANGULAR];
            break;
        case UP_ARROW:
            ROS_DEBUG("MOVE FORWARD");
            cmd->kobra_linear = velocities[LINEAR];
            break;
        case DOWN_ARROW:
            ROS_DEBUG("MOVE BACKWARD");
            cmd->kobra_linear = -velocities[LINEAR];
            break;
        case TILT_UP:
            ROS_DEBUG("TILT UP");
            cmd->camera_tilt = velocities[TILT];
            break;
        case TILT_DOWN:
            ROS_DEBUG("TILT DOWN");
            cmd->camera_tilt = -velocities[TILT];
            break;
        case PAN_LEFT:
            ROS_DEBUG("PAN LEFT");
            cmd->camera_pan = velocities[PAN];
            break;
        case PAN_RIGHT:
            ROS_DEBUG("PAN RIGHT");
            cmd->camera_pan = -velocities[PAN];
            break;
        default:
            cmd->validity = false;
    }
    return cmd;
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

KeyboardReader::~KeyboardReader()
{
    tcsetattr(KEYBOARD, TCSANOW, &old_settings);					/* Set console settings back to standard ones */
}




#include <signal.h>

void quit(int sig);

int main(int argc, char** argv)
{
    ros::init(argc, argv, "kobra_keyboard");
    KeyboardReader joystic;

    signal(SIGINT, quit);
    joystic.keyLoop();
    
    return 0;
}

void quit(int sig)
{
    (void)sig;
    ros::shutdown();
    exit(0);
}
