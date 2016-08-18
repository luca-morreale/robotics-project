#ifndef KOBRA_JOYSTICK_H
#define KOBRA_JOYSTICK_H


#include <map>
#include <termios.h>
#include <signal.h> 

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>

#include "kobra_msgs/ptz_msg.h"


namespace gazebo {

    #define KEYBOARD 0
    #define PAN      "pan"
    #define TILT     "tilt"
    #define LINEAR   "linear"
    #define ANGULAR  "angular"

    #define RIGHT_ARROW  0x43 
    #define LEFT_ARROW   0x44
    #define UP_ARROW     0x41
    #define DOWN_ARROW   0x42

    #define TILT_UP      'w'
    #define TILT_DOWN    's'
    #define PAN_LEFT     'a'
    #define PAN_RIGHT    'd'

    #define FZERO 0.0
    #define DEFAULT_SCALE 1.0
    #define DOT_FIVE 0.5


    typedef struct Command {

        double kobra_omega;
        double kobra_linear;
        double camera_pan;
        double camera_tilt;
        bool validity;

    public:
        Command();
        bool isKobra();
        bool isPanTilt();

    } Command;

    class KeyboardReader {
    public:
        KeyboardReader();
        ~KeyboardReader() {}

        void keyLoop();

    protected:
        virtual void transformCommand(char c, Command *cmd);
        virtual void initScaleFactors();

    private:
        ros::NodeHandle *rosnode;
        ros::Publisher velocity_pub;
        ros::Publisher pantilt_pub;

        double l_scale, w_scale;
        double p_scale, t_scale;

        std::map<std::string, double> velocities;

        struct termios raw_settings;

        void getRawKeyboard();
    };
}

#endif /* KOBRA_JOYSTICK_H */
