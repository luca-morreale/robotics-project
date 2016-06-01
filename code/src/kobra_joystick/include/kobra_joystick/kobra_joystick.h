#ifndef KOBRA_JOYSTICK_H
#define KOBRA_JOYSTICK_H


#include <map>
#include <termios.h>
#include <signal.h> 

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>


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

    #define TILT_UP      0x11
    #define TILT_DOWN    0x1f
    #define PAN_LEFT     0x1e
    #define PAN_RIGHT    0x20

    #define FZERO 0.0
    #define DEFAULT_SCALE 2.0


    typedef struct Command {

        double kobra_omega = FZERO;
        double kobra_linear = FZERO;
        double camera_pan = FZERO;
        double camera_tilt = FZERO;
        bool validity = true;

        bool isPanTilt() {
            return validity && (camera_pan > 0 || camera_tilt > 0);
        }
        
        bool isKobra() {
            return validity && (kobra_omega > 0 || kobra_linear > 0);
        }

    } Command;

    class KeyboardReader {
    public:
        KeyboardReader();
        ~KeyboardReader();

        void keyLoop();

    protected:
        virtual Command *transformCommand(char cmd);
        virtual void initScaleFactors();

    private:
        ros::NodeHandle *rosnode;
        ros::Publisher velocity_pub;
        ros::Publisher pantilt_pub;

        double l_scale, w_scale;
        double p_scale, t_scale;

        std::map<std::string, double> velocities;

        struct termios old_settings;
        struct termios raw_settings;

        void getRawKeyboard();
    };
}

#endif /* KOBRA_JOYSTICK_H */
