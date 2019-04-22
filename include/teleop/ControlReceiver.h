/** 
 * @file ControlReceiver.h
 * @brief Class definition for ControlReceiver
 * @author Jack Shelata
 * @date April 19, 2019
 */

#include "ros/ros.h"
#include "std_msgs/UInt32.h"
#include "teleop/ControlMsg.h"
#include "TeleopBase.h"
#include <string>

#define SCALE 25

#define A 0
#define B 1
#define X 2
#define Y 3
#define LB 4
#define RB 5
#define BACK 6
#define START 7
#define CENTER 8
#define LEFTJOYIN 9
#define RIGHTJOYIN 10
#define UP 11
#define DOWN 12
#define LEFT 13
#define RIGHT 14
#define LEFTJOY 15
#define RIGHTJOY 16
#define LT 17
#define RT 18


/***************************************************************
**                Class Definition
****************************************************************/

/**
 * Struct to hold the details of each input.
 */ 
struct ControlInputs {
  std::string name;
  int bitLength;
  int offset;
  int value;
  bool axis;
};

/** 
 * ControlReceiver class. This class is a subclass of TeleopBase class.
 * It decodes the Controller input from the '/control/ topic and passes
 * to serial node.
 */ 
class ControlReceiver : public TeleopBase{
  public:
    ControlReceiver(ros::NodeHandle& nh_in);
    void getAll();
    void handleControlMsg(const std_msgs::UInt32::ConstPtr& msg);
    void sendToSerial();

  private:
    ControlInputs inputs[19];
    ros::NodeHandle nh;
    ros::Subscriber control_sub;
    ros::Publisher serial_pub;
    std::string inputNames[19] = {"A","B","X","Y","Left Bumper","Right Bumper",
                                "Back","Start","Center","Left Joytick Button",
                                "Right Joystick Button","Up","Down","Left",
                                "Right", "Left Joystick","Right Joystick",
                                "Left Trigger","Right Trigger"};
};