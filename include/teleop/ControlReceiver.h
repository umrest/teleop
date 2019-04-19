/** 
 * @file ControlReceiver.h
 * @brief Class definition for ControlReceiver
 * @author Jack Shelata
 * @date April 19, 2019
 */

#include "ros/ros.h"
#include "std_msgs/UInt32.h"
#include "TeleopBase.h"
#include <string>

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

  private:
    ControlInputs inputs[19];
    ros::NodeHandle nh;
    ros::Subscriber control_sub;
    std::string inputNames[19] = {"A","B","X","Y","Left Bumper","Right Bumper",
                                "Back","Start","Center","Left Joytick Button",
                                "Right Joystick Button","Up","Down","Left",
                                "Right", "Left Joystick","Right Joystick",
                                "Left Trigger","Right Trigger"};
};