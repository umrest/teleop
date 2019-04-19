/** 
 * @file Controller.h
 * @brief Class definition for Controller
 * @author Jack Shelata
 * @date April 18, 2019
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
 * Controller class. This class is a subclass of TeleopBase class.
 * It handles the controller input and transmits control data as unsigned
 * 32 bit integers to the Rover Computer on the '/conrol' topic.
 */ 
class Controller : public TeleopBase{
  public:

    /**
     * Constructor for Controller. Takes in a ros::NodeHandle and creates
     * the subscriber. Initializes all private variables.
     * @param nh_in - ros::NodeHandle used to create subscriber
     */
    Controller(ros::NodeHandle& nh_in);
    /**
     * Member function updates the data inherited variable with the current
     * input values.
     */
    void updateAll();
    /**
     * Member function sets all controls to 0.
     * @param send - If true, sends reset message to rover. Useful when
     * deactivating controls. Prevents continuous motion if controls are
     * deactivated while movement commands are being given.
     */
    void resetControls(bool send);
    /**
     * Member function sets a specific input by index and value.
     * @param index - Input index to update
     * @param value - Value to update at the specified index
     */
    void setInput(int index, int value);
    /**
     * Member function sends std_msgs::UInt32 msg if the data has changed
     * since the last message was sent.
     * @param force - If true forces send even is message is unchanged.
     */
    void sendMessage(bool force);
    /**
     * Member function which reads controll input and transmits until the
     * end.
     */
    void readAndTransmit();

  private:

    ControlInputs inputs[19];
    bool active;
    bool debug;
    bool useful;
    ros::NodeHandle nh;
    ros::Publisher control_pub;
    std_msgs::UInt32 msg;
    std::string inputNames[19] = {"A","B","X","Y","Left Bumper","Right Bumper",
                                "Back","Start","Center","Left Joytick Button",
                                "Right Joystick Button","Up","Down","Left",
                                "Right", "Left Joystick","Right Joystick",
                                "Left Trigger","Right Trigger"};
};

/**
 * Helper function to decrease resolution to signed 4 bits.
 * @param value - Integer number to reduce
 * @return 4 bit scaled down integer
 */
int reduce(int value);