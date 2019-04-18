/** @file ControllerInput.h
 *  @brief Class declaration for ControllerInput.
 *  @author Jack Shelata
 *  @date April 16, 2019
 */

#include "ros/ros.h"
#include "std_msgs/UInt32.h"
#include <bitset>
#include <cassert>
#include <string>

/***************************************************************
**                Class(es) Definition
****************************************************************/

/** @brief This class creates and stores ROS Publisher to handle controller
 *  input and send to '/control' topic.
 */

class Controller{
  public:
    Controller();
    
    void setButton(int index, unsigned int value);
    void setAxis(int index, unsigned int value);
    void updateAll();
    unsigned int getData();
    bool getActive();
    bool getDebug();

  private:
  // 19 total inputs
  unsigned int inputs[19];
  // masks for each
  unsigned int masks[19];
  // 32-bit int to hold all values
  unsigned int data;
  bool active;
  bool debug;
  std::string inputNames[19] = {"A","B","X","Y","Left Bumper","Right Bumper",
                                "Back","Start","Center","Left Joytick Button",
                                "Right Joystick Button","Up","Down","Left",
                                "Right", "Left Joystick","Right Joystick",
                                "Left Trigger","Right Trigger"};
};

int expand(unsigned int val);
unsigned int compress(int val);
int reduce(int val);

