#include "ros/ros.h"
#include "std_msgs/UInt32.h"
#include <bitset>
#include <cassert>
#include <string>

class ControlReceiver {
  public:
    ControlReceiver();
    void updateControlValues(const std_msgs::UInt32::ConstPtr& msg);
  
  private:
    ros::Subscriber control_sub;
    ros::NodeHandle nodeHandle;
    int controlVaules[19];
};

int decode(unsigned int code, int offset, bool axis);
