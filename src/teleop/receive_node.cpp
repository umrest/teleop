#include "ros/ros.h"
#include "std_msgs/UInt32.h"
#include "ControlReceiver.h"

#include <sstream>

/**
  * This node will receive the control data as an UInt32 from the station
  * computer. It will translate this, log the control data, and send it
  * to the serial node to be given to the HERO board.
  **/

void controlCallback(const std_msgs::UInt32::ConstPtr& msg){
  ROS_INFO("I heard: [%u]", msg->data);
  printf("Received: %u\n", msg->data);
}

int main(int argc, char **argv){
  
  // initialize ROS node
  ros::init(argc, argv, "rover_teleop_receive");

  // create NodeHandle to access ROS communication
  ros::NodeHandle nodeHandle;

  // create Subscriber to recieve control messages on topic "control"
  ros::Subscriber control_sub =
                  nodeHandle.subscribe("/control", 1000, controlCallback);

  // call spin to pump callbacks
  //ros::spin();
  
  unsigned int temp = 0x80000000;
  printf("%i\n",decode(temp, 31, false));

  return 0;

}
