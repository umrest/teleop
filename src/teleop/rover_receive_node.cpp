/** 
 * @file rover_receive_node.cpp
 * @brief ROS node for receiving control data
 * @author Jack Shelata
 * @date April 18, 2019
 */

#include "ros/ros.h"
#include "std_msgs/UInt32.h"
#include "ControlReceiver.h"

/**
  * This node will receive the control data as an UInt32 from the station
  * computer. It will translate this, log the control data, and send it
  * to the serial node to be given to the HERO board.
  **/
int main(int argc, char **argv){
  
  // initialize ROS node
  ros::init(argc, argv, "rover_receive");

  // create NodeHandle to access ROS communication
  ros::NodeHandle nh;

  // create ControlReceiver
  ControlReceiver receiver(nh);

  // call spin to pump callbacks
  ros::spin();

  return 0;

}
