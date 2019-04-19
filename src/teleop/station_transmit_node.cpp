/** 
 * @file station_transmit_node.cpp
 * @brief ROS node for transmitting control data
 * @author Jack Shelata
 * @date April 18, 2019
 */

#include "ros/ros.h"
#include "std_msgs/UInt32.h"
#include "Controller.h"


int main(int argc, char **argv){
  
  // initialize ROS node
  ros::init(argc, argv, "station_transmit");
  
  // create NodeHandle to access ROS communication
  ros::NodeHandle nh;
  
  // create Controller
  Controller controller(nh);

  // call controller read and transmit
  controller.readAndTransmit();

  return 0;

}