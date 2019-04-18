#include "ros/ros.h"
#include "std_msgs/Int64.h"

#include <sstream>

/**
  * This node will transmit feedback data back to the station computer.
  * It will do this by first receiving feedback data from the serial node which
  * will handle the serial communication with the HERO. The serial node will
  * take data from the receive node, serialize it, and feed it to the HERO.
  * It will also receive serial input from the HERO, package it in ROS message
  * and send to this node. This node will then publish the info as an Int64
  * for the station computer to translate and expand into useful data for the
  * controller to use.
  **/

int main(int argc, char **argv){
  
  // initialize ROS node
  ros::init(argc, argv, "rover_teleop_transmit");

  // create NodeHandle to access ROS communication
  ros::NodeHandle nodeHandle;

  // create Subscriber to recieve control messages on topic "control"
  ros::Publisher feedback_pub =
                  nodeHandle.advertise<std_msgs::Int64>("feedback", 1000);

  // set loop_rate if we are publishing every so often
  ros::Rate loop_rate(10);

  // call spin to pump callbacks
  ros::spin();

  return 0;

}
