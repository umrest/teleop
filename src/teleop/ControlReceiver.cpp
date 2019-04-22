/** 
 * @file Controller.cpp
 * @brief Class implementation for Controller
 * @author Jack Shelata
 * @date April 18, 2019
 */

#include "ControlReceiver.h"

/***************************************************************
**                Functions Definition
****************************************************************/

ControlReceiver::ControlReceiver(ros::NodeHandle& nh_in){
  
  nh = nh_in;

  for(int i = 0; i < 15; ++i){
    inputs[i].name = inputNames[i];
    inputs[i].bitLength = 1;
    inputs[i].offset = 31 - i;
    inputs[i].value = 0;
    inputs[i].axis = false;
  }
  for(int i = 15; i < 17; ++i){
    inputs[i].name = inputNames[i];
    inputs[i].bitLength = 4;
    inputs[i].offset = (13 - (4*(i-15)));
    inputs[i].value = 0;
    inputs[i].axis = true;
  }
  for(int i = 17; i < 19; ++i){
    inputs[i].name = inputNames[i];
    inputs[i].bitLength = 4;
    inputs[i].offset = (13 - (4*(i-15)));
    inputs[i].value = -7;
    inputs[i].axis = true;
  }

  control_sub = nh.subscribe("/control", 1000,
                             &ControlReceiver::handleControlMsg, this);
  serial_pub = nh.advertise<teleop::ControlMsg>("/serial_control",
                                                1000);
}

void ControlReceiver::getAll(){
  for(int i = 0; i < 19; ++i){
    inputs[i].value = getBits(inputs[i].bitLength, inputs[i].offset,
                              inputs[i].axis);
    printf("%-21s %-6i\n", inputs[i].name.c_str(), inputs[i].value);
  }
}

void ControlReceiver::handleControlMsg(const std_msgs::UInt32::ConstPtr& msg){
  setData(msg->data);
  getAll();
  sendToSerial();
}

void ControlReceiver::sendToSerial(){
  teleop::ControlMsg msg;

  // drivetrain
  msg.left_wheel_front = inputs[LEFTJOY].value * SCALE;
  msg.left_wheel_back = inputs[LEFTJOY].value * SCALE;
  msg.right_wheel_front = inputs[RIGHTJOY].value * SCALE;
  msg.right_wheel_back = inputs[RIGHTJOY].value * SCALE;

  // excavation
  msg.bucket_motor = (inputs[RT].value + 7) * 12;
  msg.excavation_actuator = inputs[UP].value;

  // collection
  msg.collection_actuators = inputs[RIGHT].value;

  serial_pub.publish(msg);  
}