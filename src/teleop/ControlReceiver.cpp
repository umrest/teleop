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
}