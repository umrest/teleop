#include "ControlReceiver.h"

ControlReceiver::ControlReceiver(){

}

void ControlReceiver::updateControlValues(
                                  const std_msgs::UInt32::ConstPtr& msg){
  
}

int decode(unsigned int code, int offset, bool axis){
  unsigned int mask = axis ? 0x0000000F : 0x00000001;
  mask <<=offset;
  code &= mask;
  return code >> offset;
} 
