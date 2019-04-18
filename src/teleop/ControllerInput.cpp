/** @file ControllerInput.cpp
 *  @brief Class implementation for ControllerInput.
 *  @author Jack Shelata
 *  @date April 16, 2019
 */

#include "ControllerInput.h"
#include "joystick.c"

/***************************************************************
**                Function(s) Definition
****************************************************************/

/** @brief Construct Controller object. Constructor creates and
 *  initializes a ROS Publisher to manage sending control commands to the
 *  Rover computer on topic '/control'
 *  @return Controller object
 *  @exception None
 */
Controller::Controller(){
  // initialize all buttons to 0
  data = 0;
  // create all the masks for the buttons
  unsigned int currentMask = 0x7FFFFFFF;
  for(int i = 0; i < 15; ++i){
    masks[i] = currentMask;
    currentMask = ~(currentMask & currentMask);
    currentMask >>= 1;
    currentMask = ~(currentMask & currentMask);
  }
  // create all the masks for the axis
  currentMask = 0xF0000000;
  currentMask >>= 15;
  currentMask = ~(currentMask & currentMask);
  for(int i = 15; i < 19; ++i){
    masks[i] = currentMask;
    currentMask = ~(currentMask & currentMask);
    currentMask >>= 4;
    currentMask = ~(currentMask & currentMask);
  }
  // the left and right trigger are at -32767 to begin so set them to this
  setAxis(17, compress(reduce(-32767)));
  setAxis(18, compress(reduce(-32767)));
  active = false;
  debug = false;
}

void Controller::updateAll(){
  // reset data
  data = 0;
  // or all values
  for(int i = 0; i < 15; ++i){
    data |= inputs[i] << (31 - i);
  }
  for(int i = 15; i < 19; ++i){
    data |= inputs[i] << (31 - (i + 4*(i-15)));
  }
}

void Controller::setButton(int index, unsigned int value){
  inputs[index] = value;
  data &= masks[index];
  data |= inputs[index] << (31 - index);
  if(index == 8 && value){
    active = !active;
    printf("Controls %s\n", active ? "ACTIVATED" : "DEACTIVATED");
  }
  if(index == 7 && value){
    debug = !debug;
    printf("Debug mode %s\n", debug ? "ACTIVATED" : "DEACTIVATED");
  }
  if(debug){
    printf("%-21s %-4i %-16u\n", inputNames[index].c_str(), expand(value), 
            getData());
  }
  if(!active){
    printf("Controls are deactivated, press Center Button to activate\n");
  }
}
void Controller::setAxis(int index, unsigned int value){
  inputs[index] = value;
  data &= masks[index];
  data |= inputs[index] << (13 - (4*(index-15)));
  if(debug){
    printf("%-21s %-4i %-16u\n", inputNames[index].c_str(), expand(value),
            getData());
  }
  if(!active){
    printf("Controls are deactivated, press 'Center Button' to activate\n");
  }
}

unsigned int Controller::getData(){
  return data;
}

bool Controller::getActive(){
  return active;
}

bool Controller::getDebug(){
  return debug;
}

int expand(unsigned int val){
  unsigned int MSB = val >> 3;
  unsigned int mask = MSB ? 0xFFFFFFF0 : 0x00000000;
  val |= mask;
  return val;
}
unsigned int compress(int val){
  assert(val <= 7 && val >= -8);
  unsigned int mask = 0x0000000F;
  unsigned int newVal = val & mask;
  return newVal;
}
int reduce(int val){
  int step = 32767 / 8;
  val /= step;
  if(val > 7) val = 7;
  if(val < -7) val = -7;
  return val;
}

int main(int argc, char** argv){
  // initialize ROS node
  ros::init(argc, argv, "station_teleop_transmit");

  // create NodeHandle to access ROS communication
  ros::NodeHandle nodeHandle;

  // create Publisher to publish messages to "control"
  ros::Publisher control_pub =
                    nodeHandle.advertise<std_msgs::UInt32>("/control", 1000);

  // set loop_rate for non-blocking input sleep
  ros::Rate loop_rate(10);
  std_msgs::UInt32 msg;
  
  unsigned int currentData = 0;
  unsigned int newData = 0;

  const char *device;
  int js;
  struct js_event event;
  struct axis_state axes[3] = {0};
  size_t axis;

  if (argc > 1)
    device = argv[1];
  else
    device = "/dev/input/js0";

  js = open(device, O_RDONLY);

  if (js == -1)
    perror("Could not open joystick");

  Controller controller;
  
  // change the device js to non-blocking
  fcntl(js, F_SETFL, O_NONBLOCK);

  /* This loop will exit if the controller is unplugged. */
 
  //while (read_event(js, &event) == 0)
  while (ros::ok()){
    // if there is new control input
    if(read_event(js, &event) == 0){
    
      switch (event.type){
      
        case JS_EVENT_BUTTON:
          controller.setButton(event.number, event.value);
          break;
          
        case JS_EVENT_AXIS:
          axis = get_axis_state(&event, axes);
          
          switch(axis){
            case 0:
              // left joystick up/down
              controller.setAxis(15, compress(reduce(axes[axis].y)));
              break;
              
            case 1:
              // left trigger up/down
              controller.setAxis(17, compress(reduce(axes[axis].x)));
              break;
              
            case 2:
              // right joy up/down
              controller.setAxis(16, compress(reduce(axes[axis].x)));
              // right trigger
              controller.setAxis(18, compress(reduce(axes[axis].y)));
              break;
              
          }
          break;
        default:
          // Ignore init events.
          break;
      }
      // get the new data
      newData = controller.getData();
      
      // if the new data is different than the current data, send and update
      if(newData != currentData){
        currentData = newData;

        if(controller.getActive()){
          msg.data = currentData;
          control_pub.publish(msg);
        }
      }
    }
    // if no event, sleep
    else{
      ros::spinOnce();
    }
  }
  

  close(js);
  return 0;
}
