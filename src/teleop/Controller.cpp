/** 
 * @file Controller.cpp
 * @brief Class implementation for Controller
 * @author Jack Shelata
 * @date April 18, 2019
 */

#include "Controller.h"
#include "joystick.c"

/***************************************************************
**                Functions Definition
****************************************************************/

Controller::Controller(ros::NodeHandle& nh_in){
  nh = nh_in;
  control_pub = nh.advertise<std_msgs::UInt32>("/control", 1000);

  resetControls(false);

  active = false;
  debug = false;
  useful = true;
  
  updateAll();
  readAndTransmit();
}

void Controller::updateAll(){
  for(int i = 0; i < 19; ++i){
    setBits(inputs[i].bitLength, inputs[i].offset, inputs[i].value);
  }
}

void Controller::resetControls(bool send){
  for(int i = 0; i < 15; ++i){
    inputs[i].name = inputNames[i];
    inputs[i].bitLength = 1;
    inputs[i].offset = 31 - i;
    inputs[i].value = 0;
    inputs[i].axis = false;
  }
  for(int i = 15; i < 19; ++i){
    inputs[i].name = inputNames[i];
    inputs[i].bitLength = 4;
    inputs[i].offset = (13 - (4*(i-15)));
    inputs[i].value = (i > 16 && send) ? -7 : 0;
    inputs[i].axis = true;
  }
  updateAll();
  if(send){
    sendMessage(true);
  }
}

void Controller::setInput(int index, int value){
  /*

  this if statement is complicated so I will explain in detail
  this if checks for input from index 8 which is the center button.
  index 8 has value 1 the control system should toggle activation.
  when the controller first starts up, the trigger values are read as 0
  but when not depressed, the correct trigger value is -7 (or -32767 before
  the resolution is stepped down). The else if that follows checks to
  make sure that the triggers have been calibrated (read correctly as
  -7) before sending any messages. When the triggers are pressed, they
  are read correctly and the program can proceed as normal. Once the
  controller has been activated, this problem is eliminated.

  */
  if((index == 8 && value && inputs[17].value != 0 && inputs[18].value != 0) ||
     (index == 8 && value && active)){
    active = !active;
    useful = true;
    printf("Controls %s\n", active ? "ACTIVATED" : "DEACTIVATED");
    if(!active) resetControls(true);
  }
  else if(index == 8 && value && inputs[17].value == 0 && inputs[18].value == 0){
    printf("Calibrate triggers by pulling both simultaneously and releasing\n");
    printf("(This only needs to happen once)\n");
  }
  if(index == 7 && value){
    debug = !debug;
    printf("Debug mode %s\n", debug ? "ACTIVATED" : "DEACTIVATED");
  }

  if(inputs[index].axis){
    inputs[index].value = reduce(value);
  }
  else{
    inputs[index].value = value;
  }

  setBits(inputs[index].bitLength, inputs[index].offset, inputs[index].value);

  if(debug){
    printf("%-21s %-4i %-16u\n", inputs[index].name.c_str(), value, getData());
  }
  if(!active && useful){
    printf("Controls are deactivated, press Center Button to activate\n");
    useful = false;
  }
}

void Controller::sendMessage(bool force){
  if(getData() != msg.data || force){
    msg.data = getData();
    control_pub.publish(msg);
  }
}

void Controller::readAndTransmit(){
    // from joystick.c
    // set up the xbox connection
    const char *device;
    int js;
    struct js_event event;
    struct axis_state axes[3] = {0};
    size_t axis;
    // set loop rate (might be unnecessary)
    ros::Rate loop_rate(10);

    device = "/dev/input/js0";
    js = open(device, O_RDONLY);
    if (js == -1){
      printf("Could not open joystick\n");
    }
    // change the device js to non-blocking
    fcntl(js, F_SETFL, O_NONBLOCK);
    
    while (ros::ok()){
    // if there is new control input
    if(read_event(js, &event) == 0){
    
      switch (event.type){
      
        case JS_EVENT_BUTTON:
          setInput(event.number, event.value);
          break;
          
        case JS_EVENT_AXIS:
          axis = get_axis_state(&event, axes);

          switch(axis){
            case 0:
              // left joystick up/down
              setInput(15, axes[axis].y);
              break;
              
            case 1:
              // left trigger up/down
              setInput(17, axes[axis].x);
              break;
              
            case 2:
              // right joy up/down
              setInput(16, axes[axis].x);
              // right trigger
              setInput(18, axes[axis].y);
              break;
            case 3:
              // down
              if(event.number == 7 && event.value > 0){
                setInput(12,1);
              }
              // up
              else if(event.number == 7 && event.value < 0){
                setInput(11,1);
              }
              // up and down return to 0
              else if(event.number == 7 && event.value == 0){
                setInput(11,0);
                setInput(12,0);
              }
              // left
              else if(event.number == 6 && event.value < 0){
                setInput(13,1);
              }
              // right
              else if(event.number == 6 && event.value > 0){
                setInput(14,1);
              }
              // left and right return to 0
              else if(event.number == 6 && event.value == 0){
                setInput(13,0);
                setInput(14,0);
              }
              
          }
          break;

        default:
          // Ignore init events.
          break;
      }

      if(active){
        sendMessage(false);
      }
    }

    // if no event, sleep
    else{
      ros::spinOnce();
    }
  }
  // close device upon exit
  close(js);
}

int reduce(int value){
  int step = 32767 / 8;
  value /= step;
  if(value > 7) value = 7;
  if(value < -7) value = -7;
  return value;
}