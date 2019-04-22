#!/usr/bin/env python

import rospy
import serial
from teleop.msg import ControlMsg


def handleControlMessage(data, ser):
    ser.write(makeString(data))
    
    
def serialNode(ser):

    rospy.init_node('serial_node', anonymous=True)

    rospy.Subscriber("/serial_control", ControlMsg, handleControlMessage, ser)

    rospy.spin()

def makeString(msg):
    return '{},{},{},{},{},{},{}\n'.format(msg.left_wheel_front,
                                           msg.left_wheel_back,
                                           msg.right_wheel_front,
                                           msg.right_wheel_back,
                                           msg.collection_actuators,
                                           msg.bucket_motor,
                                           msg.excavation_actuator)



if __name__ == '__main__':
    port = '/dev/pts/2'
    baudRate = 9600

    ser = serial.Serial(port, baudRate, rtscts=True,dsrdtr=True)

    serialNode(ser)