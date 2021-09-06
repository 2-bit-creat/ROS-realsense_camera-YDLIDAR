#! /usr/bin/env python
import cv2
import serial
import time
import numpy as np
import ctypes
import rospy
from std_msgs.msg import Int32

class serial_com:
    #change the USB number in case the port is not USB1
    seri = serial.Serial('/dev/ttyUSB1', 115200) 
    def __init__(self):
        rospy.init_node("main_node")
        
        self.steering = 0
        self.flag_sub = rospy.Subscriber(
            "/flag",
            Int32,
            self.ch_flag
        )

        self.flag = 0
        self.steering_sub = rospy.Subscriber(
            "/steering",
            Int32,
            self.ch_steering
        )

    def ch_flag(self, _data):
        self.flag = _data.data

    def ch_steering(self, _data):
        self.steering = _data.data
    

    def writeBuffer(self, data, speed, _steer):
        direction = 0

        speed = np.uint16(speed)
        _steer = np.uint16(_steer)
        speed_Lo = speed & 0xFF
        speed_Hi = speed >> 8
        steer_Lo = _steer & 0xFF
        steer_Hi = _steer >> 8

        sum = direction + speed_Lo + speed_Hi + steer_Lo + steer_Hi + 220 + 5 + 10 + 13
        clc = np.uint8(~sum)

        data.append(0x53)
        data.append(0x54)
        data.append(0x58)
        data.append(direction)
        data.append(speed_Lo)
        data.append(speed_Hi)
        data.append(steer_Lo)
        data.append(steer_Hi)
        data.append(0xDC)
        data.append(0x05)
        data.append(0x00)
        data.append(0x0D)
        data.append(0x0A)
        data.append(clc)


    def serWrite(self, ser, _speed, _steer):
        data = []
        self.writeBuffer(data, _speed, _steer)

        for i in range(0, len(data), 1):
            data[i] = np.uint8(data[i])
        ser.write(data)
        
    def run(self): 
        rate = rospy.Rate(40)
        #the while loop here can replace rospy.spin
        while not rospy.is_shutdown():
            if self.flag == 0:
                if self.steering == 1:
                    self.serWrite(self.seri, 400, 1300) #turn left
                elif self.steering == -1:
                    self.serWrite(self.seri, 400, 1800) #turn right
                else:
                    self.serWrite(self.seri, 400, 1550) #go straight
            elif self.flag == 1:
                self.serWrite(self.seri, 0, 1550) #emergency brake for 2 seconds
                time.sleep(2)
            rate.sleep()

class CAN_com:
    def __init__(self):
        rospy.init_node("main_node")
        
        self.CAN_pub = rospy.Publisher(
            "/cmd_vel",
            Int32,
            queue_size= 5
        )

        self.steering = 0
        self.flag_sub = rospy.Subscriber(
            "/flag",
            Int32,
            self.ch_flag
        )

        self.flag = 0
        self.steering_sub = rospy.Subscriber(
            "/steering",
            Int32,
            self.ch_steering
        )
   
    def ch_flag(self, _data):
        self.flag = _data.data

    def ch_steering(self, _data):
        self.steering = _data.data

    def run(self): 
        rate = rospy.Rate(40)
        #the while loop here can replace rospy.spin
        while not rospy.is_shutdown():
            if self.flag == 0:
                if self.steering == 1:
                    self.CAN_pub.publish(0) #turn left
                elif self.steering == -1:
                    self.CAN_pub.publish(0) #turn right
                else:
                    self.CAN_pub.publish(0) #go straight
            elif self.flag == 1:
                self.CAN_pub.publish(0) #emergency brake for 2 seconds
                time.sleep(2)
            rate.sleep()




if __name__ == "__main__":
    a = serial_com()
    a.run()