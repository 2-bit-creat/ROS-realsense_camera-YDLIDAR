#! /usr/bin/env python
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Int32
import rospy
import cv2
import serial
import time
import numpy as np
import ctypes


class rplidar_sub:
    initial_time=time.time()
    flag = 0
    
    def __init__(self):
        rospy.init_node("flag_node")
        self.sub = rospy.Subscriber("/scan", LaserScan, self.callback1)
        self.pub = rospy.Publisher('/flag', Int32, queue_size= 5)
        print("hello")
        rospy.spin() #if there is no rospy.spin the process will end right away. But in case there are other loops that enables the process keep going, you don't have to use rospy.spin

    def callback1(self, rp_data):
        for i in range(0, 29, 1):
            if rp_data.ranges[29-i] < 0.7 or rp_data.ranges[330+i] < 0.7: #publish 1 when the LiDAR detects something within 0.7m in the range of [0, 30] or [330, 360] degrees.
                self.pub.publish(1)
                return 
            
    def callback2(self, rp_data):
        print("0: ", rp_data.range[0], "90: ",rp_data.range[90], "180: ", rp_data.range[180], "270: ", rp_data.range[270])
        

if __name__ == "__main__":
    a = rplidar_sub()
