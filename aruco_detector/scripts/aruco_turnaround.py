#! /usr/bin/env python3

from math import atan2, degrees
import cv2
import numpy as np
import rospy
from geometry_msgs.msg import Pose2D, Twist
from std_msgs.msg import Int16

class PathPlanner:
    def __init__(self) -> None:
        self.distance = 0
        self.id = 0

        class Point:
            def __init__(self) -> None:
                self.x = 0
                self.y = 0

        self.mid_pose = Point()

        rospy.init_node("aruco_follower")

        self.mid_sub = rospy.Subscriber(
            "/aruco/center_pose",
            Pose2D,
            self.changeMidPose
        )
        self.id_sub = rospy.Subscriber(
            "/aruco/id",
            Int16,
            self.changeId
        )
        self.distance_sub = rospy.Subscriber(
            "/aruco/distance",
            Int16,
            self.changeDistance
        )
        self.aruco_flag = rospy.Publisher(
            "/aruco_flag",
            Int16,
            queue_size=1
        )

    def changeDistance(self, _dist):
        self.distance = _dist.data

    def changeId(self, _id):
        self.id = _id.data

    def changeMidPose(self, _pose):
        self.mid_pose.x = _pose.x
        self.mid_pose.y = _pose.y

    def sendCmd(self):     
        if self.distance < 50 and self.distance > 0:
            self.aruco_flag.publish(1)
            self.distance = 0
        else:
            self.aruco_flag.publish(0)

            

if __name__ == "__main__":
    pp = PathPlanner()
    rate = rospy.Rate(30)
    while not rospy.is_shutdown():
        pp.sendCmd()
        rate.sleep()
