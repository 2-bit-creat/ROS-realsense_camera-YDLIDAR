#!/usr/bin/env python3

import cv2
from cv2 import aruco
import sys
import numpy as np
import rospy
from rospy.topics import Publisher
from sensor_msgs.msg import Image
from std_msgs.msg import Int16
from geometry_msgs.msg import Pose2D
import math

class ArUcoDetector:
    def __init__(self):
        rospy.init_node("aruco_turnaround")
        self.aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
        self.parameters = aruco.DetectorParameters_create()
        self.corners = 0
        self.ids = 0
        self.rejectedImgPoints = 0
        self.detected_block = []
        self.img = None

        self.cam_mtx = np.reshape(np.array([698.0748183627579, 0, 340.8473791794117, 0, 654.433652441636, 261.3244628969213, 0, 0, 1]), (3,3))
        self.cam_dist = np.array([0.1046634609750675, -0.2336462654423322, 0.01097635493757336, 0.01393432684990838, 0])

        self.img_sub = rospy.Subscriber(
            "/camera/color/image_raw",
            Image,
            self.imshow
        )

        self.distance_pub = rospy.Publisher(
            "/aruco/distance",
            Int16,
            queue_size=3
        )

        self.detect_id = rospy.Publisher(
            "/aruco/id",
            Int16,
            queue_size=3
        )

        self.detect_mid = rospy.Publisher(
            "/aruco/center_pose",
            Pose2D,
            queue_size=3
        )

        self.test_img = rospy.Publisher(
            "/aruco/detected_img",
            Image,
            queue_size=3
        )
        
    def recv_img(self, _data):
        self.img = self.imgmsg_to_cv2(_data).copy()
        self.run()

    def run(self):
        # Convert GrayScale for ArUco
        _img = cv2.cvtColor(self.img, cv2.COLOR_BGR2GRAY)

        # Detecting Marker
        self.corners, self.ids, self.rejectedImgPoints = aruco.detectMarkers(
            _img,
            self.aruco_dict,
            parameters=self.parameters
        )
        rvec, tvec ,_ = aruco.estimatePoseSingleMarkers(self.corners, 0.07, self.cam_mtx, self.cam_dist)


        if len(self.corners) > 0:
            temp = self.img.copy()
            aruco.drawAxis(temp, self.cam_mtx, self.cam_dist, rvec[0], tvec[0], 0.1)
            cv2.putText(temp, "%.1f cm -- %.0f deg" % ((tvec[0][0][2] * 100), (rvec[0][0][2] / math.pi * 180)), (0, 230), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (244, 244, 244))
            self.test_img.publish(self.cv2_to_imgmsg(temp))
            self.ids = self.ids[0][0]
            axis = self.corners[0][0]
            
            min_x, min_y, max_x, max_y = self.calcSquareSize(axis)
            self.detect_id.publish(self.ids)

            mid_x, mid_y = self.getSquareAxis(axis)
            p2d = Pose2D()
            p2d.x, p2d.y = mid_x, mid_y
            self.detect_mid.publish(p2d)
            
            self.distance_pub.publish(int(tvec[0][0][2] * 100))
        else:
            self.test_img.publish(self.cv2_to_imgmsg(self.img))


    def getSquareAxis(self,_arr):
        mid_x = int((_arr[0,0] + _arr[2,0]) / 2)
        mid_y = int((_arr[0,1] + _arr[2,1]) / 2)

        return [mid_x, mid_y]

    def imgmsg_to_cv2(self, img_msg, _type="rgb8"):
        dtype = np.dtype("uint8") # Hardcode to 8 bits...
        dtype = dtype.newbyteorder('>' if img_msg.is_bigendian else '<')
        image_opencv = None
        if _type == "mono8":
            image_opencv = np.ndarray(
                shape=(
                    img_msg.height,
                    img_msg.width
                ), # and three channels of data. Since OpenCV works with bgr natively, we don't need to reorder the channels.
                dtype=dtype,
                buffer=img_msg.data
            )
        elif _type == "rgb8":
            image_opencv = np.ndarray(
                shape=(
                    img_msg.height,
                    img_msg.width,
                    3
                ), # and three channels of data. Since OpenCV works with bgr natively, we don't need to reorder the channels.
                dtype=dtype,
                buffer=img_msg.data
            )
        
        # If the byt order is different between the message and the system.
        if img_msg.is_bigendian == (sys.byteorder == 'little'):
            image_opencv = image_opencv.byteswap().newbyteorder()
        image_opencv = cv2.cvtColor(image_opencv, cv2.COLOR_RGB2BGR)
        return image_opencv

    def cv2_to_imgmsg(self, _img, _type="bgr8"):
        img_msg = Image()
        img_msg.height = _img.shape[0]
        img_msg.width = _img.shape[1]
        img_msg.encoding = _type
        img_msg.is_bigendian = 0
        img_msg.data = _img.tostring()
        img_msg.step = len(img_msg.data) // img_msg.height # That double line is actually integer division, not a comment
        return img_msg

    @staticmethod
    def calcSquareSize(_axis):
        min_x = int(np.min(_axis[:,0]))
        min_y = int(np.min(_axis[:,1]))
        
        max_x = int(np.max(_axis[:,0]))
        max_y = int(np.max(_axis[:,1]))
        return min_x, min_y, max_x, max_y

    @staticmethod
    def checkMidPosition(_x, _y, _w, _h):
        boundary = {
            "x1":int(_w/2)-100,
            "x2":int(_w/2)+100,
            "y1":int(_h/2)-100,
            "y2":int(_h/2)+100
        }
        if _x > boundary["x1"] and _x < boundary["x2"]:
            if _y > boundary["y1"] and _y < boundary["y2"]:
                return True
        return False


if __name__ == "__main__":
    acTracer = ArUcoDetector()
    rospy.spin()