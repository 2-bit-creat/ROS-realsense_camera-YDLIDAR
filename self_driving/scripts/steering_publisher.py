#! /usr/bin/env python
import cv2
import numpy as np
import serial
import matplotlib.pyplot as plt
from sensor_msgs.msg import Image
from std_msgs.msg import Int32
from std_msgs.msg import Float32
import time
import rospy
from cv_bridge import CvBridge

class LaneDetection:
    # customize these values
    low_yellow = np.array([15, 150, 120]) 
    high_yellow = np.array([25, 230, 210])
    low_white = np.array([0, 150, 0])
    high_white = np.array([255, 255, 255])
    
    def __init__(self):
        rospy.init_node("steering_node")
        self.cvbridge = CvBridge()
        #go to usb_cam-test.launch and change video number ex) value="/dev/video4"
        self.img_sub = rospy.Subscriber("/camera/color/image_raw",Image,self.convertIng)  
        self.steering_pub = rospy.Publisher("/steering", Float32, queue_size=5)
        rospy.spin()

    def convertIng(self, img):
        frame = self.cvbridge.imgmsg_to_cv2(img, "bgr8")
        frame = cv2.resize(frame, dsize=(640, 480), interpolation=cv2.INTER_AREA)
        bird_eye_image = self.bird_eye(frame)
        colored_image = self.color_detect(bird_eye_image)
        self.slidng_list_left = self.sliding_left(colored_image)
        self.sliding_list_right = self.sliding_right(colored_image)
        frame_steer = self.steering(frame, self.slidng_list_left, self.sliding_list_right)

    def steering(self, frame, slidng_list_left, sliding_list_right):
        x_left = []
        x_right = []

        #extract x coordinates from slidng_list_left
        for i in range(0, len(slidng_list_left)): 
            x_left.append(slidng_list_left[i][0])
        left_diff_arr = np.diff(x_left)
        left_diff_sum = np.sum(left_diff_arr)
        left_avg = int(left_diff_sum/8)

        #remove inappropriate cases 
        for i in range(0, len(left_diff_arr)): 
            if (left_diff_arr[i] > (left_avg +80)) or (left_diff_arr[i] < (left_avg - 80)): #the value 80 can be modified
                self.slidng_list_left = []
                left_diff_sum = 0

        #extract x coordinates from sliding_list_right
        for i in range(0, len(sliding_list_right)): 
            x_right.append(sliding_list_right[i][0])
        right_diff_arr = np.diff(x_right)
        right_diff_sum = np.sum(right_diff_arr)
        right_avg = int(right_diff_sum/8)

        #remove inappropriate cases
        for i in range(0, len(right_diff_arr)): 
            if (right_diff_arr[i] > (right_avg +80)) or (right_diff_arr[i] < (right_avg - 80)): #the value 80 can be modified
                self.sliding_list_right = []
                right_diff_sum = 0
        
        #publish average values divided by 100
        avg_val = float((left_diff_sum + right_diff_sum)/2)
        self.steering_pub.publish(avg_val/100)
    
        return frame #return frame is for visualization

    def sliding_left(slef, img):
        left_list = []
        '''
        row: starting from y=179 to y=460, moving by 40
        col: starting from x=19 to y=300, moving by 5
        '''
        for j in range(179, img.shape[0] - 20, 40): 
            j_list = []
            for i in range(19, int(img.shape[1]/2) - 20, 5):
                num_sum = np.sum(img[j - 19:j + 21, i - 19:i + 21]) #window size is 20*20
                if num_sum > 100000: #pick (i,j) where its num_sum is over 100000
                    j_list.append(i)
            try:
                len_list = [] 
                #cluster if a gap between elements in the list is over 5
                result = np.split(j_list, np.where(np.diff(j_list) > 5)[0] + 1) 
                for k in range(0, len(result)):
                    len_list.append(len(result[k])) #append the lengths of each cluster
                largest_integer = max(len_list)
            
                for l in range(0, len(result)):
                    if len(result[l]) == largest_integer: 
                        avg = int(np.sum(result[l]) / len(result[l])) #average
                        left_list.append((avg, j)) #avg points of left side 
            except:
                continue
        return left_list

    
    def sliding_right(slef, img):
        right_list = []
        '''
        row: starting from x=179 to y=560, moving by 40
        col: starting from x=320 to y=620, moving by 5
        '''
        for j in range(179, img.shape[0] - 20, 40): 
            j_list = []
            for i in range(int(img.shape[1]/2), img.shape[1] - 20, 5): 
                num_sum = np.sum(img[j - 19:j + 21, i - 19:i + 21]) #window size is 20*20
                if num_sum > 100000: #pick (i,j) where its num_sum is over 100000
                    j_list.append(i)
            try:
                len_list = []
                #cluster if a gap between elements in the list is over 5
                result = np.split(j_list, np.where(np.diff(j_list) > 5)[0] + 1) 
                for k in range(0, len(result)):
                    len_list.append(len(result[k])) #append the lengths of each cluster
                largest_integer = max(len_list)
            
                for l in range(0, len(result)):
                    if len(result[l]) == largest_integer: 
                        avg = int(np.sum(result[l]) / len(result[l])) #average
                        right_list.append((avg, j)) #avg points of left side 
            except:
                continue
        return right_list

    def color_detect(self, img):
        hls = cv2.cvtColor(img, cv2.COLOR_BGR2HLS)
        mask_white = cv2.inRange(hls, self.low_white, self.high_white)
        mask_yellow = cv2.inRange(hls, self.low_yellow, self.high_yellow)
        mask = cv2.bitwise_or(mask_white, mask_yellow)
        return mask

    def bird_eye(self, frame):
        #customize these values
        pts1 = np.float32([[190, 200], [450, 200], [70, 450], [570, 450]]) 
        pts2 = np.float32([[0, 0], [640, 0], [0, 480], [640, 480]])
        matrix = cv2.getPerspectiveTransform(pts1, pts2)
        result = cv2.warpPerspective(frame, matrix, (640, 480))
        return result

if __name__ == '__main__':
    a = LaneDetection()
