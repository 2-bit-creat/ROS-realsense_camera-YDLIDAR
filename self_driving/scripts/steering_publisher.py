#! /usr/bin/env python
import cv2
import numpy as np
import serial
import matplotlib.pyplot as plt
from sensor_msgs.msg import Image
from std_msgs.msg import Int32
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
        self.img_sub = rospy.Subscriber("/color/image_raw",Image,self.convertIng)  
        self.steering_pub = rospy.Publisher("/steering", Int32, queue_size=5)
        rospy.spin()

    def convertIng(self, img):
        frame = self.cvbridge.imgmsg_to_cv2(img, "bgr8")
        frame = cv2.resize(frame, dsize=(640, 480), interpolation=cv2.INTER_AREA)
        colored_image = self.color_detect(frame)
        bird_eye_image = self.bird_eye(colored_image)
        self.slidng_list_left = self.sliding_left(bird_eye_image)
        self.sliding_list_right = self.sliding_right(bird_eye_image)
        frame_steer = self.steering(frame, self.slidng_list_left, self.sliding_list_right)

        #visualization for users
        bird_eye_image = cv2.cvtColor(bird_eye_image, cv2.COLOR_GRAY2BGR)
        for i in self.slidng_list_left:
            cv2.circle(bird_eye_image, i, 7, (255, 255, 0), -1)
        for i in self.sliding_list_right:
            cv2.circle(bird_eye_image, i, 7, (255, 255, 0), -1)

        # customize these values    
        cv2.circle(frame_steer, (60, 339), 5, (0, 0, 255), -1) 
        cv2.circle(frame_steer, (580, 339), 5, (0, 0, 255), -1)
        cv2.circle(frame_steer, (0, 419), 5, (0, 0, 255), -1)
        cv2.circle(frame_steer, (640, 419), 5, (0, 0, 255), -1)

        cv2.imshow("bird_eye_with_circles", bird_eye_image)
        cv2.imshow("frame_with_direction_and_bird_eye_points", frame_steer)
        cv2.waitKey(1)
        

    def steering(self, frame, slidng_list_left, sliding_list_right):
        x_left = []
        x_right = []

        #extract x coordinates from slidng_list_left
        for i in range(0, len(slidng_list_left)): 
            x_left.append(slidng_list_left[i][0])
        left_diff_arr = np.diff(x_left)
        left_diff_sum = np.sum(left_diff_arr)
        left_avg = int(left_diff_sum/5)

        #remove inappropriate cases 
        for i in range(0, len(left_diff_arr)): 
            if (left_diff_arr[i] > (left_avg +20)) or (left_diff_arr[i] < (left_avg - 20)): #the value 20 can be modified
                self.slidng_list_left = []
                left_diff_sum = 0

        #extract x coordinates from sliding_list_right
        for i in range(0, len(sliding_list_right)): 
            x_right.append(sliding_list_right[i][0])
        right_diff_arr = np.diff(x_right)
        right_diff_sum = np.sum(right_diff_arr)
        right_avg = int(right_diff_sum/5)

        #remove inappropriate cases
        for i in range(0, len(right_diff_arr)): 
            if (right_diff_arr[i] > (right_avg +20)) or (right_diff_arr[i] < (right_avg - 20)): #the value 20 can be modified
                self.sliding_list_right = []
                right_diff_sum = 0

        avg_val = float((left_diff_sum + right_diff_sum)/2)

        # customize these valuse
        if avg_val > 20: #set the maximum value to 20
            avg_val = 20 
        elif avg_val < -20: #set the minimum value to -20
            avg_val = -20 
        norm_val = float(avg_val/float(20)) #normailze data from -1 to 1
        if norm_val < -0.35:
            cv2.arrowedLine(frame, (300, 340), (340, 340), (255, 0, 0), 4)
            self.steering_pub.publish(-1) #turn right
        elif norm_val > 0.35:
            cv2.arrowedLine(frame, (340, 340), (300, 340), (255, 0, 0), 4)
            self.steering_pub.publish(1) #turn left
        else:
            cv2.arrowedLine(frame, (320, 340), (320, 300), (255, 0, 0), 4)
            self.steering_pub.publish(0) #go straight

        return frame #return frame is for visualization

    def sliding_left(slef, img):
        left_list = []
        '''
        row: starting from y=259 to y=460, moving by 40
        col: starting from x=19 to y=300, moving by 5
        '''
        for j in range(259, img.shape[0] - 20, 40): 
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
        row: starting from x=259 to y=560, moving by 40
        col: starting from x=320 to y=620, moving by 5
        '''
        for j in range(259, img.shape[0] - 20, 40): 
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
        pts1 = np.float32([[60, 339], [580, 339], [0, 419], [640, 419]]) 
        pts2 = np.float32([[0, 0], [640, 0], [0, 480], [640, 480]])
        matrix = cv2.getPerspectiveTransform(pts1, pts2)
        result = cv2.warpPerspective(frame, matrix, (640, 480))
        return result

if __name__ == '__main__':
    a = LaneDetection()