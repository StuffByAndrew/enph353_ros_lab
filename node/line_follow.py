#!/usr/bin/env python
from logging import error
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2 as cv
import matplotlib.pyplot as plt
import numpy as np
import timeit


class LineFollower():
    bridge = CvBridge()
    angle_PID = {
        'p': 40,
        'i': 0,
        'd': 0
    }
    img_dim = None

    def detects_line(self, thresh_img):
        return np.count_nonzero(thresh_img==255)>100

    def line_follow(self, img):
        # Find center of mass of road
        _, thresh = cv.threshold(img,120,255,cv.THRESH_BINARY_INV)
        # cv.imshow('a', thresh)
        # cv.waitKey(0)

        move = Twist()
        if self.detects_line(thresh):
            center_M = cv.moments(thresh)
            cX = int(center_M["m10"] / center_M["m00"])
            cY = int(center_M["m01"] / center_M["m00"])
            cv.circle(img, (cX, cY), 10, (0,0,255), -1)
            
            
            ang_err = float(cX - self.img_dim[0]/2)
            
            # Normalise
            ang_err /= self.img_dim[0]
            # fix direction and multiply by pid
            ang_g = self.angle_PID['p'] * -ang_err
    
            move.linear.x = 0.7
            move.angular.z = ang_g
        else:
            move.linear.x = 0
            move.angular.z = 0.8

        print(move)
        follow_pub.publish(move)

    def proccess_image(self, data):
        self.img_dim = data.width, data.height
        cv_image = self.bridge.imgmsg_to_cv2(data, desired_encoding='mono8')
        # self.histogram(cv_image)
        self.line_follow(cv_image)
 
    def histogram(self, img):
        n, bins, patches = plt.hist(gray.flatten(), bins=50)
        plt.show()
    

if __name__ == '__main__':
    rospy.init_node('line_follower', anonymous=True)

    lf = LineFollower()

    follow_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    rospy.Subscriber("/R1/camera1/image_raw", Image, lf.proccess_image)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()
