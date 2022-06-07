#!/usr/bin/env python2
#!/usr/bin/env python3

from __future__ import print_function
import roslib
import socket
roslib.load_manifest('subcamera')
import sys
import rospy
import cv2
import numpy as np
import time
import imutils
import math
import os
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from multiprocessing import *
from gorev import gorev2
from PID import PID

def pid_process(center_x, center_y, hatax, hatay, pid):

    p = PID()

    while pid.value:
        vx.value, vy.value, hatax.value, hatay.value  = p.update(center_x.value, center_y.value)
    
    print("PID'den cikiliyor")

def detected(frame):
    # ORTAMA, ISIK MIKTARINA GORE VS BU DEGERLER BIRAZ DAHA DARALTILABILIR
    # EN GENIS ARALIK: 0,100,100 + 10,255,255 VE 160,100,100 + 180,255,255
    """lower_red1 = np.array([0, 125, 100])
    upper_red1 = np.array([10, 255, 255])
    lower_red2 = np.array([160, 125, 100])
    upper_red2 = np.array([180, 255, 255])"""

    lower_red1 = np.array([0, 150, 100])
    upper_red1 = np.array([10, 255, 255])
    lower_red2 = np.array([170, 150, 100])
    upper_red2 = np.array([180, 255, 255])
    
    while True:

        # APPLYING PRE-PROCESSING TO IMAGE

        # import necessary packages
        frame = imutils.resize(frame, width=600)
        blurred = cv2.GaussianBlur(frame, (11, 11), 0)

        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

        # DEFINE THE LOWER AND UPPER BOUNDARIES OF COLOR RED
        mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
        mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
        mask = cv2.add(mask1, mask2)

        # FIND COUNTORS IN THE MASK AND INITIALIZE THE CURRENT (X, Y) CENTER OF THE CIRCLE
        cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cnts = imutils.grab_contours(cnts)
        center = (0,0)
        global merkez
        merkez = (0,0)
        # DETERMINE WHETHER OR NOT RED CIRCLE IS DETECTED
        if len(cnts) > 0:
            # print("sayi cnt: ", len(cnts)) --> 1 cikiyor

            # FIND THE LARGEST CONTOUR AND ITS CENTER(LARGEST CIRCLE)
            c = max(cnts, key=cv2.contourArea)
            # print("c: ", c)
            ((x, y), radius) = cv2.minEnclosingCircle(
                c)  # outer scope of circle, works even there is not a perfect circle
            M = cv2.moments(c)
	    merkez = (int(x),int(y))
	 
	    if M["m00"] > 0:
                center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))  # formula of finding the center of circle
            # only proceed if the radius meets a minimum size
            #print("radius: ", radius)
            if radius > 10:  # duruma gore esik belirlenebilir
                cv2.circle(frame, (int(x), int(y)), int(radius),
                           (0, 255, 255), 2)
                cv2.circle(frame, center, 5, (0, 0, 255), -1),
                cv2.circle(frame, (int(x),int(y)), 5, (0, 255, 0), -1)
        
        center_x.value = merkez[0]
        center_y.value = merkez[1]
	
        cv2.circle(frame, (int(frame.shape[1] / 2.0), int(frame.shape[0] / 2.0)), 5, (0, 0, 0), -1)
        return frame

class image_converter:

    def __init__(self):
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("camera/image_raw",Image,self.callback)

    def callback(self,data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            cv_image = detected(cv_image)
        except CvBridgeError as e:
            print(e)
            
        cv2.imshow("Kamera", cv_image)
        cv2.waitKey(3)

def main(args):
    ic = image_converter()
    rospy.init_node('image_converter', anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
        cv2.destroyAllWindows()

if __name__ == '__main__':
    with Manager() as manager:
        center_x = manager.Value("i", 0)
        center_y = manager.Value("i", 0)
        uzaklik = manager.Value("i", 0)
        vx = manager.Value("f", 0)
        vy = manager.Value("f", 0)
        hatax = manager.Value("f", 0)
        hatay = manager.Value("f", 0)
        pid = manager.Value("i", 0)
        pid_process = Process(target=pid_process, args=(center_x, center_y, hatax, hatay, pid))
        node = Process(target=main, args=(sys.argv))
        gorev2 = Process(target=gorev2, args=(center_x, uzaklik, pid, vx, vy, hatax, hatay))
        node.start()
        gorev2.start()
        while 1:
            if pid.value != 0:
                break
        pid_process.start()
        gorev2.join()
        node.join()
            
            
            
