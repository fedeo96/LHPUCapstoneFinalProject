"""
########################################################################################################################
# COMPONENT: cameraSonarTalker.py
# DESCRIPTION: Publisher file. It receives data from both the camera and the sonar sensor placed in front of the kart.
#              Then, it will publish the data that will be used by the Listener.
# PROJECT: LHPU_Capstone_final_project
#
# AUTHOR: Raviteja Chandu & Federico Deidda
#
# LAST MODIFIED: 04/14/2022
########################################################################################################################
"""

#!/usr/bin/env python3

# --------------------------------------------------- IMPORTS ----------------------------------------------------------


import rospy
from std_msgs.msg import Float32MultiArray
import RPi.GPIO as GPIO
import time
import cv2
import math
import numpy as np


# --------------------------------------------------- VARIABLES --------------------------------------------------------


# pin definiton
ECHO_FRONT = 24
TRIG_FRONT = 26

# initializing video capture
cap = cv2.VideoCapture(0)
information = Float32MultiArray()

# pin initialization
GPIO.setmode(GPIO.BOARD) 
GPIO.setup(ECHO_FRONT, GPIO.IN)
GPIO.setup(TRIG_FRONT, GPIO.OUT)
GPIO.setwarnings(False)


# --------------------------------------------------- FUNCTIONS --------------------------------------------------------


#functions definitions
def resize(frame):
    return cv2.resize(frame, (640,480))


def rectangle(frame):
    return cv2.rectangle(frame, (200, 0), (400, 200), [255,255,255], 3) 

   
def line(frame, (x1,y1), (x2,y2), color, thickness):  
    return cv2.line(frame, (x1,y1), (x2,y2), color, thickness)


def talker():
    pub = rospy.Publisher('capstone', Float32MultiArray, queue_size = 10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(100)
    while True:
    
        # capture the video frame by frame 
        ret, frame = cap.read()

        # define the color range for the mask
        green_low = np.array([53,40,20])
        green_up = np.array([78,255,255])

        # scalin of the video to 640x480
        frame = resize(frame)

        # apply gaussian blur
        blur = cv2.GaussianBlur(frame,(5,5),0)
    
        # color conversion and apply the mask
        hsv = cv2.cvtColor(blur, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, green_low, green_up)
        res = cv2.bitwise_and(frame,frame, mask=mask)

        '''
        Remember: canny and contours are almost the same thing.
        Canny draws the contours by taking the result of the bitwise and between the mask and the frame
        Contours draws the contours directly in the raw frame without any image processing
        '''

        # define region of interest
        ROI = np.array([[(0,120), (640,120), (640,360), (0,360)]], dtype=np.int32)
        blank = np.zeros_like(mask)
        region_of_interest = cv2.fillPoly(blank, ROI, 255)
        region_of_interest_image = cv2.bitwise_and(mask, region_of_interest)

        # draw contours
        contours, _ = cv2.findContours(region_of_interest_image, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
        contour_thickness = 3
        cv2.drawContours(frame, contours, -1, (0,255,0), 3)
    
        if len(contours) != 0:
        
            # find the biggest contour (c_max) by the area
            c_max = max(contours, key=cv2.contourArea)
            rect = cv2.minAreaRect(c_max)
            box= cv2.boxPoints(rect)
            box= np.int0(box)

            # draw the biggest contour in blue
            cv2.drawContours(frame, [box], 0, (0,0,255), 2)

            # take the coordinates from the box
            xb1 = box[0][0] 
            xb2 = box[1][0]
            xb3 = box[2][0]
            xb4 = box[3][0]
            yb1 = box[0][1]
            yb2 = box[1][1]
            yb3 = box[2][1]
            yb4 = box[3][1]

            xm1 = (xb2+xb1)/2 
            ym1 = (yb2+yb1)/2
            xm2 = (xb4+xb3)/2 
            ym2 = (yb4+yb3)/2

            center_x = (xm1+xm2)/2
            center_y = (ym1+ym2)/2

            # draw the center line in order to obtain a reference for the steering
            frame = line(frame, (center_x,center_y), (320,480), (255,0,0), 3)

            ang = math.degrees(math.atan2((480-center_y),(320-center_x)))
            dist = get_distance(TRIG_FRONT, ECHO_FRONT)
            len_cont = len(contours)

            information.data = [ang, dist, len_cont]
            
            # send the angle and the length of the contour 
            rospy.loginfo(information)
            pub.publish(information)
            rate.sleep()

        else:

            ang = 85
            dist = get_distance(TRIG_FRONT, ECHO_FRONT)
            len_cont = 0
            
            information.data = [ang, dist, len_cont]
            
            # send the angle and the length of the contour 
            rospy.loginfo(information)
            pub.publish(information)
            rate.sleep()

        # display the resulting frame
        cv2.imshow('frame', frame)

        # the 'q' button is set as the
        # quitting button you may use any
        # desired button of your choice
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # After the loop release the cap object
    cap.release()
    # Destroy all the windows
    cv2.destroyAllWindows()


def get_distance(TRIGGER, ECHO):
    GPIO.output(TRIGGER, True)
    time.sleep(0.0001)
    GPIO.output(TRIGGER, False)
    start = time.time()
    stop = 0
    
    while GPIO.input(ECHO) == 0:
        start = time.time()
    
    while GPIO.input(ECHO) == 1:
        stop = time.time()

    time_diff = stop - start
    difference = (time_diff*(34300))/2  
    return int(difference)


# ---------------------------------------------------- MAIN ------------------------------------------------------------


if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        GPIO.cleanup
    except KeyboardInterrupt:
        GPIO.cleanup
