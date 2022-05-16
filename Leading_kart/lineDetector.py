"""
########################################################################################################################
# COMPONENT: lineDetector.py
# DESCRIPTION: Publisher code for detecting yellow lines. The kart will follow a central line, if present. Else it will 
#              look to right or left for new yellow lines to follow.
# PROJECT: LHPU_Capstone_final_project
#
# AUTHOR: Raviteja Chandu & Federico Deidda
#
# LAST MODIFIED: 04/14/2022
########################################################################################################################
"""

#!/usr/bin/env python

# --------------------------------------------------- IMPORTS ----------------------------------------------------------

import cv2
import numpy as np
import math
import Jetson.GPIO as GPIO
import rospy
from std_msgs.msg import Float32MultiArray
import os

# --------------------------------------------------- VARIABLES --------------------------------------------------------


angle=0
vid = cv2.VideoCapture(0)
angar=Float32MultiArray()


# --------------------------------------------------- FUNCTIONS --------------------------------------------------------


def mathe():
		pub = rospy.Publisher('Add',Float32MultiArray, queue_size=1)
		rospy.init_node('mathe', anonymous=True)
		rate = rospy.Rate (100) #100hz

		while(True):
				
				# Capture the video frame by frame
				ret, frame1 = vid.read()
				h,w,_=frame1.shape
				frame = cv2.GaussianBlur(frame1, (9,9),0)
				gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

				# Display the resulting frame
				hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
			 	lower_yellow = np.array([21,100,30])
			    upper_yellow = np.array([31,255,255])

				mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
				mask2=cv2.inRange(hsv, lower_yellow, upper_yellow)
				res = cv2.bitwise_and(frame,frame, mask= mask)
                                cv2.imshow('res',res)

				ROI=np.array([[	(round(w*0.167),h),(round(w*0.167),round(h*0.5)),(round(0.83*w),round(0.5*h)),(round(w*0.83),h)]],dtype= np.int32)
				ROIL=np.array([[(0,h),(0,0),(round(0.167*w),0),(round(w*0.167),h)]],dtype= np.int32)
				ROIR=np.array([[(round(w*0.833),h),(round(w*0.833),0),(w,0),(w,h)]],dtype= np.int32)

				blank= np.zeros_like(mask)
				blank2= np.zeros_like(mask2)
				roif= cv2.fillPoly(blank, ROI,255)
				roii= cv2.bitwise_and(mask,roif)
				roil= cv2.fillPoly(blank2, ROIL,255)
				roir= cv2.fillPoly(blank2, ROIR,255)
				sumrl = cv2.bitwise_or(roir, roil)
				roirl= cv2.bitwise_and(mask2,sumrl)

				cv2.imshow('Region of Interest3', roirl)

				contours, hierarchy = cv2.findContours(roii, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
				contoursrl, hierarchy = cv2.findContours(roirl, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)

				laneshift=180

				if len(contoursrl)!=0:
					areas = [cv2.contourArea(c) for c in contoursrl]
					max_index = np.argmax(areas)
					cnt=contoursrl[max_index]
					((x, y), (width, height), angle_of_rotation) = cv2.minAreaRect(cnt)
					box = cv2.boxPoints(((x, y), (width, height), angle_of_rotation))
					box = np.int0(box)
					cv2.drawContours(frame,[box], 0, (255,0, 0), 3)
					laneshift=180*math.atan(((w*0.5)-x)/(h-y))/3.14
					
				if len(contours)!=0:
					areas = [cv2.contourArea(c) for c in contours]
					max_index = np.argmax(areas)
					cnt=contours[max_index]
					((x, y), (width, height), angle_of_rotation) = cv2.minAreaRect(cnt)
					box = cv2.boxPoints(((x, y), (width, height), angle_of_rotation))
					box = np.int0(box)
					cv2.drawContours(frame,[box], 0, (0, 255, 0), 3)
					angle=180*math.atan(((w*0.5)-x)/(h-y))/3.14 
					print(angle)
					rospy.loginfo(angle)
					angar.data=[angle,laneshift]
					pub.publish(angar)

				else:
					angle=180
					rospy.loginfo(angle)
					angar.data=[angle,laneshift]
					pub.publish(angar)

				cv2.imshow('Contours', frame)
				
				if cv2.waitKey(1) & 0xFF == ord('q'):
					break


# ---------------------------------------------------- MAIN ------------------------------------------------------------


if __name__ =='__main__':
	try:
		mathe()
	except rospy.ROSInterruptException:
		pass
	except KeyboardInterrupt:
		GPIO.cleanup	
	except cv2.waitKey(1) & 0xFF == ord('q'):
			# After the loop release the cap object
			vid.release()
			# Destroy all the windows
			cv2.destroyAllWindows()
