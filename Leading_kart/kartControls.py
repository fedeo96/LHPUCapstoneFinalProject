"""
########################################################################################################################
# COMPONENT: kartControls.py
# DESCRIPTION: Subscriber code to implement throttle control strategies based on voltage, distance from the obstacle and
#              the steering angle.
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
from sensor_msgs.msg import LaserScan
from adafruit_servokit import ServoKit
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Float32
import Jetson.GPIO as GPIO


# --------------------------------------------------- VARIABLES --------------------------------------------------------


GPIO.cleanup
kit = ServoKit(channels=16)
paprev=0
c=0
flag=1


# ------------------------------------------------------ CLASS ---------------------------------------------------------


class CarRun:
	def  __init__(load):
		load.dist= 0
		load.angle= 0
		load.voltage= 0
		
	def Lid(load,ldata):
		# retrieving lidar data from the sensor

		data = [num*100 for num in ldata.ranges[:]]
		f=data[353:359]
		f2=data[0:14]
		full=f+f2
		front = min(full)  # [350:359]
		load.dist=front

	def Cam(load,cdata):
		# retrieving camera data

		steer=cdata.data
		load.angle=steer

	def Voltage(load,volt):
		# retrieving voltage data form the battery

		v=volt.data
		load.voltage=v
		CarRun.Move(load)
		
	def Move(load):
		# implementation of control strategies

		a=load.angle
		pa=abs(a[0])
		print("Angle = %f",a)
		d=load.dist
		v=load.voltage
		global paprev
		global flag
		global c

		if d<70:
			if flag==1: c=a[1]
			flag=0
			if c>0:
				kit.servo[0].angle= 45
				kit.continuous_servo[1].throttle = 0.137+(0.03*(v-8.4)/(6-8.4)) # y=1.132614 - 0.2366207*x + 0.01386054*x^2
			elif c<0:
				kit.servo[0].angle= 135
				kit.continuous_servo[1].throttle = 0.137+(0.03*(v-8.4)/(6-8.4))
			else:
				kit.continuous_servo[1].throttle = 0
				kit.servo[0].angle= 87
		else:
			flag=1
			if a[0]==0:
				kit.servo[0].angle= 87
				kit.continuous_servo[1].throttle = 0.15+(0.03*(v-8.4)/(6-8.4))
			elif a[0]==180:
				kit.continuous_servo[1].throttle = 0
				kit.servo[0].angle= 87
			else :
				if a[0]>0:
					kit.servo[0].angle= 87-(pa)*50/90
					kit.continuous_servo[1].throttle = 0.137-(pa*0.01/90)+(0.03*(v-8.4)/(6-8.4))
					paprev=pa
				elif a[0]<0:
					kit.servo[0].angle= 87+(pa)*50/90
					kit.continuous_servo[1].throttle = 0.137-(pa*0.01/90)+(0.03*(v-8.4)/(6-8.4))
					paprev=pa


# --------------------------------------------------- FUNCTIONS --------------------------------------------------------


def listener():
	rospy.init_node('listener', anonymous=True)
	Car= CarRun()
	rospy.Subscriber("Add",Float32MultiArray,Car.Cam)
	rospy.Subscriber("/scan",LaserScan,Car.Lid)
	rospy.Subscriber("Voltage",Float32,Car.Voltage)
	rospy.spin()


# ---------------------------------------------------- MAIN ------------------------------------------------------------


if __name__ =='__main__':
	try:
		listener()
	except KeyboardInterrupt:
			kit.continuous_servo[9].throttle = 0
			GPIO.cleanup	
