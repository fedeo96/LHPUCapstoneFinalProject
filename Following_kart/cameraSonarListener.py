"""
########################################################################################################################
# COMPONENT: cameraSonarListener.py
# DESCRIPTION: Listener file. It listens to the data received from the cameraSonarTalker.py file and then applies the 
#              control strategies based on distance to the leading kart and steerg angle.
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
from adafruit_servokit import ServoKit
import time
import os
import socketio 
from gevent import pywsgi
from engineio.payload import Payload
import threading


# --------------------------------------------------- VARIABLES --------------------------------------------------------


# variables initialization
kit = ServoKit(channels=16)
kit.servo[0].angle = 85

threshold = 40
throttle = 0.145
pGain = 0.001
dGain = 0.0055
error, prev_error = 0, 0

Payload.max_decode_packets = 100000

sio = socketio.Server(async_mode='gevent', ping_interval =.2 , ping_timeout = 5, cors_allowed_origins ="*")
app = socketio.WSGIApp(sio)


# --------------------------------------------------- FUNCTIONS --------------------------------------------------------


def startServer():
    print("starting server")
    pywsgi.WSGIServer(('0.0.0.0',8081), app).serve_forever()


@sio.event
def connect(sid, environ):
    print('\n\r connected to server \n\r ########## \n\r    \n\r  \n\r', sid) 
     
    
@sio.event
def disconnect(sid):
    global controlData
    controlData = (0,0)
    print('disconnect ', sid)
    

def sendData(data):
    print("SENDING", data)
    sio.emit('information',data, ignore_queue =True)
    #socketio.sleep(0)


def getThrottle(distance):
    global error, prev_error
    error = distance - threshold
    throttle_pid = throttle + (error)*pGain + (error - prev_error)*dGain
    prev_error = error
    rospy.loginfo("PID: %f", throttle_pid)
    curr_time = time.time()
    sendData({"time": curr_time, "throttle": throttle_pid})
    return throttle_pid


def callback(data):

    ang = data.data[0]
    dist = data.data[1]
    len_cont = data.data[2]

    # LOGIC for Servo motors
    if ang > 45 and ang < 135:
        kit.servo[0].angle = ang
        if dist > threshold-20 and dist < threshold+30:
            kit.continuous_servo[1].throttle = getThrottle(dist) 
        else:
            kit.continuous_servo[1].throttle = 0
        if len_cont == 0:
            kit.servo[0].angle = 85
            kit.continuous_servo[1].throttle = 0
            rospy.loginfo("PID:", 0)


def listener():
    rospy.Subscriber("capstone", Float32MultiArray, callback)


# ---------------------------------------------------- MAIN ------------------------------------------------------------


if __name__=='__main__':
    rospy.init_node('listener', anonymous=True)
    serverThread = threading.Thread(target=startServer)
    serverThread.start()
    listener()
    rospy.spin()
