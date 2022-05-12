"""
########################################################################################################################
# COMPONENT: sensorFusions.py
# DESCRIPTION: Publisher and Subscriber code to read the voltage from the Arduino and send it to the kartControls file
#              and also subscribe to the LiDAR sensor and plot "X" or "!" on the display based on the distance to the 
#              object using I2C communication protocol.
# PROJECT: LHPU_Capstone_final_project
#
# AUTHOR: Raviteja Chandu & Federico Deidda
#
# LAST MODIFIED: 04/14/2022
########################################################################################################################
"""

#!/usr/bin/env python

# --------------------------------------------------- IMPORTS ----------------------------------------------------------


import smbus
import time
import rospy
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Float32
from sensor_msgs.msg import LaserScan
import os


# --------------------------------------------------- VARIABLES --------------------------------------------------------


front, left, right = 0, 0, 0
bus = smbus.SMBus(0)
address = 0x09
pub = rospy.Publisher('Voltage',Float32, queue_size=1)


# --------------------------------------------------- FUNCTIONS --------------------------------------------------------


def write(value):
        bus.write_byte(address,value)


def read():
        light = bus.read_byte(address)
        return light


def callback(data):
    data = [num*100 for num in data.ranges[:]]
    print("\n\n\n\n\n")
    print("FRONT")
    print(data[353:359],data[0:14])
    print("\n")

    f=data[353:359]
    f2=data[0:14]
    full=f+f2
    front = min(full)
    left = min(data[15:49])
    right = min(data[290:334])
    
    write(int(front))
    time.sleep(0.1)
    voltage=read()
    voltage=(voltage/25.0)-1.6
    pub.publish(voltage)
    print(voltage)


def listener():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("/scan", LaserScan, callback)
    rospy.spin()


# ---------------------------------------------------- MAIN ------------------------------------------------------------


if __name__=='__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        os.system("rosrun sonar_motors stop.py")
    except KeyboardInterrupt:
        os.system("rosrun sonar_motors stop.py")
