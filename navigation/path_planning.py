#! /usr/bin/env python3
import math
from re import X
import numpy as np
import rospy
from f1tenth_simulator.msg import pid_input
from sensor_msgs.msg import LaserScan

import matplotlib.pyplot as plt

angle_range_lidar = 360
# angle_range = np.pi
car_length = 1.5

desired_trajectory = 0.8

vel=1.25

# write publisher for custom message

pub = rospy.Publisher('error', pid_input, queue_size=10)
## ***********************CREATE A CUSTOM .MSG named PIDinput.msg in msg directory ****** 
# should have atleast ,pid_vel,pid_error, Feel free to add any more if required


def getRange(data, angle):
    # Get lidar range data in form of a list from the message
    
#    zri= 30*(len(data.ranges)/270)
#    i= int(angle*len(data.ranges)/360)
#    a=data.ranges[zri]
#    b=data.ranges[i]
#    return data.ranges[i]

    i=(math.floor(1080*angle/360)%1080)
    x=data.ranges[i]
    if x<0 :
        x=0
    elif x>1000:
        x=1000
    return x

#def Path_Planning():
 #   pass
    # decide the deviation from correct path publish in PID as message
    # write algorithm that give error to Controls script for steering


def callback(data):
    # Function to call other modules

    theta=70
    a = getRange(data,theta+90)
    b = getRange(data,90)
    swing = math.radians(theta)
    alpha = math.atan((a*math.cos(swing)-b)/(a*math.sin(swing)))
    AB = b*(math.cos(alpha))
    AC= vel*10
    CD = AB+AC*(math.sin(alpha))

    error = desired_trajectory - CD
    # publish custom message here
    
    msg = pid_input()
    msg.pid_error = error
    msg.pid_vel = vel
    pub.publish(msg)

if __name__ == '__main__':
    print("Laser node started")
    rospy.init_node('dist_finder', anonymous=True)
    rospy.Subscriber('scan', LaserScan, callback)
    rospy.spin()
