#!/usr/bin/env python3

import rospy

from ackermann_msgs.msg import AckermannDriveStamped
from f1tenth_simulator.msg import pid_input
import math
import numpy as np

pub = rospy.Publisher('drive', AckermannDriveStamped, queue_size=1)

kp = 0.5
kd = 0.2
kp_vel = 0.1
kd_vel = 0.1

ki = 0.0
servo_offset = 0.0
prev_error = 0.0 
error = 0.0
integral = 0.0
vel_input = 1.0


def control(data):
	global integral
	global prev_error
	global vel_input
	global kp
	global ki
	global kd
	global kd_vel
	global kp_vel
	velocity = data.pid_vel
	angle = servo_offset
	error = (5*data.pid_error)
	print("Error Control"),error
	if error!=0.0:
		#if abs(error - prev_error)>0.5: 	
		# 	integral = integral + error	
			control_error = kp*(error) + kd*((error - prev_error))# + ki*integral
			print("Control"), control_error
		# integral = integral/1.3
		
		# print "Control error",control_error
			angle = angle + control_error*np.pi/180
		# print "Control error",control_error

			control_error_vel = kp_vel*(error) + kd_vel*(error - prev_error)
		# print "Control error velocity",control_error_vel

			#velocity = velocity - abs(control_error_vel)
			velocity = velocity + abs(control_error_vel)

	
	prev_error = error
	

	if angle > 30*np.pi/180:
		angle = 30*np.pi/180
	if angle < -30*np.pi/180:
		angle = -30*np.pi/180
	


	print("Velocity",velocity)
	print("Angle in Degrees",angle*180/np.pi) # Just for reference
	msg = AckermannDriveStamped()

	# velocity = 1
	# # if angle > 20*np.pi/180 or angle < -20*np.pi/180:
	# # 	velocity = 0.3
	
	if angle >= 10*np.pi/180 or angle <= -10*np.pi/180:
		velocity = 1

	if angle > 20*np.pi/180 or angle < -20*np.pi/180:
		velocity = 0.6

	if angle >= -1*np.pi/180 and angle <= 1*np.pi/180:
		velocity = 5

	if velocity < 0:
		velocity = 0

	if velocity >= 5:
		velocity = 5
	


	print("Velocity",velocity)
	print("Angle",angle)


	msg.drive.speed = velocity
	msg.drive.steering_angle = angle
	pub.publish(msg)

def listener():
	global kp
	global ki
	global kd
	global vel_input
	kp = float(input("Enter Kp Value: "))
	ki = float(input("Enter Ki Value: "))
	kd = float(input("Enter Kd Value: "))
	vel_input = float(input("Enter Velocity: "))
	rospy.init_node('pid_controller', anonymous=True)
	rospy.Subscriber('error', pid_input, control)
	rospy.spin()


if __name__ == '__main__':
	print("Listening to error for PID")
	listener()
