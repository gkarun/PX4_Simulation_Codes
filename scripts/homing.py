#!/usr/bin/env python

import rospy
import time
import sys
import os
import numpy as np
import random
import tf

from geometry_msgs.msg import TwistStamped, PoseStamped
from math import *

rospy.init_node('homingNode',anonymous=True)

pi = 3.141592635

twist = TwistStamped()

pub = rospy.Publisher('/mavros/setpoint_velocity/cmd_vel', TwistStamped, queue_size=10)

def toDec(x, n):
	x_ = int(x*(10**n))
	return x_/(10**n + 0.0)

def toDegree(x):
	return x*180.0/pi

def signum2(x):
	if x>0:
		return 1
	else:  #if x<0:
		return -1
	


def obtain_R_theta_phi(odomMessage):
	
	x_c = odomMessage.pose.position.x
	y_c = odomMessage.pose.position.y
	z_c = odomMessage.pose.position.z

	R     = np.sqrt(x_c**2 + y_c**2 + z_c**2)
	phi   = np.arcsin(z_c/R)  #(z_c, np.sqrt(x_c**2 + y_c**2))  # -pi/2 to pi/2
	theta = np.arctan2(y_c,x_c)                        #  0   to 2*pi

	return R, theta, phi

def getRPY(odomMessage):
	
	quaternion = (
	    odomMessage.pose.orientation.x,
	    odomMessage.pose.orientation.y,
	    odomMessage.pose.orientation.z,
	    odomMessage.pose.orientation.w)
		
	euler = tf.transformations.euler_from_quaternion(quaternion)

	roll  =  euler[0]
	pitch = -euler[1]
	yaw   =  euler[2]
	
	roll  = np.arctan2(np.sin(roll),np.cos(roll))   # 0 to 2*pi
	pitch = np.arcsin(np.sin(pitch))#,np.cos(pitch)) # -pi/2, pi/2
	yaw   = np.arctan2(np.sin(yaw),np.cos(yaw))     # 0 to 2*pi

	return roll, pitch, yaw


def printAngles(gamma, beta, alpha, R, theta, phi, yaw_rate, pitch_rate, z_c):
	
	yaw_rate   = toDec(toDegree(yaw_rate),4)
	pitch_rate   = toDec(toDegree(pitch_rate),4)
	z_c   = toDec(z_c,4)
	R     = toDec(R,4)

	# print "y_r ", yaw_rate, "  p_r ", pitch_rate, 
	print " z_c ", z_c , " R ",R, " pitch ", toDec(toDegree(beta),3), " yaw ", toDec(toDegree(alpha),3)

	
def homingLinearVel(v, alpha, beta, sign3, K_z):

	x_c_dot = v*np.cos(alpha)*np.cos(beta)
	y_c_dot = v*np.sin(alpha)*np.cos(beta)
	z_c_dot = v*np.sin(beta) #-K_z * sign3   #v*np.sin(beta)
	
	return x_c_dot, y_c_dot, z_c_dot


def homingAngularVel(K_alpha, K_beta, sign1, sign2):        #sign1 = sgn(phi+beta); #sign2 = sgn(alpha - theta - pi)
	
	omega_x = 0                   # gamma_dot
	omega_y = -K_beta  * sign1    # beta_dot
	omega_z = -K_alpha * sign2    # alpha_dot

	return omega_x, omega_y, omega_z


def moveQuadrotor(v_x, v_y, v_z, roll_rate, pitch_rate, yaw_rate):
	
 	twist.twist.linear.x = v_x
	twist.twist.linear.y = v_y
	twist.twist.linear.z = v_z
	twist.twist.angular.x = roll_rate
	twist.twist.angular.y = pitch_rate
	twist.twist.angular.z = yaw_rate
	pub.publish(twist)	


def checkHome(odomMessage):

	x_c = odomMessage.pose.position.x
	y_c = odomMessage.pose.position.y
	z_c = odomMessage.pose.position.z
	
	homeFlag = 0

	if (x_c**2 + y_c**2 + z_c**2) < 2.25:
		homeFlag = 1
	
	return homeFlag


odomMessage = rospy.wait_for_message('/mavros/local_position/pose', PoseStamped, timeout = 0.3)
R, theta, phi = obtain_R_theta_phi(odomMessage)
gamma, beta, alpha = getRPY(odomMessage)

v       = 1.5    # m/s
K_alpha = 2.0    # rad/s
K_beta  = 200    # rad/s
K_z_max = 1.5 	 # m/s
K_z_coeff = 1.0


oneTimeFlag = 0

while not rospy.is_shutdown():
	
	odomMessage = rospy.wait_for_message('/mavros/local_position/pose', PoseStamped, timeout = 0.3)	
	homeFlag = checkHome(odomMessage)
	
	if homeFlag == 0:
		if oneTimeFlag == 0:
			print "Outside Home"
			oneTimeFlag = 1
			
		R, theta, phi = obtain_R_theta_phi(odomMessage)
		gamma, beta, alpha = getRPY(odomMessage)
		z_c = odomMessage.pose.position.z

		surf1 = np.arctan2(np.sin(phi + beta), np.cos(phi + beta))
		surf2 = np.arctan2(np.sin(alpha - theta - pi), np.cos(alpha - theta - pi))
		surf3 = phi

		surface1 = signum2(surf1)
		surface2 = signum2(surf2)
		surface3 = signum2(surf3)		
		
		K_z = K_z_max*(1.00 - np.exp(-K_z_coeff*z_c))

		v_x, v_y, v_z = homingLinearVel(v, alpha, beta, surface3, K_z)
		roll_rate, pitch_rate, yaw_rate = homingAngularVel(K_alpha, K_beta, surface1, surface2)
		
		printAngles(gamma, beta, alpha, R, theta, phi, yaw_rate, pitch_rate, z_c)
		moveQuadrotor(v, 0, 0, 0, pitch_rate, 0)
	
	else:	
		if oneTimeFlag == 1:
			print "Inside Home"
			oneTimeFlag = 0
		moveQuadrotor(0, 0, -0.2, 0, 0, 0)

	rospy.sleep(0.01)


