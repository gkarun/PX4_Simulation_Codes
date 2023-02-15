#!/usr/bin/env python

import rospy
import time
import sys
import os
import numpy as np
import random
import tf

from geometry_msgs.msg import TwistStamped, PoseStamped
from mavros_msgs.msg import AttitudeTarget
from math import *

rospy.init_node('homingNode',anonymous=True)

pi = 3.141592635

attitudeRaw = AttitudeTarget()

pub = rospy.Publisher('/mavros/setpoint_raw/attitude', AttitudeTarget, queue_size=10)


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
	phi   = np.arcsin(z_c/R)                   # -pi/2 to pi/2
	theta = np.arctan2(y_c,x_c)                #  0   to 2*pi

	return R, theta, phi


def getXYZRPY(odomMessage):
	
	quaternion = (
	    odomMessage.pose.orientation.x,
	    odomMessage.pose.orientation.y,
	    odomMessage.pose.orientation.z,
	    odomMessage.pose.orientation.w)
		
	euler = tf.transformations.euler_from_quaternion(quaternion)

	roll  = euler[0]
	pitch = -euler[1]
	yaw   = euler[2]
	
	roll  = np.arctan2(np.sin(roll),np.cos(roll))   # 0 to 2*pi
	pitch = np.arcsin(np.sin(pitch))		#,np.cos(pitch)) # -pi/2, pi/2
	yaw   = np.arctan2(np.sin(yaw),np.cos(yaw))     # 0 to 2*pi
	
	x_c = odomMessage.pose.position.x
	y_c = odomMessage.pose.position.y
	z_c = odomMessage.pose.position.z

	return x_c, y_c, z_c, roll, pitch, yaw


def printAngles(gamma, beta, alpha, R, theta, phi, z_c, y_c, x_c):
	
	z_c = toDec(z_c,4)
	R   = toDec(R,4)
	x_c = toDec(x_c,4)
	y_c = toDec(y_c,4)
	
	
	print " z_c ", z_c , " y_c ", y_c, " x_c ", x_c, " pitch ", toDec(toDegree(beta),4), " yaw ", toDec(toDegree(alpha),4)

	
def homingLinearVel(v, alpha, beta):

	x_c_dot = v*np.cos(alpha)*np.cos(beta)
	y_c_dot = v*np.sin(alpha)*np.cos(beta)
	z_c_dot = v*np.sin(beta) 
	
	return x_c_dot, y_c_dot, z_c_dot


def homingAngularVel(K_alpha, K_beta, sign1, sign2):        #sign1 = sgn(phi+beta); #sign2 = sgn(alpha - theta - pi)
	
	omega_x = 0                   # gamma_dot
	omega_y = -K_beta  * sign1    # beta_dot
	omega_z = -K_alpha * sign2    # alpha_dot

	return omega_x, omega_y, omega_z


def moveQuadrotor(v_x, v_y, v_z, roll, pitch, yaw):
	
 	attitudeRaw.body_rate.x = v_x
	attitudeRaw.body_rate.y = v_y
	attitudeRaw.body_rate.z = v_z

	quat = tf.transformations.quaternion_from_euler(roll, pitch, yaw)      # possibility of error
	attitudeRaw.orientation.x = quat[0]
	attitudeRaw.orientation.y = quat[1]
	attitudeRaw.orientation.z = quat[2]
	attitudeRaw.orientation.w = quat[3]
	
	attitudeRaw.thrust = v_z
	pub.publish(attitudeRaw)	


def checkHome(odomMessage):

	x_c = odomMessage.pose.position.x
	y_c = odomMessage.pose.position.y
	z_c = odomMessage.pose.position.z
	
	homeFlag = 0

	if (x_c**2 + y_c**2 + z_c**2) < 2.25:
		homeFlag = 1
	
	return homeFlag



v       = 1.0    # m/s
K_alpha = 1.0    # rad/s
K_beta  = 1.0    # rad/s
K_z_max = 1.5 	 # m/s
K_z_coeff = 1.0

def f_alpha(alpha, x_c, y_c):
	
	theta = np.arctan2(y_c,x_c)
	return -K_alpha*signum2(np.arctan2(np.sin(alpha - theta - pi), np.cos(alpha - theta - pi)))


def f_beta(beta, x_c, y_c, z_c):
	
	R   = np.sqrt(x_c**2 + y_c**2 + z_c**2)
	phi = np.arcsin(z_c/R)
	return -K_beta*signum2(np.arctan2(np.sin(phi + beta), np.cos(phi + beta)))

	
def f_x(alpha, beta):
	
	return v*np.cos(alpha)*np.cos(beta)


def f_y(alpha, beta):
	
	return v*np.sin(alpha)*np.cos(beta)


def f_z(beta):
	
	return v*np.sin(beta)


def integrateRK4(alpha, beta, x_c, y_c, z_c, h):
	
	k_alpha_1 = f_alpha(alpha, x_c, y_c)
	k_beta_1  = f_beta(beta, x_c, y_c, z_c)
        k_x_1     = f_x(alpha, beta)
	k_y_1	  = f_y(alpha, beta)
	k_z_1	  = f_z(beta)

	k_alpha_2 = f_alpha(alpha + k_alpha_1*h/2.0, x_c + k_x_1*h/2.0, y_c + k_y_1*h/2.0)
	k_beta_2  = f_beta(beta + k_beta_1*h/2.0, x_c + k_x_1*h/2.0, y_c + k_y_1*h/2.0, z_c + k_z_1*h/2.0)
        k_x_2     = f_x(alpha + k_alpha_1*h/2.0, beta + k_beta_1*h/2.0)
	k_y_2	  = f_y(alpha + k_alpha_1*h/2.0, beta + k_beta_1*h/2.0)
	k_z_2	  = f_z(beta + k_beta_1*h/2.0)

	k_alpha_3 = f_alpha(alpha + k_alpha_2*h/2.0, x_c + k_x_2*h/2.0, y_c + k_y_2*h/2.0)
	k_beta_3  = f_beta(beta + k_beta_2*h/2.0, x_c + k_x_2*h/2.0, y_c + k_y_2*h/2.0, z_c + k_z_2*h/2.0)
        k_x_3     = f_x(alpha + k_alpha_2*h/2.0, beta + k_beta_2*h/2.0)
	k_y_3	  = f_y(alpha + k_alpha_2*h/2.0, beta + k_beta_2*h/2.0)
	k_z_3	  = f_z(beta + k_beta_2*h/2.0)

	k_alpha_4 = f_alpha(alpha + k_alpha_3*h, x_c + k_x_3*h, y_c + k_y_3*h)
	k_beta_4  = f_beta(beta + k_beta_3*h, x_c + k_x_3*h, y_c + k_y_3*h, z_c + k_z_3*h)
        k_x_4     = f_x(alpha + k_alpha_3*h, beta + k_beta_3*h)
	k_y_4	  = f_y(alpha + k_alpha_3*h, beta + k_beta_3*h)
	k_z_4	  = f_z(beta + k_beta_3*h)

	alpha_ = alpha + h*(k_alpha_1 + 2*k_alpha_2 + 2*k_alpha_3 + k_alpha_4)/6.0
	beta_  = beta  + h*(k_beta_1  + 2*k_beta_2  + 2*k_beta_3  + k_beta_4 )/6.0
	
	x_c_ = x_c + h*(k_x_1 + 2*k_x_2 + 2*k_x_3 + k_x_4)/6.0
	y_c_ = y_c + h*(k_y_1 + 2*k_y_2 + 2*k_y_3 + k_y_4)/6.0
	z_c_ = z_c + h*(k_z_1 + 2*k_z_2 + 2*k_z_3 + k_z_4)/6.0

	return alpha_, beta_, x_c_, y_c_, z_c_



oneTimeFlag = 0

while not rospy.is_shutdown():
	
	odomMessage = rospy.wait_for_message('/mavros/local_position/pose', PoseStamped, timeout = 0.3)	
	homeFlag = checkHome(odomMessage)
	
	if homeFlag == 0:
		if oneTimeFlag == 0:
			print "Outside Home"
			oneTimeFlag = 1
			
		R, theta, phi = obtain_R_theta_phi(odomMessage)
		x_c, y_c, z_c, gamma, beta, alpha = getXYZRPY(odomMessage)
		z_c = odomMessage.pose.position.z
		
		print "\n\n\n\nCurrent\n\n"
		printAngles(gamma, beta, alpha, R, theta, phi, z_c, y_c, x_c)
		
		alpha_, beta_, x_c_, y_c_, z_c_ = integrateRK4(alpha, beta, x_c, y_c, z_c, 1)
		v_x_, v_y_, v_z_ = homingLinearVel(v, alpha_, beta_)
		moveQuadrotor(v_x_, v_y_, v_z_, 0.0, beta_, alpha_)
		
		print "\n\nWaypoint\n\n"
		printAngles(gamma, beta_, alpha_, R, theta, phi, z_c_, y_c_, x_c_)
	
	else:	
		if oneTimeFlag == 1:
			print "Inside Home"
			oneTimeFlag = 0
		moveQuadrotor(0, 0, -0.2, 0, 0, 0)

	rospy.sleep(0.01)


