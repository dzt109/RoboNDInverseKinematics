#!/usr/bin/env python

# Copyright (C) 2017 Electric Movement Inc.
#
# This file is part of Robotic Arm: Pick and Place project for Udacity
# Robotics nano-degree program
#
# All Rights Reserved.

# Author: Harsh Pandya


import rospy
import tf
from kuka_arm.srv import *
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import Pose
from mpmath import *
from numpy import array
import numpy as np
from numpy import sin, cos, sqrt, pi
from math import atan2


def cosine_rule(a,b,c):
  cos_C = (a*a + b*b - c*c) / (2*a*b)
  sin_C = sqrt(1 - cos_C * cos_C)
  return atan2(sin_C, cos_C)

def Rot(roll=0,pitch=0,yaw=0,tx=0,ty=0,tz=0):
    alpha = yaw
    beta = pitch
    gamma = roll
    R = np.matrix([
    [1.0*cos(alpha)*cos(beta), -1.0*sin(alpha)*cos(gamma) + sin(beta)*sin(gamma)*cos(alpha), 1.0*sin(alpha)*sin(gamma) + sin(beta)*cos(alpha)*cos(gamma),tx],
    [1.0*sin(alpha)*cos(beta),  sin(alpha)*sin(beta)*sin(gamma) + 1.0*cos(alpha)*cos(gamma), sin(alpha)*sin(beta)*cos(gamma) - 1.0*sin(gamma)*cos(alpha),ty],
    [          -1.0*sin(beta),                                     1.0*sin(gamma)*cos(beta),                                    1.0*cos(beta)*cos(gamma),tz],
        [0,0,0,1]])

    return R

def handle_calculate_IK(req):
    rospy.loginfo("Received %s eef-poses from the plan" % len(req.poses))
    if len(req.poses) < 1:
        print "No valid poses received"
        return -1
    else:
		

	alpha0 = 0
	a0 =   0
	d1 = 0.75 
        alpha1 = -np.pi/2
        a1 = 0.35
	d2 = 0  
        alpha2 = 0
        a2 = 1.25
	d3 = 0
	alpha3= -np.pi/2
	a3 =  -0.054
	d4= 1.5
        alpha4= np.pi/2
	a4 =0
	d5=0
	alpha5 =-np.pi/2
	a5 =0
	d6 =0
	alpha6 = 0
	a6=0
	dG=0.303
	q7=0
	#q2: -pi/2+q2}

        R_corr = np.matrix([[0.0,  0.0,  1.0, 0.0],
	    	            [  0, -1.0,    0,   0],
		            [  1,    0, -0.0,   0],
		            [  0,    0,    0, 1.0]])
        # Initialize service response
        joint_trajectory_list = []
        for x in xrange(0, len(req.poses)):
            # IK code starts here
            joint_trajectory_point = JointTrajectoryPoint()

	    # Extract end-effector position and orientation from request
	    # px,py,pz = end-effector position
	    # roll, pitch, yaw = end-effector orientation
            px = req.poses[x].position.x
            py = req.poses[x].position.y
            pz = req.poses[x].position.z

            (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(
                [req.poses[x].orientation.x, req.poses[x].orientation.y,
                    req.poses[x].orientation.z, req.poses[x].orientation.w])
     
            ### Your IK code here 
	    # Compensate for rotation discrepancy between DH parameters and Gazebo
	    #
	    Rrpy = np.matmul(Rot(roll=roll,pitch=pitch,yaw=yaw) , R_corr)
	    nx = Rrpy[0,2]
	    ny = Rrpy[1,2]
	    nz = Rrpy[2,2]
	
	    wx = px - (d6+dG)*nx
	    wy = py - (d6+dG)*ny
	    wz = pz - (d6+dG)*nz

	    # Calculate joint angles using Geometric IK method
	    #
	    #
	    A = sqrt(d4**2+a3**2)	
	    S1 = sqrt(wx**2+wy**2) - a1
	    B = sqrt(S1**2 + (wz-d1)**2)
	    C = a2
	
	    alpha = atan2(wz-d1,S1)
	    beta = cosine_rule(B,C,A)
	    gamma = cosine_rule(A,C,B)
	    delta = atan2(d4,abs(a3))
	    q1 =  atan2(wy,wx)
	    q2 =  (pi/2 - beta - alpha)
	    q3 =  -(gamma - delta)
            ###
	    T0_3T = np.matrix([
			   [sin(q2 + q3)*cos(q1), sin(q1)*sin(q2 + q3), cos(q2 + q3), 0],
			   [cos(q1)*cos(q2 + q3), sin(q1)*cos(q2 + q3),-sin(q2 + q3), 0],
			   [-sin(q1),       cos(q1),    0, 0],
			   [(1.25*sin(q2) + 0.35)*cos(q1), (1.25*sin(q2) + 0.35)*sin(q1), 1.25*cos(q2) + 0.75, 1]])

	    R3_G = np.matmul(T0_3T,Rrpy)

	    sin_q4_sin_q5 = R3_G[2, 2]
	    cos_q4_sin_q5 =  -R3_G[0, 2]
	      
	    sin_q5 = sqrt(cos_q4_sin_q5**2 + sin_q4_sin_q5**2) 
	    cos_q5 = R3_G[1, 2]
		
	    sin_q6_sin_q5 = -R3_G[1, 1]
	    cos_q6_sin_q5 = R3_G[1, 0] 

	    theta1 = q1
	    theta2 = q2 - 0*pi/2
	    theta3 = q3

	      
	    theta4 = atan2(sin_q4_sin_q5, cos_q4_sin_q5)
	    theta5 = atan2(sin_q5, cos_q5)
	    theta6 = atan2(sin_q6_sin_q5, cos_q6_sin_q5)
		
            # Populate response for the IK request
            # In the next line replace theta1,theta2...,theta6 by your joint angle variables
	    joint_trajectory_point.positions = [theta1, theta2, theta3, theta4, theta5, theta6]
	    joint_trajectory_list.append(joint_trajectory_point)

        rospy.loginfo("length of Joint Trajectory List: %s" % len(joint_trajectory_list))
        return CalculateIKResponse(joint_trajectory_list)


def IK_server():
    # initialize node and declare calculate_ik service
    rospy.init_node('IK_server')
    s = rospy.Service('calculate_ik', CalculateIK, handle_calculate_IK)
    print "Ready to receive an IK request"
    rospy.spin()

if __name__ == "__main__":
    IK_server()
