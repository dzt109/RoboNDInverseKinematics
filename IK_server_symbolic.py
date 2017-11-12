#!/usr/bin/env python

# Copyright (C) 2017 Electric Movement Inc.
#
# This file is part of Robotic Arm: Pick and Place project for Udacity
# Robotics nano-degree program
#
# All Rights Reserved.

# Author: Harsh Pandya

# import modules
import rospy
import tf
from kuka_arm.srv import *
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import Pose
from mpmath import *
from sympy import sin, cos, atan2, simplify, pi, symbols, sqrt
from sympy.matrices import Matrix
from numpy import array
import numpy as np


def cosine_rule(a,b,c):
  cos_C = (a*a + b*b - c*c) / (2*a*b)
  sin_C = sqrt(1 - cos_C * cos_C)
  return atan2(sin_C, cos_C)

def Rot(roll=0,pitch=0,yaw=0,tx=0,ty=0,tz=0):
    alpha = yaw
    beta = pitch
    gamma = roll
    R = Matrix([
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
		
        ### Your FK code here
        # Create symbols
	#
	q1, q2, q3, q4,q5,q6,q7 = symbols('q1:8')
	d1, d2, d3, d4,d5,d6,dG = symbols('d1:8')
	a0, a1, a2, a3, a4, a5, a6 = symbols('a0:7')
	alpha0, alpha1, alpha2, alpha3, alpha4, alpha5, alpha6 = symbols('alpha0:7')
	#   
	# Create Modified DH parameters
	#
	s = {alpha0: 0,  a0:   0, d1: 0.75, 
     	     alpha1: -pi/2,  a1: 0.35, d2: 0,  
	     alpha2: 0,  a2: 1.25, d3: 0,
	     alpha3: -pi/2,  a3:  -0.054, d4: .125,
	     alpha4: pi/2, a4:0, d5:0,
	     alpha5:-pi/2, a5:0, d6:0,
	     alpha6:0, a6:0, dG:0.303,
	     q7:0, q2: -pi/2+q2}
	#            
	# Define Modified DH Transformation matrix
	#
	#
	# Create individual transformation matrices
	#
	T0_1 = Matrix([[             cos(q1),            -sin(q1),            0,              a0],
	               [ sin(q1)*cos(alpha0), cos(q1)*cos(alpha0), -sin(alpha0), -sin(alpha0)*d1],
	               [ sin(q1)*sin(alpha0), cos(q1)*sin(alpha0),  cos(alpha0),  cos(alpha0)*d1],
	               [                   0,                   0,            0,               1]])
	T0_1 = T0_1.subs(s)

	T1_2 = Matrix([[             cos(q2),            -sin(q2),            0,              a1],
	               [ sin(q2)*cos(alpha1), cos(q2)*cos(alpha1), -sin(alpha1), -sin(alpha1)*d2],
        	       [ sin(q2)*sin(alpha1), cos(q2)*sin(alpha1),  cos(alpha1),  cos(alpha1)*d2],
	               [                   0,                   0,            0,               1]])
	T1_2 = T1_2.subs(s)

	T2_3 = Matrix([[             cos(q3),            -sin(q3),            0,              a2],
		       [ sin(q3)*cos(alpha2), cos(q3)*cos(alpha2), -sin(alpha2), -sin(alpha2)*d3],
		       [ sin(q3)*sin(alpha2), cos(q3)*sin(alpha2),  cos(alpha2),  cos(alpha2)*d3],
		       [                   0,                   0,            0,               1]])
	T2_3 = T2_3.subs(s)

	T3_4 = Matrix([[             cos(q4),            -sin(q4),            0,              a3],
		       [ sin(q4)*cos(alpha3), cos(q4)*cos(alpha3), -sin(alpha3), -sin(alpha3)*d4],
		       [ sin(q4)*sin(alpha3), cos(q4)*sin(alpha3),  cos(alpha3),  cos(alpha3)*d4],
		       [                   0,                   0,            0,               1]])
	T3_4 = T3_4.subs(s)

	T4_5 = Matrix([[             cos(q5),            -sin(q5),            0,              a4],
		       [ sin(q5)*cos(alpha4), cos(q5)*cos(alpha4), -sin(alpha4), -sin(alpha4)*d5],
		       [ sin(q5)*sin(alpha4), cos(q5)*sin(alpha4),  cos(alpha4),  cos(alpha4)*d5],
		       [                   0,                   0,            0,               1]])
	T4_5 = T4_5.subs(s)

	T5_6 = Matrix([[             cos(q6),            -sin(q6),            0,              a5],
		       [ sin(q6)*cos(alpha5), cos(q6)*cos(alpha5), -sin(alpha5), -sin(alpha5)*d6],
		       [ sin(q6)*sin(alpha5), cos(q6)*sin(alpha5),  cos(alpha5),  cos(alpha5)*d6],
		       [                   0,                   0,            0,               1]])
	T5_6 = T5_6.subs(s)

	T6_G = Matrix([[             cos(q7),            -sin(q7),            0,              a6],
		       [ sin(q7)*cos(alpha6), cos(q7)*cos(alpha6), -sin(alpha6), -sin(alpha6)*dG],
		       [ sin(q7)*sin(alpha6), cos(q7)*sin(alpha6),  cos(alpha6),  cos(alpha6)*dG],
		       [                   0,                   0,            0,               1]])
	T6_G = T6_G.subs(s)

	#
	#
	# Extract rotation matrices from the transformation matrices
	#
	T0_2 = simplify(T0_1 * T1_2)
	T0_3 = simplify(T0_2 * T2_3)
	T0_4 = simplify(T0_3 * T3_4)
	T0_5 = simplify(T0_4 * T4_5)
	T0_6 = simplify(T0_5 * T5_6)
	T0_G = simplify(T0_6 * T6_G)
	#
	print "T0_1"
	print T0_1
	print "T0_2"
	print T0_2
	print "T0_3"
	print T0_3
	print "T0_4"
	print T0_4
	print "T0_5"
	print T0_5
	print "T0_6"
	print T0_6
	print "T0_G"
	print T0_G

	#Correction
	R_z = Matrix[[cos(np.pi), -sin(np.pi), 0, 0],
		     [sin(np.pi), cos(np.pi),  0, 0],
		     [0,           0,          1, 0],
		     [0,           0,          0, 1]]

	R_y = Matrix[[cos(-np.pi), 	0,           sin(-np.pi), 	0],
		     [0, 	  	1,           0, 		0],
		     [-sin(-np.pi),      0,          cos(-np.pi), 	0],
		     [0,           	0,          0, 			1]]

	R_corr = simplify(R_z * R_y)
        ###
	T_Total = simplify(T0_G * R_corr)
	print "T_total"		
	print T_total

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
	    Rrpy = (Rot(roll=roll,pitch=pitch,yaw=yaw) * R_corr).evalf(5)
	    nx = Rrpy[0,2]
	    ny = Rrpy[1,2]
	    nz = Rrpy[2,2]
	
	    wx = px - (d6.subs(s)+dG.subs(s)).evalf(5)*nx
	    wy = py - (d6.subs(s)+dG.subs(s)).evalf(5)*ny
	    wz = pz - (d6.subs(s)+dG.subs(s)).evalf(5)*nz

	    # Calculate joint angles using Geometric IK method
	    #
	    #
	    A = sqrt(d4**2+a3**2).evalf(5)	
	    S1 = sqrt(wx**2+wy**2) - a1.evalf(5)
	    B = sqrt(S1**2 + (wz-d1)**2)
	    C = a2
	
	    alpha = atan2(wz-d1,S1).subs(s).evalf(5)
	    beta = cosine_rule(A,C,B).subs(s).evalf(5)
	    gamma = cosine_rule(B,C,A).subs(s).evalf(5)
	    delta = atan2(d4,abs(a3)).subs(s).evalf(5)
	    theta1 = atan2(wy,wx)
	    theta2 = (pi/2 - beta - alpha)
	    theta3 = -(gamma - delta)
            ###

	    R3_G = (T0_3.subs({q1:theta1,q2:theta2,q3:theta3})).T*Rrpy

	    sin_q4 = R3_G[2, 2]
	    cos_q4 =  -R3_G[0, 2]
	    sin_q5 = sqrt(R3_G[0, 2]**2 + R3_G[2, 2]**2) 
	    cos_q5 = R3_G[1, 2]
    
	    sin_q6 = -R3_G[1, 1]
	    cos_q6 = R3_G[1, 0] 
  
	    theta4 = atan2(sin_q4, cos_q4)
   	    theta5 = atan2(sin_q5, cos_q5)
	    theta6 = atan2(sin_q6, cos_q6)
		
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
