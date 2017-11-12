## Project 2: Inverse Kinematics Pick & Place
Deepak Trivedi, November 11, 2017

KUKA KR210: RRRRRR Serial Manipulator
---


**Overview of Steps**  

1. Set up your ROS Workspace.
2. Download or clone the [project repository](https://github.com/udacity/RoboND-Kinematics-Project) into the ***src*** directory of your ROS Workspace.  
3. Experiment with the forward_kinematics environment and get familiar with the robot.
4. Launch in [demo mode](https://classroom.udacity.com/nanodegrees/nd209/parts/7b2fd2d7-e181-401e-977a-6158c77bf816/modules/8855de3f-2897-46c3-a805-628b5ecf045b/lessons/91d017b1-4493-4522-ad52-04a74a01094c/concepts/ae64bb91-e8c4-44c9-adbe-798e8f688193).
5. Perform Kinematic Analysis for the robot following the [project rubric](https://review.udacity.com/#!/rubrics/972/view).
6. Fill in the `IK_server.py` with your Inverse Kinematics code. 


[//]: # (Image References)

[image4]: ./misc_images/Classic-DHparameters.png
[image5]: ./misc_images/kr210urdfxacro.PNG
[image6]: ./misc_images/forward_kinematics_2.PNG
[image7]: ./misc_images/forward_kinematics_1.PNG
[image8]: ./misc_images/derivation_1.jpg
[image9]: ./misc_images/derivation_2.jpg
[image10]: ./misc_images/pick_show.PNG

## [Rubric](https://review.udacity.com/#!/rubrics/972/view) Points
### Here I will consider the rubric points individually and describe how I addressed each point in my implementation.  

---
### Writeup / README

#### 1. Provide a Writeup / README that includes all the rubric points and how you addressed each one.  You can submit your writeup as markdown or pdf.  

You're reading it!

### Kinematic Analysis
#### 1. Run the forward_kinematics demo and evaluate the kr210.urdf.xacro file to perform kinematic analysis of Kuka KR210 robot and derive its DH parameters.

The Kuka210 Forward kinematics simulator was launched using 

`$ roslaunch kuka_arm forward_kinematics.launch`

Getting acquainted with the forward kinematics using this simulator went a long way in getting a 'feel' for the manipulator kinematics. This general understanding of how the arm responds to joint actuation was very helpful later in debugging.  

In the following image, the manipulator is shown actuated using an arbitrary set of joint parameters. 
![alt text][image6]
Another example is shown below. 
![alt text][image7]

The geometrical specifications of the manipulator necessary to derive the relevant DH parameters were obtained by parsing the `kr210.urdf.xacro` file. The image below shows a snapshot of one of the relevant sections of the file.    

![alt text][image5]

With the help of these parameters, and a hand sketch showing all the joints in the unactuated pose (all joint angles equal to zero, the DH parameter table was obtained.

![alt text][image8]

The table is presented in the next section.  
                                                                                                                                         
#### Modified DH Parameters

The DH parameters consist of the following four transformations: 

1. `d`: offset along previous `z` to the common normal
2. `theta`: angle about previous `z`, from old `x` to new <math>x</math>
3. `a`: length of the common normal.  Assuming a revolute joint, this is the radius about previous `z`.
4. `alpha`: angle about common normal, from old `z` axis to new `z` axis
 

![alt text][image4]
This image is due to Wikimedia Commons contributor Ollydbg (https://commons.wikimedia.org/w/index.php?curid=31730433)

Please refer to the Wikipedia page on DH transformation for citation and more details.
https://en.wikipedia.org/wiki/Denavit%E2%80%93Hartenberg_parameters
 

The table below provides the values of these parameters for the  for Kuka KR210 obtained using the information provided in the `kr210.urdf.xacro` file in conjunction with the kinematic diagram presented above.  
 
Links | alpha(i-1) | a(i-1) | d(i-1) | theta(i)
---   | ---        | ---    | ---    | ---
0->1  | 0          | 0      | 0.75   | q1
1->2  | - pi/2     | 0.35   | 0      | -pi/2 + q2
2->3  | 0          | 1.25   | 0      | q3
3->4  |  -pi/2     | -0.054  | 1.5    | q4
4->5  | pi/2       | 0      | 0      | q5
5->6  | -pi/2      | 0      | 0      |  q6
6->EE | 0          | 0      | 0.303  | 0

#### 2. Using the DH parameter table you derived earlier, create individual transformation matrices about each joint. In addition, also generate a generalized homogeneous transform between base_link and gripper_link using only end-effector(gripper) pose.

The derivation of the transformation matrices was done interactively using a Jupyter Notebook. The notebook, `./Kuka210_Inverse_Kinematics_Analysis.ipynb` is also uploaded for reference. 

The way DH parameters are defined here, the transformation matrix could be obtained as follows: 

1. Rotate about x[i-1] by alpha[i-1]
2. Translate along x[i-1] by a[i-1]
3. Rotate about resulting axis z[i] by theta[i]
4. Translate along axis z[i] by d[i]

This leads to a chain of four sequential transformations:

`T[i-1, i] = R(​x[i-1], ​alpha[i-1]) ​*​ D(x​ [i-1], ​a[i-1]) ​*​ R(​z[i], ​theta[i]) ​*​ D(​z[i], ​d[i])`

The code snippets below show how this symbolic manipulation was performed. More details are available in the Jupyter Notebook mentioned above. 

#### Import relevant libraries
```python
	from mpmath import *
	from sympy import sin, cos, atan2, simplify, pi, symbols, sqrt
	from sympy.matrices import Matrix
	from numpy import array
	import numpy as np
```
#### Define symbols
```python
	q1, q2, q3, q4,q5,q6,q7 = symbols('q1:8')
	d1, d2, d3, d4,d5,d6,dG = symbols('d1:8')
	a0, a1, a2, a3, a4, a5, a6 = symbols('a0:7')
	alpha0, alpha1, alpha2, alpha3, alpha4, alpha5, alpha6 = symbols('alpha0:7')
```

#### Assign values for later substitution
```python
	s = {alpha0: 0,  a0:   0, d1: 0.75, 
     	     alpha1: -pi/2,  a1: 0.35, d2: 0,  
	     alpha2: 0,  a2: 1.25, d3: 0,
	     alpha3: -pi/2,  a3:  -0.054, d4: .125,
	     alpha4: pi/2, a4:0, d5:0,
	     alpha5:-pi/2, a5:0, d6:0,
	     alpha6:0, a6:0, dG:0.303,
	     q7:0, q2: -pi/2+q2}

```

#### Define transformation matrices for each joint
```python
	T0_1 = Matrix([[             cos(q1),            -sin(q1),            0,              a0],
	               [ sin(q1)*cos(alpha0), cos(q1)*cos(alpha0), -sin(alpha0), -sin(alpha0)*d1],
	               [ sin(q1)*sin(alpha0), cos(q1)*sin(alpha0),  cos(alpha0),  cos(alpha0)*d1],
	               [                   0,                   0,            0,               1]])
	T0_1 = T0_1.subs(s)

```

Similarly, transformations `T1_2`, `T2_3`, `T3_4`, `T4_5`, `T5_6`, and `T6_G` are assigned. These are not produced here for brevity. Please refer to the Jupyter Notebook for more details as necessary. 

A corrective transformation between the DH parameter convention and Gazebo convention is obtained. This takes care of the reference frame used in the `URDF` file and the reference frame defined for our DH parameter definition.

```python
	R_z = Matrix([[np.cos(np.pi), -np.sin(np.pi), 0, 0],[np.sin(np.pi), np.cos(np.pi),  0, 0],[0,           0,          1, 0], [0,           0,          0, 1]])
	
	R_y = Matrix([[cos(-np.pi/2), 	0,           sin(-np.pi/2), 	0],
			     [0, 	  	1,           0, 		0],
			     [-sin(-np.pi/2),      0,          cos(-np.pi/2), 	0],
			     [0,           	0,          0, 			1]])
	
	R_corr = simplify(R_z * R_y)

```

This matrix turns out to be 

```python			
	R_corr   = 	Matrix([
						[0.0,  0.0, 1.0, 0.0],
						[  0, -1.0,   0,   0],
						[  1,    0, 0.0,   0],
						[  0,    0,   0, 1.0]])
 ```

In order to generate a generalized homogeneous transform between `base_link` and `gripper_link` using only end-effector(gripper) pose, we create a function to generate transformations as follows:  
 
```python
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
```

Given the gripper (roll, pitch, yaw), the generalized `base_link` to `gripper_link` transformation could be obtained with the help of this function. For example:
 
```python

	roll, pitch, yaw = 1,1,1
	Rrpy = Rot(roll=roll,pitch=pitch,yaw=yaw) * R_corr
	Rrpy.evalf(2)

    Matrix([
    [ 0.29, 0.072, -0.95,   0],
    [ 0.45, -0.89, 0.072,   0],
    [-0.84, -0.45, -0.29,   0],
    [    0,     0,     0, 1.0]])

```

 

#### 3. Decouple Inverse Kinematics problem into Inverse Position Kinematics and inverse Orientation Kinematics; doing so derive the equations to calculate all individual joint angles.



In order to solve the kinematics problem, we first realize that the position of the wrist center depends only on the first three joint parameters, `q1`,`q2` and `q3`.  

```python
	nx = Rrpy[0,2]
	ny = Rrpy[1,2]
	nz = Rrpy[2,2]
```

```python
	    wx = px - (d6.subs(s)+dG.subs(s))*nx
	    wy = py - (d6.subs(s)+dG.subs(s))*ny
	    wz = pz - (d6.subs(s)+dG.subs(s))*nz

```
Due to this fact, the inverse position problem and the inverse orientation problems could be uncoupled, as shown below. 

#### Inverse position problem

The image below shows a derivation of the parameters `theta1`, `theta2` and `theta3` in terms of the wrist center coordinates obtained above and the DH parameters. 

![alt text][image9]

In code, this is done as follows 


```python
	    
	    A = sqrt(d4**2+a3**2)	
	    S1 = sqrt(wx**2+wy**2) - a1
	    B = sqrt(S1**2 + (wz-d1)**2)
	    C = a2
	
	    alpha = atan2(wz-d1,S1)
	    beta = cosine_rule(B,C,A)
	    gamma = cosine_rule(A,C,B)
	    delta = atan2(d4,abs(a3))
```
Here the cosine rule function is defined as: 

```python

	def cosine_rule(a,b,c):	
  		cos_C = (a*a + b*b - c*c) / (2*a*b)
  		sin_C = sqrt(1 - cos_C * cos_C)
  		return atan2(sin_C, cos_C)

```

Once these angles are obtained using the cosine rule, the first three joint angles can be obtained as follows:
 
```python
	
	    theta1 = (atan2(wy,wx).subs(s)).evalf(3)
	    theta2 = ((pi/2 - beta - alpha).subs(s)).evalf(3)
	    theta3 = (-(gamma - delta).subs(s)).evalf(3)

```
The knowledge of `theta1`, `theta2`, `theta3` solves the inverse position problem for the wrist center. 

#### Inverse orientation problem

In order to solve the inverse orientation problem we realize that there are two independent ways of obtaining the transformation from `link_3` to `link_gripper`. These two transformations should be identical. Therefore, equating these two transformations would provide us a set of equations that could be solved for `theta4`, `theta5` and `theta6`. 

The first method is to use the overall `Rrpy` matrix, and the inverse of the (now known) `R0_3` matrix to obtain the `R3_G` matrix. 
  
```python
	R3_G = ((T0_3.subs({q1:theta1,q2:theta2,q3:theta3})).T[:3,:3]*Rrpy[:3,:3]).evalf(2)
```
The other method is to sequentially apply transformations `T3_4`,`T4_5`, `T5_6` and `T6_G` obtained earlier, as shown below: 

```python

	R3_G1 = simplify(T3_4*T4_5*T5_6*T6_G)
	R3_G1


    Matrix([
    [-sin(q4)*sin(q6) + cos(q4)*cos(q5)*cos(q6), -sin(q4)*cos(q6) - sin(q6)*cos(q4)*cos(q5), -sin(q5)*cos(q4), -0.303*sin(q5)*cos(q4) - 0.054],
    [                           sin(q5)*cos(q6),                           -sin(q5)*sin(q6),          cos(q5),          0.303*cos(q5) + 0.125],
    [-sin(q4)*cos(q5)*cos(q6) - sin(q6)*cos(q4),  sin(q4)*sin(q6)*cos(q5) - cos(q4)*cos(q6),  sin(q4)*sin(q5),          0.303*sin(q4)*sin(q5)],
    [                                         0,                                          0,                0,                              1]])
```

Since `R3_G = R3_G1`, we should be able to equate individual terms of these matrices, and thereby calculate `q4`, `q5` and `q6`. 

In order to do this in a way that preserves the sign of the angles, we would use `atan2` with certain elements of the matrix and avoid `asin` and `acos`. This is shown below: 

```python

	sin_q4_sin_q5 = R3_G[2, 2]
	cos_q4_sin_q5 =  -R3_G[0, 2]
	  
	sin_q5 = sqrt(cos_q4_sin_q5**2 + sin_q4_sin_q5**2) 
	cos_q5 = R3_G[1, 2]
	    
	sin_q6_sin_q5 = -R3_G[1, 1]
	cos_q6_sin_q5 = R3_G[1, 0] 
	  
	theta4 = atan2(sin_q4_sin_q5, cos_q4_sin_q5)
	theta5 = atan2(sin_q5, cos_q5)
	theta6 = atan2(sin_q6_sin_q5, cos_q6_sin_q5)
```

This completes the derivation of all the six joint angles. 
 

The next section talks about efficient implementation of the inverse kinematics code. 

### Project Implementation

The first version of the code directly implemented symbolic math as shown in the previous section of the report. This is computationally extremely expensive and unnecessary. A more efficient, purely numerical version was therefore developed, based on symbolic analysis done in the Jupyter Notebook, with relevant results used in the Python code for numerical computation. 

The symbolic version of the code is available at `IK_server_symbolic.py`. 
The numerical version of the code is available at `IK_server.py`.

The initial code had a number of mistakes, to do with signs and values of DH parameters. The forward kinematics tool, launched using `$ roslaunch kuka_arm forward_kinematics.launch` was indispensable for debugging the math and the code.  

The snapshot below is a demonstration of the effectiveness of the inverse kinematics code. The manipulator was able to successfully pick and place 9 out of 11 cylinders. Both cylinders that were not picked belong to the middle platform of the rightmost (from Robot's stance) column. It was unclear why these cylinders were not picked up, since the end effector did successfully reach the cylinders on both occasions. 

![alt text][image10]

### Next steps

Following would be a few relevant next steps in this project: 

1. Determine the root cause for the 2 out of 11 failures. It appears that the end effectors does reach the correct location, but is unable to grasp the cylinders. This needs to be investigated further.
 
2. It is not clear if the paths generated by the path planning algorithm are optimized at all, or if so in what sense. The overall efficiency of the process could be improved if paths could be optimized further. 

### Appendix

The relevant portions of the numerical version of the code are produced here for the sake of completeness. Please explore the rest of the git repository for the full version of the code.

```python

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
```

```python
	def cosine_rule(a,b,c):
	  cos_C = (a*a + b*b - c*c) / (2*a*b)
	  sin_C = sqrt(1 - cos_C * cos_C)
	  return atan2(sin_C, cos_C)
```

```python

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
```

```python

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
```
