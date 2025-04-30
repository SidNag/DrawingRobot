#!/usr/bin/env python
import numpy as np
from scipy.linalg import expm, logm
from final_header import *

"""
Use 'expm' for matrix exponential.
Angles are in radian, distance are in meters.
"""
q1 = np.array([-0.15,0.15,0.01])
q2 = np.array([0,0.12,0.152]) + q1
q3 = np.array([0.244,0,0]) + q2
q4 = np.array([0.213,-0.093,0]) + q3
q5 = np.array([0,0.083,0]) + q4
q6 = np.array([0.083,0,0]) + q5
w1 = np.array([0,0,1])
w2 = np.array([0,1,0])
w3 = np.array([0,1,0])
w4 = np.array([0,1,0])
w5 = np.array([1,0,0])
w6 = np.array([0,1,0])
xPose = q6[0]
yPose = q6[1] + (59 + 82)/1000
zPose = q6[2] + 0.0535
v1 = np.cross(-w1,q1)[np.newaxis].T
v2 = np.cross(-w2,q2)[np.newaxis].T
v3 = np.cross(-w3,q3)[np.newaxis].T
v4 = np.cross(-w4,q4)[np.newaxis].T
v5 = np.cross(-w5,q5)[np.newaxis].T
v6 = np.cross(-w6,q6)[np.newaxis].T

PI = np.pi

def skew(A):
    a = A[0]
    b = A[1]
    c = A[2]
    
    skewA = np.array([[0, -c, b], [c, 0, -a], [-b, a, 0]])
    return skewA

def Get_MS():
	# =================== Your code starts here ====================#
	# Fill in the correct values for S1~6, as well as the M matrix
	S1 = np.concatenate((w1, np.cross(-w1,q1)))
	S2 = np.concatenate((w2, np.cross(-w2,q2)))
	S3 = np.concatenate((w3, np.cross(-w3,q3)))
	S4 = np.concatenate((w4, np.cross(-w4,q4)))
	S5 = np.concatenate((w5, np.cross(-w5,q5)))
	S6 = np.concatenate((w6, np.cross(-w6,q6)))
	
	M = np.array([
		[0,-1,0, xPose],
		[0,0,-1, yPose],
		[1,0,0, zPose],
		[0,0,0,1]
	])
 
	S1 = np.block([
		[skew(w1), v1],
		[np.zeros((1,4))]
	])
	S2 = np.block([
		[skew(w2), v2],
		[np.zeros((1,4))]
	])
	S3 = np.block([
		[skew(w3), v3],
		[np.zeros((1,4))]
	])
	S4 = np.block([
		[skew(w4), v4],
		[np.zeros((1,4))]
	])
	S5 = np.block([
		[skew(w5), v5],
		[np.zeros((1,4))]
	])
	S6 = np.block([
		[skew(w6), v6],
		[np.zeros((1,4))]
	])
	
 
	S = [S1, S2, S3, S4, S5, S6]




	# ==============================================================#
	return M, S


"""
Function that calculates encoder numbers for each motor
"""
def lab_fk(theta1, theta2, theta3, theta4, theta5, theta6):
# Initialize the return_value
	return_value = [None, None, None, None, None, None]

	# =========== Implement joint angle to encoder expressions here ===========
	# print("Foward kinematics calculated:\n")

	# =================== Your code starts here ====================#
	S1, S2, S3, S4, S5, S6 = Get_MS()[1][0], Get_MS()[1][1], Get_MS()[1][2], Get_MS()[1][3], Get_MS()[1][4], Get_MS()[1][5]
	T = expm(S1*theta1)@expm(S2*theta2)@expm(S3*theta3)@expm(S4*theta4)@expm(S5*theta5)@expm(S6*theta6)@Get_MS()[0]








	# ==============================================================#

	# print(str(T) + "\n")

	return_value[0] = theta1 + PI
	return_value[1] = theta2
	return_value[2] = theta3
	return_value[3] = theta4 - (0.5*PI)
	return_value[4] = theta5
	return_value[5] = theta6

	return return_value


"""
Function that calculates an elbow up Inverse Kinematic solution for the UR3
"""
def lab_invk(xWgrip, yWgrip, zWgrip, yaw_WgripDegree):
	# =================== Your code starts here ====================#
	yaw = np.radians(yaw_WgripDegree)
	L1 = 0.152
	xBase = xWgrip + 0.15
	yBase = yWgrip - 0.15
	zBase = zWgrip - 0.01 - L1
 
	xcen = xBase - 0.0535*np.cos(yaw)
	ycen = yBase - 0.0535*np.sin(yaw)
	zcen = zBase + 0.059
 
	thetacen = np.arctan2(ycen,xcen)
	rcen = np.sqrt(xcen**2 + ycen**2)
	L2 = 0.11
	thetaInt1 = np.arcsin(L2/rcen)
 
	theta1 = thetacen - thetaInt1
 ###################################################### 
 
 
	L7 = 0.083
	L8 = 0.082

	xend0 = xcen - L7*np.cos(theta1) + (L7 + 0.027)*np.sin(theta1)
	yend0 = ycen - L7*np.sin(theta1) - (L7 + 0.027)*np.cos(theta1)
	zend = zcen + L8
	end = np.array([xend0,yend0,zend])
 
	
	L3 = 0.244
	L5 = 0.213
 
	# base = np.array([0,0,L1])
 
	c = np.linalg.norm(end)
	thetaInt3 = np.arccos((c**2 - L3**2 - L5**2)/(-2*L3*L5))
	thetaInt2 = np.arccos((L5**2 - c**2 - L3**2)/(-2*c*L3))
	# thetaInt21 = np.arcsin((end[0]-base[0])/c)
	# print('test1',np.linalg.norm(end[0:2]),c)
	# thetaInt21 = np.arccos((np.linalg.norm(end[0:2]))/c)
	thetaInt21 = np.arcsin((end[2])/c)
	# print(thetaInt2, thetaInt21)
 

	
	theta2 = -(thetaInt2 + thetaInt21)
	theta3 = np.pi - thetaInt3
	theta4 = -(theta3 - (-theta2))
	theta5 = -np.pi/2
	theta6 = np.pi/2 - yaw + theta1
	# print(theta1,theta2,theta3,theta4,theta5,theta6)
 
	degreeList = [np.degrees(theta1),np.degrees(theta2),np.degrees(theta3),np.degrees(theta4),np.degrees(theta5),np.degrees(theta6)]
	# print('Degree List ',degreeList)
	# ==============================================================#
	return lab_fk(theta1, theta2, theta3, theta4, theta5, theta6)
