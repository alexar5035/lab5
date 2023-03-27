#!/usr/bin/env python3
import rospy
import numpy as np
import math
from robot_vision_lectures.msg import XYZarray
from robot_vision_lectures.msg import SphereParams

# initial matrices
a_matrix = []
b_matrix = []

# empty array
P = np.array([])

# empty sphere parameter message
SParams = SphereParams()

def receive(point_data):
	global a_matrix
	global b_matrix
	
	# loop through point_data to create A and B matrices
	# A = 2Xn, 2Yn, 2Zn, 1
	# B = Xn^2 + Yn^2 + Zn^2
	for point in point_data.points:
			a_matrix.append([2*point.x, 2*point.y, 2*point.z, 1])
			b_matrix.append([point.x**2 + point.y**2 + point.z**2])
			
def model_fitting(a_matrix, b_matrix):
	global P
	# arrays created with initial matrices
	A = np.array(a_matrix)
	B = np.array(b_matrix)
	
	# calculate P and account for mismatched dimensions with requirements
	try: 
		# calculate the product of A^T and A
		ATA = np.matmul(A.T, A)
		# calculate the product of A^T and B
		ATB = np.matmul(A.T, B)
		# calculate P based on the math solution
		P = np.matmul(np.linalg.inv(ATA), ATB)
	except:
		print("matrice dimensions mismatch, continue")

def calc_sparams(P):
	# center sphere parameters
	SParams.xc = P[0]
	SParams.yc = P[1]
	SParams.zc = P[2]
	
	# radius calculation: SQRT(P[3] + Xc^2 + Yc^2 + Zc^2)
	radius = math.sqrt(P[3] + P[0]**2 + P[1]**2 + P[2]**2)
	SParams.radius = radius
	
if __name__ == '__main__':
	# ball detection node initialized
	rospy.init_node('sphere_fit', anonymous = True)
	# subscriber for point_data
	pd_sub = rospy.Subscriber("/xyz_cropped_ball", XYZarray, receive)
	# publisher fot sphere paramaters
	sp_pub = rospy.Publisher("/sphere_params", SphereParams, queue_size = 10)
	# loop rate
	rate = rospy.Rate(10)
	
	while not rospy.is_shutdown():
		# check if matrices are not empty then run model_fitting
		if len(a_matrix) > 0 and len(b_matrix) > 0:
			model_fitting(a_matrix, b_matrix)
			# check if P is not empty then run calc_sparams
			if len(P) > 0:
				calc_sparams(P)
				# publish sphere params
				sp_pub.publish(SParams)
		rate.sleep()

