import cPickle
import pickle
import gzip
import os
import sys
import time

import scipy
import scipy.ndimage

import matplotlib
matplotlib.use('TkAgg')

import numpy as np
import matplotlib.pyplot as plt
from numpy.linalg import inv

import math
import random

import networkx as nx
from sklearn.neighbors import NearestNeighbors

from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt

from mpl_toolkits.mplot3d.art3d import Poly3DCollection

from matplotlib import animation

from munkres import Munkres, print_matrix


#RANDOM PARAMETERS
PI = math.pi
spatial_boundary = 10
velocity_boundary = 1
angular_velocity_boundary = 2*PI
eps = .5

#CUBE MASS AND GEOMETRY 
#FOR COOL ALTERNATIVES check out 
#https://en.wikipedia.org/wiki/List_of_moments_of_inertia
M = 1; l = .5; w = .5; d = .5
I = (1./12)*M*np.array([[(l*l + d*d), 0., 0.],
		  [0., (w*w + d*d), 0.],
		  [0., 0., (w*w + l*l)]])

#GENERATE RANDOM STATE
def random_state():

	#position
	px = random.random()*spatial_boundary; py = random.random()*spatial_boundary
	pz = random.random()*spatial_boundary

	#quaternion
	#notation from http://www.chrobotics.com/library/understanding-quaternions
	theta = random.random()*2*PI; 
	ux = random.random(); uy = random.random(); uz = random.random()

	u_vec = np.array([ux,uy,uz])
	u_norm = np.sqrt(np.dot(u_vec,u_vec))
	ux /= u_norm; uy /= u_norm; uz /= u_norm 

	qt = math.cos(0.5*theta)
	qx = ux * math.sin(0.5*theta)
	qy = uy * math.sin(0.5*theta)
	qz = uz * math.sin(0.5*theta)

	#velocities
	vx = random.random()*velocity_boundary
	vy = random.random()*velocity_boundary
	vz = random.random()*velocity_boundary

	#angular velocities
	wx = random.random()*angular_velocity_boundary 
	wy = random.random()*angular_velocity_boundary
	wz = random.random()*angular_velocity_boundary

	return np.array([px, py, pz, qt, qx, qy, qz, 
					 vx, vy, vz, wx, wy, wz])

#GENERATE RANDOM STATE WITH A BIAS TOWARDS GOAL STATE
def random_state_bias(bias_state, gamma):

	#position
	px = np.random.normal(bias_state[0],gamma*(spatial_boundary/2.)**.5)
	py = np.random.normal(bias_state[1],gamma*(spatial_boundary/2.)**.5)
	pz = np.random.normal(bias_state[2],gamma*(spatial_boundary/2.)**.5)

	#quaternion (simple-stupid version)
	qt = np.random.normal(bias_state[3],gamma*(.2**.5))
	qx = np.random.normal(bias_state[4],gamma*(.2**.5))
	qy = np.random.normal(bias_state[5],gamma*(.2**.5))
	qz = np.random.normal(bias_state[6],gamma*(.2**.5))

	q_vec = np.array([qt,qx,qy,qz])
	q_norm = np.sqrt(np.dot(q_vec,q_vec))
	qt /= q_norm; qx /= q_norm; qy /= q_norm; qz /= q_norm

	#velocities
	vx = np.random.normal(bias_state[7],gamma*(.2**.5))
	vy = np.random.normal(bias_state[8],gamma*(.2**.5))
	vz = np.random.normal(bias_state[9],gamma*(.2**.5))

	wx = np.random.normal(bias_state[10],gamma*(PI**.5))
	wy = np.random.normal(bias_state[11],gamma*(PI**.5))
	wz = np.random.normal(bias_state[12],gamma*(PI**.5))

	return np.array([px, py, pz, qt, qx, qy, qz, 
					 vx, vy, vz, wx, wy, wz])

#DEFINE STATE METRIC
def state_metric(x, y):

	w_p = .25; w_q = .25; w_v = .025; w_w = .025
	delta = x - y

	dist = w_p*(np.dot(delta[0:3],delta[0:3]))	
#	dist = w_p*(1./spatial_boundary**2)*(np.dot(delta[0:3],delta[0:3]))
	dist += w_q*((1 - math.fabs(np.dot(x[3:7],y[3:7])))**2) 
	dist += w_v*(np.dot(delta[7:10], delta[7:10]))
	dist += w_w*(np.dot(delta[10:13], delta[10:13]))	

	return dist

#COMPUTES THE QUATERNION PRODUCT OF TWO QUATERNION VECTORS
def quaternion_product(q, r):
	t = np.zeros(4)

	# this could be simplified (= x-product - dot product)
	t[0] = r[0]*q[0] - r[1]*q[1] - r[2]*q[2] - r[3]*q[3]
	t[1] = r[0]*q[1] + r[1]*q[0] - r[2]*q[3] + r[3]*q[2]
	t[2] = r[0]*q[2] + r[1]*q[3] + r[2]*q[0] - r[3]*q[1]
	t[3] = r[0]*q[3] - r[1]*q[2] + r[2]*q[1] + r[3]*q[0]
	return t

#CONVERTS A QUATERNION VECTOR TO ITS CORRESPONDING ROTATION MATRIX
def quaternion_matrix(quaternion):

	qw = quaternion[0]; qx = quaternion[1]
	qy = quaternion[2]; qz = quaternion[3]
	return np.array([[1-2*qy*qy-2*qz*qz, 2*qx*qy-2*qz*qw, 2*qx*qz+2*qy*qw],
					 [2*qx*qy+2*qz*qw, 1-2*qx*qx-2*qz*qz, 2*qy*qz-2*qx*qw],
					 [2*qx*qz-2*qy*qw,  2*qy*qz+2*qx*qw, 1-2*qx*qx-2*qy*qy] ])

#COMPUTES THE STATE DERIVATIVE AS A FUNCTION OF THE STATE AND CONTROL
#THIS IS THE HEART OF THE DYNAMICS
def x_dot(x, u):

	forces = u[0:3]
	torques = u[3:6]
	xdot = np.zeros(13)

	#pdot = v
	xdot[0:3] = x[7:10]

	#qdot = 1/2 w_hat x q
	w_hat = np.array([0., x[10], x[11], x[12] ])
	quaternion = x[3:7] 
	quat_product = quaternion_product(w_hat, quaternion)
	xdot[3:7] = 0.5*quat_product

	#vdot = F/m
	R = quaternion_matrix(quaternion)	

	xdot[7:10] = np.dot(R,forces) / (1.*M)

	#wdot = R x I^-1 x R^T v torque
	xdot[10:13] = np.dot(R,np.dot(np.dot(R, inv(I)), np.dot(R.transpose(), torques)))
	return xdot

#APPLY CONTROL U TO STATE X OVER TIME INTERVAL DEL_T
#USES EULER'S METHOD TO FORWARD STEP IN TIME
def propagate(x, u, del_t):
	dt = .001
	n = int(del_t / dt)
	x_prop = x
	for i in range(n):	
		x_prop += dt * x_dot(x_prop, u)
	
	return x_prop	

#RUNGE KUTTA 4
def propagate_rk4(x, u, del_t):

	dt = .01
	n = int(del_t / dt)
	x_prop = x
	for i in range(n):	
	
		k1 = x_dot(x_prop, u)
		k2 = x_dot(x_prop + .5*dt*k1, u)
		k3 = x_dot(x_prop + .5*dt*k2, u)
		k4 = x_dot(x_prop + dt*k3, u)	
	
		x_prop += dt*(k1+2.*k2+2.*k3+k4)/6.
		
	return x_prop

#DESCRIPTION:
#	EXTENDS X_NEAR TOWARDS X BY SELECTING 'BEST' CONTROL
#	FOR LOOP CAN BE MADE PARALLEL
#INPUT: 
#	X IS 'RANDOM' STATE 
#	X_NEAR IS NEAREST STATE IN TREE TO RANDOM STATE
#	CONTROL IS A MATRIX WITH (NUMBER OF CONTROLS X CONTROL SIZE)
#OUTPUT:
#	BEST_STATE IS THE STATE WITH THE NEAREST 'DISTANCE' TO X
#	BEST_CONTROL IS THE CONTROL THAT GENERATES BEST_STATE
def NEW_STATE(x, x_near, u_control):

	T = .1
	U = u_control
	uN = U.shape[0]
	best_dist = float("inf")
	best_state = np.zeros(13)
	best_control = 0
	for i in range(uN):
		x_o = np.zeros(13) + x_near
		u_i = U[i,:]
		x_i = propagate_rk4(x_o, u_i, T)
	
		#COLLISION DETECTION WOULD GO HERE
	
		if state_metric(x_i, x) < best_dist:
			best_dist = state_metric(x_i, x)
			best_state = x_i
			best_control = i

	return [best_state, best_control]


#EXTENDS RRT_GRAPH AND CORRESPONDING STATES AND CONTROLS
#TOWARDS A GIVEN 'RANDOM' STATE X WITH SPECIFIED CONTROL
#RETURNS NEW_STATE, THE APPLED CONTROL, AND THE INDEX OF 
#THE NEAREST STATE 
def EXTEND(rrt_graph, rrt_states, rrt_controls, x, u_control):

	#find neaarest neighbor state
	nbrs = NearestNeighbors(n_neighbors=1, algorithm='ball_tree',
	            metric='pyfunc', func=state_metric).fit(rrt_states)
	distances, indices = nbrs.kneighbors(x)	
	index = indices[0,0]	
	
	#handle case when rrt_states is initial state array 
	if rrt_states.shape == (13,):
		x_near = np.zeros(13) + rrt_states
	else:
		x_near = np.zeros(13) + rrt_states[index]

	#find new state and new control
	new_state, new_control = NEW_STATE(x, x_near, u_control)

	return [index, new_state, new_control]

def print_state(state):

	print('P: ' + str(state[0:3]))
	print('Q: ' + str(state[3:7]))
	print('V: ' + str(state[7:10]))
	print('W: ' + str(state[10:13]))

#GENERATES PATH BETWEEN INITIAL AND GOAL STATES WITH 
#1-WAY RRT (THIS IS OBSOLETE; USE generate_BiD_path)
def generate_path(start_state, goal_state, controls):

	initial_state = start_state
	final_state = goal_state


	#initialize rrt
	rrt_graph = nx.Graph(); rrt_graph.add_node(0)

	#initialize lookup tables
	rrt_states = initial_state
	rrt_controls = [0]

	best_dist = float("inf")

	while True:

		#generate randomm state
		rand_state = random_state_bias(final_state, 1.)

		#extend tree
		index, newstate, newcontrol = EXTEND(rrt_graph, rrt_states, 
											rrt_controls, rand_state, 
											controls)
		#update lookup tables
		rrt_states = np.vstack([rrt_states, newstate])
		rrt_controls = np.column_stack([rrt_controls, newcontrol])

		#update graphs
		n_nodes = rrt_states.shape[0] - 1
		rrt_graph.add_node(n_nodes)
		rrt_graph.add_edge(index, n_nodes)

		print('State:		' + str(n_nodes))
		dist = state_metric(rrt_states[n_nodes,:], final_state)

		if dist < best_dist:
			best_dist = dist

	#	print('Dist:		' + str(dist))


		#if newstate is within epsilon of goal state, determine path and break
		if dist <= eps:
			path = nx.shortest_path(rrt_graph, 0, n_nodes)
			break

	path_states = np.zeros([len(path), 13])
	path_controls = np.zeros([len(path), len(U[0,:])])	
	rrt_controls.reshape(len(rrt_states[:,0]))

	for i in range(len(path)):

		path_states[i,:] = rrt_states[path[i],:]
		path_controls[i,:] = np.zeros(6) + controls[rrt_controls[0,i],:]

	return [rrt_states, path_states, path_controls]

#GENERATES PATH BETWEEN INITIAL AND GOAL STATES WITH 
#BIDIRECTIONAL RRT
#RETURNS THE FULL RRT STATE TABLE, THE PATH TABLE
#AND THE CONTROL TABLE FOR BOTH START AND END TREES
def generate_BiD_path(start_state, goal_state, controls):

	initial_state = start_state
	final_state = goal_state

	#initialize rrt A
	rrt_graph_A = nx.Graph(); rrt_graph_A.add_node(0)

	#initialize lookup tables A
	rrt_states_A = initial_state
	rrt_controls_A = [0]

	#initialize rrt B
	rrt_graph_B = nx.Graph(); rrt_graph_B.add_node(0)

	#initialize lookup tables B
	rrt_states_B = final_state
	rrt_controls_B = [0]

	best_dist = float("inf")

	path_A = [0]
	path_B = [0]

	count = 0
	while True:

		#generate randomm state
		rand_state = random_state()	

		if count%2 == 0:

			#extend tree A towards a random state
			index_A, newstate_A, newcontrol_A = EXTEND(rrt_graph_A, rrt_states_A, 
												rrt_controls_A, rand_state,
												controls)
			#update lookup tables A
			rrt_states_A = np.vstack([rrt_states_A, newstate_A])
			rrt_controls_A = np.column_stack([rrt_controls_A, newcontrol_A])
			#update graphs A
			n_nodes_A = rrt_states_A.shape[0] - 1
			rrt_graph_A.add_node(n_nodes_A)
			rrt_graph_A.add_edge(index_A, n_nodes_A)

			#extend tree B towards newstate A
			index_B, newstate_B, newcontrol_B = EXTEND(rrt_graph_B, rrt_states_B, 
												rrt_controls_B, newstate_A,
												-controls)
										
			#update lookup tables B
			rrt_states_B = np.vstack([rrt_states_B, newstate_B])
			rrt_controls_B = np.column_stack([rrt_controls_B, newcontrol_B])		
		
			#update graphs B
			n_nodes_B = rrt_states_B.shape[0] - 1
			rrt_graph_B.add_node(n_nodes_B)
			rrt_graph_B.add_edge(index_B, n_nodes_B)
					
			#if newstate is within epsilon of goal state, determine path and break
			dist = state_metric(newstate_A, newstate_B)
			if dist <= eps:
				path_A = nx.shortest_path(rrt_graph_A, 0, n_nodes_A)
				path_B = nx.shortest_path(rrt_graph_B, 0, n_nodes_B)
				break
		
		else:
			#extend tree B towards random state
			index_B, newstate_B, newcontrol_B = EXTEND(rrt_graph_B, rrt_states_B, 
												rrt_controls_B, rand_state,
												-controls)
										
			#update lookup tables B
			rrt_states_B = np.vstack([rrt_states_B, newstate_B])
			rrt_controls_B = np.column_stack([rrt_controls_B, newcontrol_B])		
		
			#update graphs B
			n_nodes_B = rrt_states_B.shape[0] - 1
			rrt_graph_B.add_node(n_nodes_B)
			rrt_graph_B.add_edge(index_B, n_nodes_B)
		
			#extend tree A towards newstateB
			index_A, newstate_A, newcontrol_A = EXTEND(rrt_graph_A, rrt_states_A, 
												rrt_controls_A, newstate_B,
												controls)
			#update lookup tables A
			rrt_states_A = np.vstack([rrt_states_A, newstate_A])
			rrt_controls_A = np.column_stack([rrt_controls_A, newcontrol_A])
			#update graphs A
			n_nodes_A = rrt_states_A.shape[0] - 1
			rrt_graph_A.add_node(n_nodes_A)
			rrt_graph_A.add_edge(index_A, n_nodes_A)
		
			#if newstate is within epsilon of goal state, determine path and break
			dist = state_metric(newstate_A, newstate_B)
			if dist <= eps:
				path_A = nx.shortest_path(rrt_graph_A, 0, n_nodes_A)
				path_B = nx.shortest_path(rrt_graph_B, 0, n_nodes_B)
				break
	
		count += 1

		if dist < best_dist:
			best_dist = dist

		if (count%50) == 0:
			print('	NODE:	' + str(count))
			print('	BEST DISTANCE:		' + str(best_dist))
#			print('DIST:		' + str(dist))

	path_states_A = np.zeros([len(path_A), 13])
	path_controls_A = np.zeros([len(path_A), len(controls[0,:])])	
	rrt_controls_A.reshape(len(rrt_states_A[:,0]))

	for i in range(len(path_A)):

		path_states_A[i,:] = rrt_states_A[path_A[i],:]
		path_controls_A[i,:] = np.zeros(6) + controls[rrt_controls_A[0,i],:]
	
	path_states_B = np.zeros([len(path_B), 13])
	path_controls_B = np.zeros([len(path_B), len(controls[0,:])])	
	rrt_controls_B.reshape(len(rrt_states_B[:,0]))

	for i in range(len(path_B)):

		path_states_B[i,:] = rrt_states_B[path_B[i],:]
		path_controls_B[i,:] = np.zeros(6) + controls[rrt_controls_B[0,i],:]

	return [rrt_states_A, path_states_A, path_controls_A,
			rrt_states_B, path_states_B, path_controls_B]


#sphere based collision checking
def collision(state_a, state_b):
	
	radius = math.sqrt((l/2.)**2 + (w/2.)**2 + (d/2.)**2)
	xa = state_a[0]; ya = state_a[1]; za = state_a[2];
	xb = state_b[0]; yb = state_b[1]; zb = state_b[2];
	
	if (xa-xb)**2. + (ya-yb)**2. + (za-zb)**2. > 4.*radius**2.:
		return False
	else:
		return True

def valid_state(state, siblings):
		n_sibilings = len(siblings[:,0])
		
		for i in range(n_sibilings):
		
			if collision(state,siblings[i]):
				return False
				
		return True

#assumes path and sibling paths are of same lengths
def collision_paths(path, sibling_paths):
	
	path_length = len(path[:,0])
	
	for i in range(path_length):
		if valid_state(path[i,:], sibling_paths[:,i,:]) == False:
			return True	
	
	return False	

#assumes matching order has been determined
def fleet_sequential_avoidance(initial_fleet, final_fleet, control):

	n_fleet = len(initial_fleet[:,0])

	num_siblings = 1

	print("Sat path " + str(0))
	rrt_a, path_a, u_a, rrt_b, path_b, u_b = \
	generate_BiD_path(initial_fleet[0,:], final_fleet[0,:], control)
	b_flip = np.fliplr(path_b.transpose()).transpose()
	sat_path = np.vstack([path_a, b_flip])

	sibling_paths = np.zeros([num_siblings, len(sat_path[:,0]), 13])
	sibling_paths[0,:,:] = sat_path
	max_sat_path = len(sat_path[:,0])

	print('	First Sat path shape:	' + str(np.shape(sibling_paths)))		

	i = 1
	while i < n_fleet:
		
		print("Sat path " + str(i))
		
		rrt_a, path_a, u_a, rrt_b, path_b, u_b = \
		generate_BiD_path(initial_fleet[i,:], final_fleet[i,:], control)
		b_flip = np.fliplr(path_b.transpose()).transpose()
		sat_path = np.vstack([path_a, b_flip])

		sat_length = len(sat_path[:,0])

		if sat_length <= max_sat_path:
			
			#truncate sibling paths
			sat_siblings =  sibling_paths[:,0:sat_length,:]
			
		else:
			
			#extend sibling paths
			sat_siblings = np.zeros([num_siblings, sat_length, 13])
			
			sat_siblings[: ,0:max_sat_path, :] = sibling_paths
#CAUTION: index error possible
			for j in range(max_sat_path,sat_length):
				sat_siblings[:,j,:] = sat_siblings[:,max_sat_path-1,:]
			
		if collision_paths(sat_path, sat_siblings):
			print("		BAD PATH, RECOMPUTING TRAJECTORY...")
			continue
		
		print('	New Sibling shape:	' + str(np.shape(sat_path)))	
		print('	Siblings Path shape: 	' + str(np.shape(sibling_paths)))	
			
		i += 1
		num_siblings += 1
		if sat_length <= max_sat_path:
			
			#extend sat_path
			sat_path_extend = np.zeros([max_sat_path,13])
			sat_path_extend[0:sat_length] = sat_path
			
			for j in range(sat_length,max_sat_path):
				sat_path_extend[j,:] = sat_path_extend[sat_length-1,:]
			
			#update sibling path	
			new_sibling_paths = np.zeros([num_siblings, max_sat_path, 13])
			new_sibling_paths[0:num_siblings-1, :, :] = sibling_paths
			new_sibling_paths[num_siblings-1,:,:] = sat_path_extend
			
			sibling_paths = new_sibling_paths
			
		else:
			
			#extend sibling paths
			sibling_paths_extend = np.zeros([num_siblings-1, sat_length, 13])
			sibling_paths_extend[:,0:max_sat_path,:] = sibling_paths
			for j in range(max_sat_path,sat_length):
				sibling_paths_extend[:,j,:] = \
				sibling_paths_extend[:,max_sat_path-1,:]
			
			#update sibling path AND max_sat_path	
			new_sibling_paths = np.zeros([num_siblings, sat_length, 13])
			new_sibling_paths[0:num_siblings-1, :, :] = sibling_paths_extend
			new_sibling_paths[num_siblings-1,:,:] = sat_path
			max_sat_path = sat_length
			
			
			sibling_paths = new_sibling_paths
			
	fleet_path = sibling_paths
		
	return fleet_path				
	
#RaBiD RRT: recursive asymmetric BiD RRT
def RaBiD_RRT(start_state, goal_state, sibling_path, controls):

	n_siblings = len(sibling_path[:,0,0])
	sibling_length = len(sibling_path[0,:,0])
	
	sibling_final = sibling_path[:,sibling_length-1,:]
	
	initial_state = start_state
	final_state = goal_state

	#initialize rrt A
	rrt_graph_A = nx.Graph(); rrt_graph_A.add_node(0)

	#initialize lookup tables A
	rrt_states_A = initial_state
	rrt_controls_A = [0]

	#initialize rrt B
	rrt_graph_B = nx.Graph(); rrt_graph_B.add_node(0)

	#initialize lookup tables B
	rrt_states_B = final_state
	rrt_controls_B = [0]

	best_dist = float("inf")

	path_A = [0]
	path_B = [0]

	count = 0
	while True:

		#generate randomm state
		rand_state = random_state()	

		if count%2 == 0:

			#extend tree A towards a random state
			index_A, newstate_A, newcontrol_A = EXTEND(rrt_graph_A, rrt_states_A, 
												rrt_controls_A, rand_state,
												controls)
			
			depth = nx.shortest_path_length(rrt_graph_A, 0, index_A)
			
#caution: indexing error possible			
			if depth + 1 <= sibling_length:
				#truncate siblings
				sibling_states = sibling_path[:,depth,:]
			else:
				sibling_states = sibling_path[:,sibling_length-1,:]

			#if there is a collision skip
			if valid_state(newstate_A, sibling_states) == False:
				continue
			
			#update lookup tables A
			rrt_states_A = np.vstack([rrt_states_A, newstate_A])
			rrt_controls_A = np.column_stack([rrt_controls_A, newcontrol_A])
			#update graphs A
			n_nodes_A = rrt_states_A.shape[0] - 1
			rrt_graph_A.add_node(n_nodes_A)
			rrt_graph_A.add_edge(index_A, n_nodes_A)

			#extend tree B towards newstate A
			index_B, newstate_B, newcontrol_B = EXTEND(rrt_graph_B, rrt_states_B, 
												rrt_controls_B, newstate_A,
												-controls)
										
			#update lookup tables B
			rrt_states_B = np.vstack([rrt_states_B, newstate_B])
			rrt_controls_B = np.column_stack([rrt_controls_B, newcontrol_B])		
		
			#update graphs B
			n_nodes_B = rrt_states_B.shape[0] - 1
			rrt_graph_B.add_node(n_nodes_B)
			rrt_graph_B.add_edge(index_B, n_nodes_B)
					
			#if newstate is within epsilon of goal state, determine path and break
			dist = state_metric(newstate_A, newstate_B)
			if dist <= eps:
				path_A = nx.shortest_path(rrt_graph_A, 0, n_nodes_A)
				path_B = nx.shortest_path(rrt_graph_B, 0, n_nodes_B)

				path_states_A = np.zeros([len(path_A), 13])
				for i in range(len(path_A)):
					path_states_A[i,:] = rrt_states_A[path_A[i],:]

				path_states_B = np.zeros([len(path_B), 13])
				for i in range(len(path_B)):
					path_states_B[i,:] = rrt_states_B[path_B[i],:]

				a_states = path_states_A
				b_states = np.fliplr(path_states_B.transpose()).transpose()				

				a_length = len(path_states_A); b_length = len(b_states)

				sibling_path_2 = np.zeros([n_siblings, len(b_states), 13])

	#CAUTION: potential indexing error
				if sibling_length <= a_length:

					for i in range(b_length):
						sibling_path_2[:,i,:] = sibling_final

				elif sibling_length > a_length and sibling_length <= \
					 (a_length + b_length):

					for i in range(sibling_length-a_length):
						sibling_path_2[:,i,:] = sibling_path[:,i+a_length,:]

					for i in range(a_length+b_length-sibling_length):
						sibling_path_2[:,i,:] = sibling_final

				else:

					for i in range(b_length):
						sibling_path_2[:,i,:] = sibling_path[:,i+a_length,:]

				#beauitful recursive part
				if collision_paths(b_states, sibling_path_2) == True:

					print('		Collision Found in 2nd half')
					print('		Recursive BiD search...')
					b_states = RaBiD_RRT(b_states[0,:], b_states[b_length-1],\
										 sibling_path_2, controls)

				else:

					#b_states is good to go
					pass

				path = np.vstack((a_states, b_states))

				return path
		
		else:	
			#extend tree B towards random state
			index_B, newstate_B, newcontrol_B = EXTEND(rrt_graph_B, rrt_states_B, 
												rrt_controls_B, rand_state,
												-controls)
										
			#update lookup tables B
			rrt_states_B = np.vstack([rrt_states_B, newstate_B])
			rrt_controls_B = np.column_stack([rrt_controls_B, newcontrol_B])		
		
			#update graphs B
			n_nodes_B = rrt_states_B.shape[0] - 1
			rrt_graph_B.add_node(n_nodes_B)
			rrt_graph_B.add_edge(index_B, n_nodes_B)
		
			#extend tree A towards newstateB
			index_A, newstate_A, newcontrol_A = EXTEND(rrt_graph_A, rrt_states_A, 
												rrt_controls_A, newstate_B,
												controls)
												
			
			depth = nx.shortest_path_length(rrt_graph_A, 0, index_A)
			
#caution: indexing error possible			
			if depth + 1 <= sibling_length:
				sibling_states = sibling_path[:,depth,:]				
			else:
				sibling_states = sibling_path[:,sibling_length-1,:]
	
	
			#if there is a collision skip
			if valid_state(newstate_A, sibling_states) == False:
				continue
																
			#update lookup tables A
			rrt_states_A = np.vstack([rrt_states_A, newstate_A])
			rrt_controls_A = np.column_stack([rrt_controls_A, newcontrol_A])
			#update graphs A
			n_nodes_A = rrt_states_A.shape[0] - 1
			rrt_graph_A.add_node(n_nodes_A)
			rrt_graph_A.add_edge(index_A, n_nodes_A)
		
			#if newstate is within epsilon of goal state, determine path and break
			dist = state_metric(newstate_A, newstate_B)
			if dist <= eps:
				path_A = nx.shortest_path(rrt_graph_A, 0, n_nodes_A)
				path_B = nx.shortest_path(rrt_graph_B, 0, n_nodes_B)
				
				path_states_A = np.zeros([len(path_A), 13])
				for i in range(len(path_A)):
					path_states_A[i,:] = rrt_states_A[path_A[i],:]
				
				path_states_B = np.zeros([len(path_B), 13])
				for i in range(len(path_B)):
					path_states_B[i,:] = rrt_states_B[path_B[i],:]
				
				a_states = path_states_A
				b_states = np.fliplr(path_states_B.transpose()).transpose()				
				
				a_length = len(path_states_A); b_length = len(b_states)

				sibling_path_2 = np.zeros([n_siblings, len(b_states), 13])

#CAUTION: potential indexing error
				if sibling_length <= a_length:
						
					for i in range(b_length):
						sibling_path_2[:,i,:] = sibling_final
						
				elif sibling_length > a_length and sibling_length <= \
					 (a_length + b_length):
					
					for i in range(sibling_length-a_length):
						sibling_path_2[:,i,:] = sibling_path[:,i+a_length,:]
						
					for i in range(a_length+b_length-sibling_length):
						sibling_path_2[:,i,:] = sibling_final
											
				else:
				
					for i in range(b_length):
						sibling_path_2[:,i,:] = sibling_path[:,i+a_length,:]
				
				#beauitful recursive part
				if collision_paths(b_states, sibling_path_2) == True:
					
					print('		Collision Found in 2nd half')
					print('		Recursive BiD search...')
					b_states = RaBiD_RRT(b_states[0,:], b_states[b_length-1],\
										 sibling_path_2, controls)
					
				else:
					
					#b_states is good to go
					pass
				
				path = np.vstack((a_states, b_states))
				
				return path
								
		count += 1

		if dist < best_dist:
			best_dist = dist

		if (count%50) == 0:
			print('	NODES:	' + str(count))			
			print('	BEST DISTANCE:		' + str(best_dist))


#assumes matching order has been determined
def fleet_RaBiD(initial_fleet, final_fleet, control):

	n_fleet = len(initial_fleet[:,0])

	num_siblings = 1

	print("Sat path " + str(0))
	rrt_a, path_a, u_a, rrt_b, path_b, u_b = \
	generate_BiD_path(initial_fleet[0,:], final_fleet[0,:], control)
	b_flip = np.fliplr(path_b.transpose()).transpose()
	sat_path = np.vstack([path_a, b_flip])

	sibling_paths = np.zeros([num_siblings, len(sat_path[:,0]), 13])
	sibling_paths[0,:,:] = sat_path
	max_sat_path = len(sat_path[:,0])

	print('	First Sat path shape:	' + str(np.shape(sibling_paths)))		

	i = 1
	while i < n_fleet:

		print("Sat path " + str(i))

#		rrt_a, path_a, u_a, rrt_b, path_b, u_b = \
#		generate_BiD_path(initial_fleet[i,:], final_fleet[i,:], control)
#		b_flip = np.fliplr(path_b.transpose()).transpose()
#		sat_path = np.vstack([path_a, b_flip])

		sat_path = RaBiD_RRT(initial_fleet[i,:], final_fleet[i,:],\
		 					 sibling_paths, control)

		sat_length = len(sat_path[:,0])

#		if sat_length <= max_sat_path:

			#truncate sibling paths
#			sat_siblings =  sibling_paths[:,0:sat_length,:]

#		else:

			#extend sibling paths
#			sat_siblings = np.zeros([num_siblings, sat_length, 13])

#			sat_siblings[: ,0:max_sat_path, :] = sibling_paths
#CAUTION: index error possible
#			for j in range(max_sat_path,sat_length):
#				sat_siblings[:,j,:] = sat_siblings[:,max_sat_path-1,:]

#		if collision_paths(sat_path, sat_siblings):
#			print("		BAD PATH, RECOMPUTING TRAJECTORY...")
#			continue

		print('	New Sibling shape:	' + str(np.shape(sat_path)))	
		print('	Siblings Path shape: 	' + str(np.shape(sibling_paths)))	

		i += 1
		num_siblings += 1
		if sat_length <= max_sat_path:

			#extend sat_path
			sat_path_extend = np.zeros([max_sat_path,13])
			sat_path_extend[0:sat_length] = sat_path

			for j in range(sat_length,max_sat_path):
				sat_path_extend[j,:] = sat_path_extend[sat_length-1,:]

			#update sibling path	
			new_sibling_paths = np.zeros([num_siblings, max_sat_path, 13])
			new_sibling_paths[0:num_siblings-1, :, :] = sibling_paths
			new_sibling_paths[num_siblings-1,:,:] = sat_path_extend

			sibling_paths = new_sibling_paths

		else:

			#extend sibling paths
			sibling_paths_extend = np.zeros([num_siblings-1, sat_length, 13])
			sibling_paths_extend[:,0:max_sat_path,:] = sibling_paths
			for j in range(max_sat_path,sat_length):
				sibling_paths_extend[:,j,:] = \
				sibling_paths_extend[:,max_sat_path-1,:]

			#update sibling path AND max_sat_path	
			new_sibling_paths = np.zeros([num_siblings, sat_length, 13])
			new_sibling_paths[0:num_siblings-1, :, :] = sibling_paths_extend
			new_sibling_paths[num_siblings-1,:,:] = sat_path
			max_sat_path = sat_length


			sibling_paths = new_sibling_paths

	fleet_path = sibling_paths

	return fleet_path

#DESCRIPTION:
#	SIMPLE FLEET PLANNER. APPLIES A BI-PARTITE MATCHING ALGORITHM
#	BASED ON METRIC DISTANCES
#INPUT: 
#	INITIAL_FLEET IS A MATRIX (FLEET SIZE X STATE SIZE)
#	FINAL_FLEET IS DITTO
#	CONTROL IS A MATRIX WITH (NUMBER OF CONTROLS X CONTROL SIZE)
#OUTPUT:
#	FLEET PATHS IS A 3D ARRAY (FLEET SIZE X PATH LENGTH X STATE SIZE)
def fleet_simple(initial_fleet, final_fleet, control):

	#size of fleet
	n_fleet = len(initial_fleet[:,0])

	#COMPUTE COST MATRIX FOR EACH MATCH
	cost_matrix = np.zeros([n_fleet, n_fleet])

	for i in range(n_fleet):
		for j in range(n_fleet): 
			cost_matrix[i,j] = state_metric(initial_fleet[i,:],
											final_fleet[j,:])

	#COMPUTE BIPARTITE MATCHING ASSIGNMENTS (HUNGARIAN ~O(N^3), N: n_fleet)
	target_fleet = np.zeros([n_fleet,13])
	m = Munkres()
	indices = m.compute(cost_matrix)
	assignments = np.zeros(n_fleet)
	for i in range(n_fleet):
		assignments[i] = indices[i][1]	
		target_fleet[i,:] = final_fleet[assignments[i],:]	
#		np.array([indices[i][1], indices[i][1], indices[2][1]])  
	

	#COMPUTE PATHS
#	fleet_paths = fleet_sequential_avoidance(initial_fleet, target_fleet,control)
	fleet_paths = fleet_RaBiD(initial_fleet, target_fleet, control)

	'''
	#COMPUTE PATHS
	MAX_PATH = 10000
	overshoot = np.zeros([n_fleet, MAX_PATH, 13])
	path_lengths = np.zeros(n_fleet)

	for i in range(n_fleet):

		print("Sat path " + str(i))

		#compute path
		target_i = assignments[i]
		rrt_a, path_a, u_a, rrt_b, path_b, u_b = \
		generate_BiD_path(initial_fleet[i,:], final_fleet[target_i,:], control)
		b_flip = np.fliplr(path_b.transpose()).transpose()
		sat_path = np.vstack([path_a, b_flip])
		sat_length = len(sat_path[:,0])
	
		#populate overshoot 3D array
		overshoot[i,0:sat_length, :] = sat_path
	
		#record length of path
		path_lengths[i] = sat_length

	#copy path data into fleet_path 3D array & buffer 	
	max_path_length = np.amax(path_lengths)

	fleet_paths = np.zeros([n_fleet, max_path_length, 13])

	for i in range(n_fleet):
	
		#copy paths into fleet_paths
		sat_length = path_lengths[i]
	
		#note: the effect of this line could be reached outside the loop
		fleet_paths[i,0:sat_length,:] = overshoot[i,0:sat_length, :]
	
		final_state = fleet_paths[i,sat_length-1,:]
	
		#buffer remaining space with final state 
		if sat_length != max_path_length:
			#print('maxpathlength: '+str(max_path_length))
			#print('sat_length: '+str(sat_length))
			for j in range(int(sat_length),int(max_path_length)):			
				fleet_paths[i,j,:] = final_state
	''' 
	
	
	return fleet_paths
	

#RETURNS A GRAPHICAL RENDERING OF A STATE
def state_to_cube(state, ax):

	quat = state[3:7]
	pos = state[0:3]

	corners = np.array([[w/2, 	l/2, 	d/2],
						[w/2, 	l/2, 	-d/2],
						[w/2, 	-l/2, 	d/2],
						[w/2, 	-l/2, 	-d/2],
						[-w/2, 	l/2, 	d/2],
						[-w/2, 	l/2, 	-d/2],
						[-w/2, 	-l/2, 	d/2],
						[-w/2, 	-l/2, 	-d/2]])

	R = quaternion_matrix(quat)

	rotated_corners = np.dot(R, corners.transpose())

	cube_corners = rotated_corners + np.array([pos,]*8).transpose()

#	ax = Axes3D(fig)
	ax.set_xlim([0, 10])
	ax.set_ylim([0, 10])
	ax.set_zlim([0, 10])
	ax.set_xlabel('X Label')
	ax.set_ylabel('Y Label')
	ax.set_zlabel('Z Label')

	#FACE +x
	x_corners = np.array([cube_corners[0,0],cube_corners[0,2],
	 					  cube_corners[0,3],cube_corners[0,1]])
	y_corners = np.array([cube_corners[1,0],cube_corners[1,2],
	 					  cube_corners[1,3],cube_corners[1,1]])
	z_corners = np.array([cube_corners[2,0],cube_corners[2,2],
	 					  cube_corners[2,3],cube_corners[2,1]])
	verts = [zip(x_corners, y_corners, z_corners)]
	ax.add_collection3d(Poly3DCollection(verts))

	#FACE -x
	x_corners = np.array([cube_corners[0,4],cube_corners[0,5],
	 					  cube_corners[0,7],cube_corners[0,6]])
	y_corners = np.array([cube_corners[1,4],cube_corners[1,5],
	 					  cube_corners[1,7],cube_corners[1,6]])
	z_corners = np.array([cube_corners[2,4],cube_corners[2,5],
	 					  cube_corners[2,7],cube_corners[2,6]])
	verts = [zip(x_corners, y_corners, z_corners)]
	ax.add_collection3d(Poly3DCollection(verts))

	#FACE +y
	x_corners = np.array([cube_corners[0,0],cube_corners[0,1],
	 					  cube_corners[0,5],cube_corners[0,4]])
	y_corners = np.array([cube_corners[1,0],cube_corners[1,1],
	 					  cube_corners[1,5],cube_corners[1,4]])
	z_corners = np.array([cube_corners[2,0],cube_corners[2,1],
	 					  cube_corners[2,5],cube_corners[2,4]])
	verts = [zip(x_corners, y_corners, z_corners)]
	ax.add_collection3d(Poly3DCollection(verts))

	#FACE -y
	x_corners = np.array([cube_corners[0,2],cube_corners[0,3],
	 					  cube_corners[0,7],cube_corners[0,6]])
	y_corners = np.array([cube_corners[1,2],cube_corners[1,3],
	 					  cube_corners[1,7],cube_corners[1,6]])
	z_corners = np.array([cube_corners[2,2],cube_corners[2,3],
	 					  cube_corners[2,7],cube_corners[2,6]])
	verts = [zip(x_corners, y_corners, z_corners)]
	ax.add_collection3d(Poly3DCollection(verts))

	#FACE +z
	x_corners = np.array([cube_corners[0,0],cube_corners[0,4],
	 					  cube_corners[0,6],cube_corners[0,2]])	
	y_corners = np.array([cube_corners[1,0],cube_corners[1,4],
	 					  cube_corners[1,6],cube_corners[1,2]])
	z_corners = np.array([cube_corners[2,0],cube_corners[2,4],
	 					  cube_corners[2,6],cube_corners[2,2]])
	verts = [zip(x_corners, y_corners, z_corners)]
	ax.add_collection3d(Poly3DCollection(verts))

	#FACE +z
	x_corners = np.array([cube_corners[0,1],cube_corners[0,5],
	 					  cube_corners[0,7],cube_corners[0,3]])
	y_corners = np.array([cube_corners[1,1],cube_corners[1,5],
	 					  cube_corners[1,7],cube_corners[1,3]])
	z_corners = np.array([cube_corners[2,1],cube_corners[2,5],
	 					  cube_corners[2,7],cube_corners[2,3]])
	verts = [zip(x_corners, y_corners, z_corners)]
	ax.add_collection3d(Poly3DCollection(verts))

	return ax

#def state_to_fleet(fleet_state):


#RETURNS A 3D PLOT OF A BIDIRECTIONAL RRT 
def plot_rrt(rrt_a, path_a, rrt_b, path_b):

	xs_A = path_a[:,0]
	ys_A = path_a[:,1]
	zs_A = path_a[:,2]

	xs_B = path_b[:,0]
	ys_B = path_b[:,1]
	zs_B = path_b[:,2]

	x_rrt_A = rrt_a[:,0]
	y_rrt_A = rrt_a[:,1]
	z_rrt_A = rrt_a[:,2]

	x_rrt_B = rrt_b[:,0]
	y_rrt_B = rrt_b[:,1]
	z_rrt_B = rrt_b[:,2]

	fig_rrt = plt.figure()
	ax_rrt = fig_rrt.add_subplot(111, projection='3d')

	ax_rrt.scatter(xs_A, ys_A, zs_A, c='b', marker='o')
	ax_rrt.scatter(xs_B, ys_B, zs_B, c='r', marker='o')
	ax_rrt.scatter(x_rrt_A, y_rrt_A, z_rrt_A, c='b', marker='_')
	ax_rrt.scatter(x_rrt_B, y_rrt_B, z_rrt_B, c='r', marker='_')
	ax_rrt.set_xlabel('X Label')
	ax_rrt.set_ylabel('Y Label')
	ax_rrt.set_zlabel('Z Label')

	plt.show()
			
def state_to_fleet(fleet_state, fig):

	n_fleet = len(fleet_state[:,0])

	ax = Axes3D(fig)

	for i in range(n_fleet):

		sat = fleet_state[i,:]

		state_to_cube(sat, ax)

	return ax

#####################
#ANIMATION
#####################
def animate_fleet(fleet_paths, animation_file):

	animation_path = '../../../Animations/'
	n_frames = len(fleet_paths[0,:,0]) - 1

	fig = plt.figure()
	ax = Axes3D(fig)
	ax.set_xlim([0, 10])
	ax.set_ylim([0, 10])
	ax.set_zlim([0, 10])

	def init():	
		return ax,

	def animate(i):
		ax_fleet = state_to_fleet(fleet_paths[:,i,:], fig)
		print('		Progress:	' + str('%.2f' % (100.*i/n_frames)) + '%')
		return ax_fleet

	# call the animator.  blit=True means only re-draw the parts that have changed.
	anim = animation.FuncAnimation(fig, animate, init_func=init,
	                               frames=n_frames, interval=100, blit=True)

	print('SAVING ANIMATION...')
	anim.save(animation_path + animation_file)
				
#PLOTS 3D BIDIRECTIONAL PATHS FOR FLEET				
def plot_fleet(fleet_paths):

	n_fleet = len(fleet_paths[:,0,0])

	fig_fleet = plt.figure()
	ax_fleet = fig_fleet.add_subplot(111, projection='3d')

	for i in range(n_fleet):
	
		path_i  = fleet_paths[i,:,:]
	 
		xi = path_i[:,0]
		yi = path_i[:,1]
		zi = path_i[:,2]
	
		ax_fleet.scatter(xi, yi, zi, c='b', marker='o')

	ax_fleet.set_xlabel('X Label')
	ax_fleet.set_ylabel('Y Label')
	ax_fleet.set_zlabel('Z Label')

	plt.show()




print('hello world!')

