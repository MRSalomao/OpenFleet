#!/usr/bin/env python
import numpy as np
import networkx as nx
from sklearn.neighbors import NearestNeighbors
import random
import math
import Dynamics


# Define PI as a global variable
PI = math.pi

def generate_BiD_path(start_state, goal_state, controls, N, obs):
  '''
      Description:
      This function generates path between initial and goal states with bidirectional rrt. If it sucesed, it returns the full rrt state table, the path table and the control table for both start and end trees.

      Input:
        (*) start_state
        (*) goal_state
        (*) controls = 3D controls
        (*) N =  Number of solution trajectories
        (*) obs = List of bounding boxes
        
      Output:
        (*) A list with:
        [rrt_states_i, path_states_i, path_controls_i,
         rrt_states_g, path_states_g, path_controls_g]
  '''
  
  eps = .5
  initial_state = start_state
  final_state = goal_state

  # Initialize RRT from Initial State
  rrt_graph_i = nx.Graph(); 
  rrt_graph_i.add_node(0)

  # Initialize lookup tables
  rrt_states_i = initial_state
  rrt_controls_i = [0]

  # Initialize RRT from Goal State
  rrt_graph_g = nx.Graph(); 
  rrt_graph_g.add_node(0)

  # Initialize lookup tables
  rrt_states_g = final_state
  rrt_controls_g = [0]

  best_dist = float("inf")

  path_i = [0]
  path_g = [0]
  
  # Create a list with both RRT
  RRT = [ ['i', rrt_graph_i, rrt_states_i, rrt_controls_i, path_i] , 
          ['g', rrt_graph_g, rrt_states_g, rrt_controls_g, path_g] ]
  
  count = 0
  
  # T is the trajectories found
  T = 0

  while T < N:
    # Generate random state
    rand_state = random_state(obs)  

    # Extend tree A towards a random state
    # Keep in mind that A is not always the tree that contains the initial state 
    index_A, newstate_A, newcontrol_A = EXTEND(RRT[0][1], RRT[0][2], RRT[0][3], rand_state, controls)
    
    # Update lookup tables A
    RRT[0][2] = np.vstack([RRT[0][2], newstate_A])
    RRT[0][3] = np.column_stack([RRT[0][3], newcontrol_A])
            
    # Update graphs A
    n_nodes_A = RRT[0][2].shape[0] - 1
    RRT[0][1].add_node(n_nodes_A)
    RRT[0][1].add_edge(index_A, n_nodes_A)

    # Extend tree B towards newstate A
    index_B, newstate_B, newcontrol_B = EXTEND(RRT[1][1], RRT[1][2], RRT[1][3], newstate_A, -controls)
                    
    # Update lookup tables B
    RRT[1][2] = np.vstack([RRT[1][2], newstate_B])
    RRT[1][3] = np.column_stack([RRT[1][3], newcontrol_B])    
    
    # Update graphs B
    n_nodes_B = RRT[1][2].shape[0] - 1
    RRT[1][1].add_node(n_nodes_B)
    RRT[1][1].add_edge(index_B, n_nodes_B)
          
    # If newstate is within epsilon of goal state, determine path and add the variable N.
    # N means the number of solution trajectories found
    dist = Dynamics.state_metric(newstate_A, newstate_B)
    count += 1
    
    if dist < best_dist:
      best_dist = dist

    if (count%50) == 0:
      print('  NODE:  ' + str(count) + '  N:  ' +str(T))
      print('  BEST DISTANCE:    ' + str(best_dist))
      
    if dist <= eps:
      RRT[0][4] = nx.shortest_path(RRT[0][1], 0, n_nodes_A)
      RRT[1][4] = nx.shortest_path(RRT[1][1], 0, n_nodes_B)
      T += 1
    
    else:
      # Swap the Trees
      RRT = [ RRT[1], RRT[0] ]
  
  # Break the list RRT into the trees I and G
  if (RRT[0][0] == 'i'):
    I = RRT[0]
    G = RRT[1]
  else:
    I = RRT[1]
    G = RRT[0]
  
  '''
  Reminder:
    x[0] = Letter
    x[1] = rrt_graph
    x[2] = rrt_states
    x[3] = rrt_controls
    x[4] = path
  '''
  path_states_i = np.zeros([len(I[4]), 13])
  path_controls_i = np.zeros([len(I[4]), len(controls[0,:])])  
  I[3].reshape(len(I[2][:,0]))

  for j in range(len(I[4])):
    path_states_i[j,:] = I[2][I[4][j],:]
    path_controls_i[j,:] = np.zeros(6) + controls[I[3][0,j],:]
  
  path_states_g = np.zeros([len(G[4]), 13])
  path_controls_g = np.zeros([len(G[4]), len(controls[0,:])])  
  G[3].reshape(len(G[2][:,0]))

  for j in range(len(G[4])):
    path_states_g[j,:] = G[2][G[4][j],:]
    path_controls_g[j,:] = np.zeros(6) + controls[G[3][0,j],:]

  return [I[2], path_states_i, path_controls_i,
          G[2], path_states_g, path_controls_g]

def random_state(obs):
  '''
      Description:
      It generates a random state

      Input:
        (*) obs = bounding boxes list
        
      Output:
        (*) Random State = [px, py, pz, qt, qx, qy, qz, vx, vy, vz, wx, wy, wz]
  '''
  
  # Random Parameters
  spatial_boundary = 12
  velocity_boundary = 1
  angular_velocity_boundary = 2*PI

  # Generate a random Position and performs a collision check for ISS
  px = 0
  py = 0
  pz = 0
  
  v = True
  
  while v:
    px = random.random()*spatial_boundary
    py = random.random()*spatial_boundary
    pz = random.random()*spatial_boundary
    
    for box in obs:
      if (box.CollisionCheck([px,py,pz])):
        print "  Collision: Random State - ISS"
        break
      
    v = False
  
  # Quaternion
  # notation from http://www.chrobotics.com/library/understanding-quaternions
  theta = random.random()*2*PI
  ux = random.random()
  uy = random.random()
  uz = random.random()

  u_vec = np.array([ux,uy,uz])
  u_norm = np.sqrt(np.dot(u_vec,u_vec))
  
  ux /= u_norm
  uy /= u_norm
  uz /= u_norm 

  qt = math.cos(0.5*theta)
  qx = ux * math.sin(0.5*theta)
  qy = uy * math.sin(0.5*theta)
  qz = uz * math.sin(0.5*theta)

  # Velocities
  vx = random.random()*velocity_boundary
  vy = random.random()*velocity_boundary
  vz = random.random()*velocity_boundary

  # Angular velocities
  wx = random.random()*angular_velocity_boundary 
  wy = random.random()*angular_velocity_boundary
  wz = random.random()*angular_velocity_boundary

  return np.array([px, py, pz, qt, qx, qy, qz, 
                   vx, vy, vz, wx, wy, wz])

def propagate(x, u, del_t):
  '''
      Description:
      Apply control u to state x over time interval del_t. It uses euler's method to forward step in time.

      Input:
        (*) x is the nearest state in tree to random state
        (*) u is the control matrix
        (*) delt_t
        
      Output:
        (*) x_prop
  '''
  dt = .001
  n = int(del_t / dt)
  x_prop = x
  for i in range(n):  
    x_prop += dt * Dynamics.x_dot(x_prop, u)
  
  return x_prop  

def NEW_STATE(x, x_near, u_control):
  '''
      Description:
        Extends x_near towards x by selecting 'best' control.

      Input:
        (*) x is 'random' state 
        (*) x_near is nearest state in tree to random state
        (*) control is a matrix with (number of controls x control size)
        
      Output:
        (*) best_state is the state with the nearest 'distance' to x
        (*) best_control is the control that generates best_state
  '''

  T = .1
  U = u_control
  uN = U.shape[0]
  best_dist = float("inf")
  best_state = np.zeros(13)
  best_control = 0
  for i in range(uN):
    x_o = np.zeros(13) + x_near
    u_i = U[i,:]
    x_i = propagate(x_o, u_i, T)
  
    if Dynamics.state_metric(x_i, x) < best_dist:
      best_dist = Dynamics.state_metric(x_i, x)
      best_state = x_i
      best_control = i

  return [best_state, best_control]

def EXTEND(rrt_graph, rrt_states, rrt_controls, x, u_control):
  '''
      Description:
        Extends rrt_graph, and its corresponding states and controls towards a given 'random' state x with specified control

      Input:
        (*) rrt_graph
        (*) rrt_states
        (*) rrt_controls
        (*) x
        (*) u_control
        
      Output:
        (*) index = index of the nearest state 
        (*) control = appled control
        (*) new_state
  '''

  # Find nearest neighbor state
  nbrs = NearestNeighbors(n_neighbors=1, algorithm='ball_tree',
              metric='pyfunc', func=Dynamics.state_metric).fit(rrt_states)
  distances, indices = nbrs.kneighbors(x)  
  index = indices[0,0]  
  
  # Handle case when rrt_states is initial state array 
  if rrt_states.shape == (13,):
    x_near = np.zeros(13) + rrt_states
  else:
    x_near = np.zeros(13) + rrt_states[index]

  # Find new state and new control
  new_state, new_control = NEW_STATE(x, x_near, u_control)
  return [index, new_state, new_control]