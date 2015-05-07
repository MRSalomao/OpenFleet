#!/usr/bin/env python
import numpy as np
import networkx as nx
from sklearn.neighbors import NearestNeighbors
import random
import math
import Dynamics


# Define PI as a global variable
PI = math.pi

class RRT:
  def __init__(self, eps, spatial_boundary, velocity_boundary, dt):
    '''
    Description:
    Class RRT.
    
    Implemented RRT Algorithms:
    * 1-Way RRT (OBSOLETE)
    * Bidirectional RRT
    * Recursive asymmetric BiD RRT
    
    Input:
    (*) eps = Minimum distance between trees
    (*) spatial_boundary = Defines the random space boundaries
    (*) velocity_boundary
    (*) dt
    '''
    
    self.eps = eps
    self.spatial_boundary = spatial_boundary
    self.velocity_boundary = velocity_boundary
    self.dt = dt

  def generate_BiD_path(self, start_state, goal_state, controls, N, environment):
    '''
        Description:
        This function generates path between initial and goal states with bidirectional rrt. If it sucesed, it returns the full rrt state table, the path table and the control table for both start and end trees.

        Input:
          (*) start_state
          (*) goal_state
          (*) controls = 3D controls
          (*) N =  Number of solution trajectories
          (*) environment = List of bounding boxes
          
        Output:
          (*) A list with:
          [rrt_states_i, path_states_i, path_controls_i,
          rrt_states_g, path_states_g, path_controls_g]
    '''
    
    initial_state = start_state
    final_state = goal_state

    # Initialize RRT from Initial State
    rrt_graph_i = nx.Graph()
    rrt_graph_i.add_node(0)

    # Initialize lookup tables
    rrt_states_i = initial_state
    rrt_controls_i = [0]

    # Initialize RRT from Goal State
    rrt_graph_g = nx.Graph() 
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
      rand_state = self.random_state()  

      # Extend tree A towards a random state
      # Keep in mind that A is not always the tree that contains the initial state 
      index_A, newstate_A, newcontrol_A = self.EXTEND(RRT[0][1], RRT[0][2], RRT[0][3], rand_state, controls)
      
      # Check for collision with environment
      if environment.collide_environment(newstate_A):
        if count%50==0:
          print('COLLISION - TREE A!')
          print('(x,y,z):         ' + str(newstate_A[0:3]))
        continue
      
      # Update lookup tables A
      RRT[0][2] = np.vstack([RRT[0][2], newstate_A])
      RRT[0][3] = np.column_stack([RRT[0][3], newcontrol_A])
              
      # Update graphs A
      n_nodes_A = RRT[0][2].shape[0] - 1
      RRT[0][1].add_node(n_nodes_A)
      RRT[0][1].add_edge(index_A, n_nodes_A)

      # Extend tree B towards newstate A
      index_B, newstate_B, newcontrol_B = self.EXTEND(RRT[1][1], RRT[1][2], RRT[1][3], newstate_A, -controls)
      
      if environment.collide_environment(newstate_B):
        if count%50==0:
          print('COLLISION - TREE B!')
          print('(x,y,z):         ' + str(newstate_A[0:3]))
        continue
                      
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
      
      if dist < best_dist:
        best_dist = dist

      if (count%50) == 0:
        print('  NODE:  ' + str(count) + '  N:  ' +str(T))
        print('  BEST DISTANCE:    ' + str(best_dist))
        
      if dist <= self.eps:
        RRT[0][4] = nx.shortest_path(RRT[0][1], 0, n_nodes_A)
        RRT[1][4] = nx.shortest_path(RRT[1][1], 0, n_nodes_B)
        T += 1
        
      # Swap the Trees
      RRT = [ RRT[1], RRT[0] ]
      count += 1
    
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

  def RaBiD_RRT(self, start_state, goal_state, sibling_path, N, environment, controls):
    '''
    Description:
    This function generates path between initial and goal states with recursive bidirectional rrt. If it sucesed, it returns the full rrt state table, the path table and the control table for both start and end trees.
      
    Input:
    (*) start_state
    (*) goal_state
    (*) controls = 3D controls
    (*) N =  Number of solution trajectories
    (*) environment = List of bounding boxes
    (*) sibling_path
    
    Output:
    (*) A list with:
    [rrt_states_i, path_states_i, path_controls_i,
    rrt_states_g, path_states_g, path_controls_g]
    '''
    n_siblings = len(sibling_path[:,0,0])
    sibling_length = len(sibling_path[0,:,0])
    sibling_final = sibling_path[:,sibling_length-1,:]
        
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
      rand_state = self.random_state()
      
      '''
      Reminder:
      RRT[0] = Letter
      RRT[1] = rrt_graph
      RRT[2] = rrt_states
      RRT[3] = rrt_controls
      RRT[4] = path
      '''
      
      # Extend tree A towards a random state
      # Keep in mind that A is not always the tree that contains the initial state 
      index_A, newstate_A, newcontrol_A = self.EXTEND(RRT[0][1], RRT[0][2], RRT[0][3], rand_state, controls)
      
      # Check for collision with environment
      if environment.collide_environment(newstate_A):
        if count%50==0:
          print('COLLISION!')
          print('(x,y,z):         ' + str(newstate_A[0:3]))
        continue
      
      depth = nx.shortest_path_length(RRT[0][1], 0, index_A)
      # Caution: indexing error possible                 
      if depth + 1 <= sibling_length:
        # Truncate siblings
        sibling_states = sibling_path[:,depth,:]
      else:
        sibling_states = sibling_path[:,sibling_length-1,:]

      # Check for a collision skip
      if Planner.valid_state(newstate_A, sibling_states) == False:
        continue
      
      # Update lookup tables A
      RRT[0][2] = np.vstack([RRT[0][2], newstate_A])
      RRT[0][3] = np.column_stack([RRT[0][3], newcontrol_A])
        
      # Update graphs A
      n_nodes_A = RRT[0][2].shape[0] - 1
      RRT[0][1].add_node(n_nodes_A)
      RRT[0][1].add_edge(index_A, n_nodes_A)
        
      # Extend tree B towards newstate A
      index_B, newstate_B, newcontrol_B = self.EXTEND(RRT[1][1], RRT[1][2], RRT[1][3], newstate_A, -controls)
      
      # Check for collision with environment
      if environment.collide_environment(newstate_B):
        if count%50==0:
          print('COLLISION!')
          print('(x,y,z):         ' + str(newstate_B[0:3]))
        continue
      
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
            
      if dist <= self.eps:
        RRT[0][4] = nx.shortest_path(RRT[0][1], 0, n_nodes_A)
        RRT[1][4] = nx.shortest_path(RRT[1][1], 0, n_nodes_B)
        
        '''
        Reminder:
        RRT[0] = Letter
        RRT[1] = rrt_graph
        RRT[2] = rrt_states
        RRT[3] = rrt_controls
        RRT[4] = path
        '''
        
        path_states_A = np.zeros([len(RRT[0][4]), 13])
        for i in range(len(RRT[0][4])):
          path_states_A[i,:] = RRT[0][2][RRT[0][4][i],:]
        
        path_states_B = np.zeros([len(RRT[1][4]), 13])
        for i in range(len(RRT[1][4])):
          path_states_B[i,:] = RRT[1][1][RRT[1][4][i],:]
        
        a_states = path_states_A
        b_states = np.fliplr(path_states_B.transpose()).transpose()                             
        a_length = len(path_states_A); b_length = len(b_states)
        sibling_path_2 = np.zeros([n_siblings, len(b_states), 13])
        
        if sibling_length <= a_length:
          for i in range(b_length):
            sibling_path_2[:,i,:] = sibling_final

        elif sibling_length > a_length and sibling_length <= (a_length + b_length):
          for i in range(sibling_length-a_length):
            sibling_path_2[:,i,:] = sibling_path[:,i+a_length,:]
            
          for i in range(a_length+b_length-sibling_length):
            sibling_path_2[:,i,:] = sibling_final
          
        else:
          for i in range(b_length):
            sibling_path_2[:,i,:] = sibling_path[:,i+a_length,:]
            
        # Beauitful recursive part
        if Planner.collision_paths(b_states, sibling_path_2) == True:
          print('         Collision Found in 2nd half')
          print('         Recursive BiD search...')
          
          b_states = self.RaBiD_RRT(b_states[0,:], b_states[b_length-1], sibling_path_2, N, environment, controls)
        else:
          pass
        
        path = np.vstack((a_states, b_states))
        return path
        T += 1
              
      else:
        # Swap the Trees
        RRT = [ RRT[1], RRT[0] ]

  def random_state(self):
    '''
        Description:
        It generates a random state
          
        Output:
          (*) Random State = [px, py, pz, qt, qx, qy, qz, vx, vy, vz, wx, wy, wz]
    '''
    
    # Random Parameters
    angular_velocity_boundary = 2*PI
    px = random.random()*self.spatial_boundary
    py = random.random()*self.spatial_boundary
    pz = random.random()*self.spatial_boundary
    
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
    vx = random.random()*self.velocity_boundary
    vy = random.random()*self.velocity_boundary
    vz = random.random()*self.velocity_boundary

    # Angular velocities
    wx = random.random()*angular_velocity_boundary 
    wy = random.random()*angular_velocity_boundary
    wz = random.random()*angular_velocity_boundary

    return np.array([px, py, pz, qt, qx, qy, qz, 
                    vx, vy, vz, wx, wy, wz])

  def propagate(self, x, u, del_t):
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

    n = int(del_t / self.dt)
    x_prop = x
    for i in range(n):    
    
        k1 = Dynamics.x_dot(x_prop, u)
        k2 = Dynamics.x_dot(x_prop + .5*self.dt*k1, u)
        k3 = Dynamics.x_dot(x_prop + .5*self.dt*k2, u)
        k4 = Dynamics.x_dot(x_prop + self.dt*k3, u)    
    
        x_prop += self.dt*(k1+2.*k2+2.*k3+k4)/6.
        
    return x_prop

  def NEW_STATE(self, x, x_near, u_control):
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
      x_i = self.propagate(x_o, u_i, T)
    
      if Dynamics.state_metric(x_i, x) < best_dist:
        best_dist = Dynamics.state_metric(x_i, x)
        best_state = x_i
        best_control = i

    return [best_state, best_control]

  def EXTEND(self, rrt_graph, rrt_states, rrt_controls, x, u_control):
    '''
        Description:
          Extends rrt_graph, and its corresponding states and controls towards a given 'random' state x with specified control

        Input:
          (*) rrt_graph
          (*) rrt_states
          (*) rrt_controls
          (*) x random state
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
    new_state, new_control = self.NEW_STATE(x, x_near, u_control)
    return [index, new_state, new_control]