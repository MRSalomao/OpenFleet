#!/usr/bin/env python
import numpy as np
import math
import Dynamics
import RRT
from munkres import Munkres, print_matrix

class Planner:
  def __init__(self, N, obs, eps, spatial_boundary, velocity_boundary, dt):
    '''
    Description:
    A Planner Based on RRT.
    
    This class is the heart of our planner system. Implemented RRT Algorithms:
     * 1-Way RRT (OBSOLETE)
     * Bidirectional RRT
     * Recursive asymmetric BiD RRT
    
    Input:
      (*) N = Number of solution trajectories
        From Lavalle: Our basic algorithm stops at the first solution trajectory found, but one could continue to grow the trees and maintain a growing collection of solution trajectories. The "best" solution found so far can be chosen according to a cost functional based on some criterion (such as execution time or energy expended).
      (*) obs = List of bounding boxes
      (*) eps = Minimum distance between trees
      (*) spatial_boundary = Defines the random space boundaries
      (*) velocity_boundary
      (*) dt
    '''
    
    # Random Paramenters
    self.N = N
    self.obs = obs
    
    self.eps = eps
    self.spatial_boundary = spatial_boundary
    self.velocity_boundary = velocity_boundary
    self.dt = dt
    
    # Controls for 3D body with rotation
    self.control = np.array([[0.,  0.,    0.,    0.,    0.,    0.],
                            [0.,  0.,    0.,    .01,  0.,    0.],
                            [0.,  0.,    0.,    -.01,  0.,   0.],
                            [0.,  0.,    0.,    0.,    .01,  0.],
                            [0.,  0.,    0.,    0.,    -.01,  0.],
                            [0.,  0.,    0.,    0.,    0.,    .01],
                            [0.,  0.,    0.,    0.,    0.,    -.01],
                            [0.,  1.,    0.,    0.,    0.,    0.],
                            [0.,  -1.,  0.,    0.,    0.,    0.]])
    

  def Fleet_Simple(self, initial_fleet, final_fleet, algorithm):
    '''
      Description:
      Simple fleet planner. Applies a bi-partite matching algorithm based on metric distances.

      Input:
        (*) initial_fleet
        (*) final_fleet
        (*) RRT Algorithm. Possible Values:
          * 1WAY: 1-Way RRT (OBSOLETE)
          * 2WAY: Bidirectional RRT
          * REC: Recursive asymmetric BiD RRT

      Output:
        (*) fleet paths is a 3d array (fleet size x path length x state size)
    '''
    
    # Size of fleet
    n_fleet = len(initial_fleet[:,0])

    # Compute cost matrix for each match
    cost_matrix = np.zeros([n_fleet, n_fleet])

    for i in range(n_fleet):
      for j in range(n_fleet): 
        cost_matrix[i,j] = Dynamics.state_metric(initial_fleet[i,:], final_fleet[j,:])

    # Compute bipartite matching assignments (hungarian ~o(n^3), n: n_fleet)
    target_fleet = np.zeros([n_fleet,13])
    m = Munkres()
    indices = m.compute(cost_matrix)
    assignments = np.zeros(n_fleet)
    for i in range(n_fleet):
      assignments[i] = indices[i][1]  
      target_fleet[i,:] = final_fleet[assignments[i],:] 
    
    # Compute paths based on a given RRT algorithm
    if (algorithm == '1WAY'):
      print "1-Way RRT is OBSOLETE."
      return False
    
    elif (algorithm == '2WAY'):
      fleet_paths = self.Fleet_Sequential_Avoidance(initial_fleet, target_fleet)
      
    elif (algorithm == 'REC'):
      fleet_paths = self.fleet_RaBiD(initial_fleet, target_fleet)
      
    return fleet_paths
  
  def Fleet_Sequential_Avoidance(self, initial_fleet, target_fleet):
    '''
      Description:
      This function verifies for collision between Satellites and generates a Bid-RRT 

      Input:
        (*) initial_fleet
        (*) target_fleet
        
      Output:
        (*) fleet paths is a 3d array (fleet size x path length x state size)
    '''
    
    # Create a RRT object
    BID_RRT = RRT.RRT(self.eps, self.spatial_boundary, self.velocity_boundary, self.dt)
    
    # Calculate the trajectory for the first satellite
    n_fleet = len(initial_fleet[:,0])
    num_siblings = 1

    print("Sat path " + str(0))
    
    # Generate RRT
    rrt_a, path_a, u_a, rrt_b, path_b, u_b = BID_RRT.generate_BiD_path(initial_fleet[0,:], target_fleet[0,:], self.control, self.N, self.obs)
    
    b_flip = np.fliplr(path_b.transpose()).transpose()
    sat_path = np.vstack([path_a, b_flip])

    sibling_paths = np.zeros([num_siblings, len(sat_path[:,0]), 13])
    sibling_paths[0,:,:] = sat_path
    max_sat_path = len(sat_path[:,0])

    print('  First Sat path shape:  ' + str(np.shape(sibling_paths))) 
    
    # Then, calculate the trajectories for others satellites and compare them to verify if a 
    # collision happened
    i = 1
    
    while i < n_fleet:
      print("Sat path " + str(i))
      
      rrt_a, path_a, u_a, rrt_b, path_b, u_b = BID_RRT.generate_BiD_path(initial_fleet[i,:], target_fleet[i,:],
                                                                     self.control, self.N, self.obs)
      
      b_flip = np.fliplr(path_b.transpose()).transpose()
      sat_path = np.vstack([path_a, b_flip])

      sat_length = len(sat_path[:,0])

      if sat_length <= max_sat_path:
        # Truncate sibling paths
        sat_siblings =  sibling_paths[:,0:sat_length,:]
        
      else:
        # Extend sibling paths
        sat_siblings = np.zeros([num_siblings, sat_length, 13])
        sat_siblings[: ,0:max_sat_path, :] = sibling_paths
        
        # CAUTION: index error possible
        for j in range(max_sat_path,sat_length):
          sat_siblings[:,j,:] = sat_siblings[:,max_sat_path-1,:]
          
      if self.collision_paths(sat_path, sat_siblings):
        print("    BAD PATH, RECOMPUTING TRAJECTORY...")
        continue
      
      print('  New Sibling shape:  ' + str(np.shape(sat_path)))  
      print('  Siblings Path shape:   ' + str(np.shape(sibling_paths)))  
      
      i += 1
      num_siblings += 1
      if sat_length <= max_sat_path:
        # Extend sat_path
        sat_path_extend = np.zeros([max_sat_path,13])
        sat_path_extend[0:sat_length] = sat_path
        
        for j in range(sat_length,max_sat_path):
          sat_path_extend[j,:] = sat_path_extend[sat_length-1,:]
        
        # Update sibling path  
        new_sibling_paths = np.zeros([num_siblings, max_sat_path, 13])
        new_sibling_paths[0:num_siblings-1, :, :] = sibling_paths
        new_sibling_paths[num_siblings-1,:,:] = sat_path_extend
        
        sibling_paths = new_sibling_paths
        
      else:
        # Extend sibling paths
        sibling_paths_extend = np.zeros([num_siblings-1, sat_length, 13])
        sibling_paths_extend[:,0:max_sat_path,:] = sibling_paths
        for j in range(max_sat_path,sat_length):
          sibling_paths_extend[:,j,:] = sibling_paths_extend[:,max_sat_path-1,:]
        
        # Update sibling path AND max_sat_path  
        new_sibling_paths = np.zeros([num_siblings, sat_length, 13])
        new_sibling_paths[0:num_siblings-1, :, :] = sibling_paths_extend
        new_sibling_paths[num_siblings-1,:,:] = sat_path
        max_sat_path = sat_length
        
        sibling_paths = new_sibling_paths
        
    return sibling_paths

  def fleet_RaBiD(self, initial_fleet, target_fleet):
    '''
      Description:
      This function verifies for collision between Satellites and generates a Recursive Bid-RRT 

      Input:
        (*) initial_fleet
        (*) target_fleet
        
      Output:
        (*) fleet paths is a 3d array (fleet size x path length x state size)
    '''
    
    # Create a RRT object
    # Paramenters:
    # eps = .5, spatial_boundary = 12, velocity_boundary = 1, dt = .01
    REC_RRT = RRT.RRT(self.eps, self.spatial_boundary, self.velocity_boundary, self.dt)

    n_fleet = len(initial_fleet[:,0])
    num_siblings = 1

    print("Sat path " + str(0))
    rrt_a, path_a, u_a, rrt_b, path_b, u_b = REC_RRT.generate_BiD_path(initial_fleet[0,:], target_fleet[0,:], self.control, self.N, self.obs)
    
    b_flip = np.fliplr(path_b.transpose()).transpose()
    sat_path = np.vstack([path_a, b_flip])
    
    sibling_paths = np.zeros([num_siblings, len(sat_path[:,0]), 13])
    sibling_paths[0,:,:] = sat_path
    max_sat_path = len(sat_path[:,0])

    print(' First Sat path shape:   ' + str(np.shape(sibling_paths)))               

    i = 1
    while i < n_fleet:
      print("Sat path " + str(i))
      sat_path = REC_RRT.RaBiD_RRT(initial_fleet[i,:], final_fleet[i,:], sibling_paths, self.N, self.obs, self.control)

      sat_length = len(sat_path[:,0])

      print(' New Sibling shape:      ' + str(np.shape(sat_path)))    
      print(' Siblings Path shape:    ' + str(np.shape(sibling_paths)))       

      i += 1
      num_siblings += 1
      if sat_length <= max_sat_path:
        # Extend sat_path
        sat_path_extend = np.zeros([max_sat_path,13])
        sat_path_extend[0:sat_length] = sat_path

        for j in range(sat_length,max_sat_path):
          sat_path_extend[j,:] = sat_path_extend[sat_length-1,:]

        # Update sibling path    
        new_sibling_paths = np.zeros([num_siblings, max_sat_path, 13])
        new_sibling_paths[0:num_siblings-1, :, :] = sibling_paths
        new_sibling_paths[num_siblings-1,:,:] = sat_path_extend
        sibling_paths = new_sibling_paths
        
      else:
        # Extend sibling paths
        sibling_paths_extend = np.zeros([num_siblings-1, sat_length, 13])
        sibling_paths_extend[:,0:max_sat_path,:] = sibling_paths
        for j in range(max_sat_path,sat_length):
          sibling_paths_extend[:,j,:] = sibling_paths_extend[:,max_sat_path-1,:]
          
        # Update sibling path AND max_sat_path   
        new_sibling_paths = np.zeros([num_siblings, sat_length, 13])
        new_sibling_paths[0:num_siblings-1, :, :] = sibling_paths_extend
        new_sibling_paths[num_siblings-1,:,:] = sat_path
        max_sat_path = sat_length
        
        sibling_paths = new_sibling_paths

    return new_sibling_paths

  def collision(self, state_a, state_b):
    '''
    Description:
    Sphere based collision checking

    Input:
      (*) State A
      (*) State B
      
    Output:
      (*) True if a collision happened
    '''
    
    radius = Dynamics.calc_radius()
    xa = state_a[0]; ya = state_a[1]; za = state_a[2];
    xb = state_b[0]; yb = state_b[1]; zb = state_b[2];
    
    if (xa-xb)**2. + (ya-yb)**2. + (za-zb)**2. > 4.*radius**2.:
      return False
    else:
      return True

  def valid_state(self, state, siblings):
    '''
    Description:
    Validation of a state

    Input:
      (*) State
      (*) Siblings
      
    Output:
      (*) True if the state is valid
    '''
    
    n_sibilings = len(siblings[:,0]) 
    for i in range(n_sibilings):
      if self.collision(state,siblings[i]):
        return False
      
    return True

  def collision_paths(self, path, sibling_paths):
    '''
    Description:
    Check whether a generate path has a collision with other satellites

    Input:
      (*) path
      (*) Siblings
      
    Output:
      (*) True if a collision happened
    '''
    path_length = len(path[:,0])
    
    for i in range(path_length):
      if self.valid_state(path[i,:], sibling_paths[:,i,:]) == False:
        return True
    
    return False