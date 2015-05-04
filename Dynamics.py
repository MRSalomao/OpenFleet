#!/usr/bin/env python
import numpy as np
import math
from numpy.linalg import inv

'''
  Cube Mass and Geometry paramenters
  Check more alternatives at:
    (*) https://en.wikipedia.org/wiki/List_of_moments_of_inertia
'''
M = 1
l = .5
w = .5
d = .5
I = (1./12)*M*np.array([[(l*l + d*d), 0., 0.],
                        [0., (w*w + d*d), 0.],
                        [0., 0., (w*w + l*l)]])

def state_metric(x, y):
  '''
      Description:
        Defines state metrics

      Input:
        (*) State x
        (*) State y
        
      Output:
        (*) dist
  '''

  w_p = .25
  w_q = .00025
  w_v = .025
  w_w = .025
  delta = x - y

  dist = w_p*(np.dot(delta[0:3],delta[0:3]))
  dist += w_q*((1 - math.fabs(np.dot(x[3:7],y[3:7])))**2) 
  dist += w_v*(np.dot(delta[7:10], delta[7:10]))
  dist += w_w*(np.dot(delta[10:13], delta[10:13]))

  return dist

def quaternion_product(q, r):
  '''
      Description:
        Computes the quaternion product of two quaternion vectors

      Input:
        (*) quaternion vector q
        (*) quaternion vector r
        
      Output:
        (*) quaternion product
  '''
  
  t = np.zeros(4)

  # this could be simplified (= x-product - dot product)
  t[0] = r[0]*q[0] - r[1]*q[1] - r[2]*q[2] - r[3]*q[3]
  t[1] = r[0]*q[1] + r[1]*q[0] - r[2]*q[3] + r[3]*q[2]
  t[2] = r[0]*q[2] + r[1]*q[3] + r[2]*q[0] - r[3]*q[1]
  t[3] = r[0]*q[3] - r[1]*q[2] + r[2]*q[1] + r[3]*q[0]
  
  return t


def quaternion_matrix(quaternion):
  '''
      Description:
        Converts a quaternion vector to its corresponding rotation matrix

      Input:
        (*) quaternion vector
        
      Output:
        (*) rotation matrix
  '''

  qw = quaternion[0]
  qx = quaternion[1]
  qy = quaternion[2]
  qz = quaternion[3]
  
  return np.array([[1-2*qy*qy-2*qz*qz, 2*qx*qy-2*qz*qw, 2*qx*qz+2*qy*qw],
                   [2*qx*qy+2*qz*qw, 1-2*qx*qx-2*qz*qz, 2*qy*qz-2*qx*qw],
                   [2*qx*qz-2*qy*qw,  2*qy*qz+2*qx*qw, 1-2*qx*qx-2*qy*qy] ])

def x_dot(x, u):
  '''
      Description:
        Computes the state derivative as a function of the state and control. This is the heart of the dynamics system

      Input:
        (*) 
        
      Output:
        (*) 
  '''

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

def calc_radius():
  '''
    Describes a Sphere based collision checking
  '''
  radius = math.sqrt((l/2.)**2 + (w/2.)**2 + (d/2.)**2)
  return radius