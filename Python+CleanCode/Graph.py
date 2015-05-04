#!/usr/bin/env python
import cPickle
import pickle
import gzip
import os
import sys
import scipy
import scipy.ndimage
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from matplotlib import animation
from munkres import Munkres, print_matrix

import numpy as np

import Dynamics

def state_to_cube(state, ax):
  '''
    Returns a graphical rendering of a state
  '''
  
  M = 1
  l = .5
  w = .5
  d = .5
  I = (1./12)*M*np.array([[(l*l + d*d), 0., 0.],
                        [0., (w*w + d*d), 0.],
                        [0., 0., (w*w + l*l)]])

  quat = state[3:7]
  pos = state[0:3]

  corners = np.array([[w/2,   l/2,   d/2],
                      [w/2,   l/2,   -d/2],
                      [w/2,   -l/2,   d/2],
                      [w/2,   -l/2,   -d/2],
                      [-w/2,   l/2,   d/2],
                      [-w/2,   l/2,   -d/2],
                      [-w/2,   -l/2,   d/2],
                      [-w/2,   -l/2,   -d/2]])

  R = Dynamics.quaternion_matrix(quat)

  rotated_corners = np.dot(R, corners.transpose())

  cube_corners = rotated_corners + np.array([pos,]*8).transpose()

  #  ax = Axes3D(fig)
  ax.set_xlim([0, 10])
  ax.set_ylim([0, 10])
  ax.set_zlim([0, 10])
  ax.set_xlabel('X Label')
  ax.set_ylabel('Y Label')
  ax.set_zlabel('Z Label')
  
  # FACE +x
  x_corners = np.array([cube_corners[0,0],cube_corners[0,2],
               cube_corners[0,3],cube_corners[0,1]])
  y_corners = np.array([cube_corners[1,0],cube_corners[1,2],
               cube_corners[1,3],cube_corners[1,1]])
  z_corners = np.array([cube_corners[2,0],cube_corners[2,2],
               cube_corners[2,3],cube_corners[2,1]])
  verts = [zip(x_corners, y_corners, z_corners)]
  ax.add_collection3d(Poly3DCollection(verts))
  
  # FACE -x
  x_corners = np.array([cube_corners[0,4],cube_corners[0,5],
               cube_corners[0,7],cube_corners[0,6]])
  y_corners = np.array([cube_corners[1,4],cube_corners[1,5],
               cube_corners[1,7],cube_corners[1,6]])
  z_corners = np.array([cube_corners[2,4],cube_corners[2,5],
               cube_corners[2,7],cube_corners[2,6]])
  verts = [zip(x_corners, y_corners, z_corners)]
  ax.add_collection3d(Poly3DCollection(verts))

  # FACE +y
  x_corners = np.array([cube_corners[0,0],cube_corners[0,1],
               cube_corners[0,5],cube_corners[0,4]])
  y_corners = np.array([cube_corners[1,0],cube_corners[1,1],
               cube_corners[1,5],cube_corners[1,4]])
  z_corners = np.array([cube_corners[2,0],cube_corners[2,1],
               cube_corners[2,5],cube_corners[2,4]])
  verts = [zip(x_corners, y_corners, z_corners)]
  ax.add_collection3d(Poly3DCollection(verts))
  
  # FACE -y
  x_corners = np.array([cube_corners[0,2],cube_corners[0,3],
               cube_corners[0,7],cube_corners[0,6]])
  y_corners = np.array([cube_corners[1,2],cube_corners[1,3],
               cube_corners[1,7],cube_corners[1,6]])
  z_corners = np.array([cube_corners[2,2],cube_corners[2,3],
               cube_corners[2,7],cube_corners[2,6]])
  verts = [zip(x_corners, y_corners, z_corners)]
  ax.add_collection3d(Poly3DCollection(verts))
  
  # FACE +z
  x_corners = np.array([cube_corners[0,0],cube_corners[0,4],
               cube_corners[0,6],cube_corners[0,2]])  
  y_corners = np.array([cube_corners[1,0],cube_corners[1,4],
               cube_corners[1,6],cube_corners[1,2]])
  z_corners = np.array([cube_corners[2,0],cube_corners[2,4],
               cube_corners[2,6],cube_corners[2,2]])
  verts = [zip(x_corners, y_corners, z_corners)]
  ax.add_collection3d(Poly3DCollection(verts))
  
  # FACE +z
  x_corners = np.array([cube_corners[0,1],cube_corners[0,5],
               cube_corners[0,7],cube_corners[0,3]])
  y_corners = np.array([cube_corners[1,1],cube_corners[1,5],
               cube_corners[1,7],cube_corners[1,3]])
  z_corners = np.array([cube_corners[2,1],cube_corners[2,5],
               cube_corners[2,7],cube_corners[2,3]])
  verts = [zip(x_corners, y_corners, z_corners)]
  ax.add_collection3d(Poly3DCollection(verts))
  
  return ax

def state_to_fleet(fleet_state, fig):
  '''
  '''

  n_fleet = len(fleet_state[:,0])

  ax = Axes3D(fig)

  for i in range(n_fleet):

    sat = fleet_state[i,:]

    state_to_cube(sat, ax)

  return ax

def animate_fleet(fleet_paths, animation_file):
  '''
    Generates the 3D animation
  '''
  animation_path = './'
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
    print('    Progress:  ' + str('%.2f' % (100.*i/n_frames)) + '%')
    return ax_fleet

  # call the animator.  blit=True means only re-draw the parts that have changed.
  anim = animation.FuncAnimation(fig, animate, init_func=init,
                                 frames=n_frames, interval=100, blit=True)

  print('SAVING ANIMATION...')
  anim.save(animation_path + animation_file)
             
def plot_fleet(fleet_paths):
  '''
    Plots 3d bidirectional paths for fleet   
  '''

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