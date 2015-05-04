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

import Fleet

PI = math.pi

animation_path = '../../../Animations/'

#'''
###################################
#DEFINE FLEET STATES off-axis tumble
##################################

blondie = np.zeros(13)
blondie[0:3] = np.zeros(3) + np.array([1,1,1])
blondie[3] = 1
blondie[10:13] = np.zeros(3) + .25*PI

angeleyes = np.zeros(13)
angeleyes[0:3] = np.zeros(3) + np.array([2,2,2])
angeleyes[3] = 1
angeleyes[10:13] = np.zeros(3) + .25*PI

tuco = np.zeros(13)
tuco[0:3] = np.zeros(3) + np.array([3,3,3])
tuco[3] = 1
tuco[10:13] = np.zeros(3) + .25*PI

initial_fleet = np.vstack([blondie, angeleyes, tuco])

del_x = 1.
final_a = np.zeros(13)
final_a[0:3] = np.array([6. + del_x, 6., 6.])
final_a[3] = 1

final_b = np.zeros(13)
final_b[0:3] = np.array([6., 6. + del_x, 6.])
final_b[3] = 1

final_c = np.zeros(13)
final_c[0:3] = np.array([6., 6., 6. + del_x])
final_c[3] = 1

final_fleet = np.vstack([final_a, final_b, final_c])

############################
#COMPUTE AND PLOT FLEET PATHS
############################

eps = 0.05
#Controls for 3D body with rotation
U_3DwithRot = \
		   np.array([[0.,	0.,		0.,		0.,		0.,		0.],
					 [0.,	0.,		0.,		.01,	0.,		0.],
					 [0.,	0.,		0.,		-.01,	0., 	0.],
  					 [0.,	0.,		0.,		0.,		.01,	0.],
					 [0.,	0.,		0.,		0.,		-.01,	0.],
					 [0.,	0.,		0.,		0.,		0.,		.01],
 					 [0.,	0.,		0.,		0.,		0.,		-.01],
					 [0.,	1.,		0.,		0.,		0.,		0.],
					 [0.,	-1.,	0.,		0.,		0.,		0.]])
fleet_paths = Fleet.fleet_simple(initial_fleet, final_fleet, U_3DwithRot)

Fleet.plot_fleet(fleet_paths)

'''
np.set_printoptions(threshold='nan')


path_length = len(fleet_paths[0,:,0])
text_file_a = open("path_a.txt", "w")
text_file_b = open("path_b.txt", "w")
text_file_c = open("path_c.txt", "w")



text_file_a.write(str(fleet_paths[0,:,:]))
text_file_b.write(str(fleet_paths[1,:,:]))
text_file_c.write(str(fleet_paths[2,:,:]))

text_file_a.close()
text_file_b.close()
text_file_c.close()
'''


#Fleet.animate_fleet(fleet_paths, 'fleet_test.mp4')

''''
def state_to_fleet(fleet_state):

	n_fleet = len(fleet_state[:,0])
	
	ax = Axes3D(fig)
	
	for i in range(n_fleet):
		
		sat = fleet_state[i,:]
		
		Fleet.state_to_cube(sat, ax)
	
	return ax


#####################
#ANIMATION
#####################


#f = open('fleet_off_axis_tumble1.pckl')
#fleet_path = pickle.load(f)
#f.close()

#fleet_path = fleet_big[:, 1:10, :]

n_frames = len(fleet_paths[0,:,0]) - 1

fig = plt.figure()
ax = Axes3D(fig)
ax.set_xlim([0, 10])
ax.set_ylim([0, 10])
ax.set_zlim([0, 10])

def init():	
	return ax,

def animate(i):
	ax_fleet = state_to_fleet(fleet_paths[:,i,:])
	print('		Progress:	' + str('%.2f' % (100.*i/n_frames)) + '%')
	return ax_fleet

# call the animator.  blit=True means only re-draw the parts that have changed.
anim = animation.FuncAnimation(fig, animate, init_func=init,
                               frames=n_frames, interval=100, blit=True)

print('SAVING ANIMATION...')
anim.save(animation_path + 'fleet_test.mp4')
'''