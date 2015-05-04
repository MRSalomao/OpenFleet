#!/usr/bin/env python
import numpy as np
import math

class SmallSat:
  def __init__(self):
    '''
    Description:
    SmallSat constructor.
    
    TODO: 
      (*) Add mass and geometry parameters here
    '''
    
    # States
    self.Initial_State = np.zeros(13)
    self.Final_State = np.zeros(13)
    
    # Mass and Geometry
    
class BoundingBox:
  def __init__(self,BoxConfigurations,offset):
    '''
    Description:
    BoundingBox constructor.
    
    This class receives the center of mass and dimensions from a .txt file.
    '''
    
    self.CenterOfMass = str(BoxConfigurations).split(" ")[0:3]
    self.Dimensions = str(BoxConfigurations).split(" ")[3:6]
    
    # Just making sure all the values are float
    self.CenterOfMass = [float(i) for i in self.CenterOfMass]
    self.CenterOfMass += offset 
    
    self.Dimensions = [float(i) for i in self.Dimensions]
    
  def CollisionCheck(self, p):
    '''
    Description:
    Collision check between the box center of mass and a given point (x,y,z).
    
    This function was designed to be used with random_state() function.
    
    How it works:
      1. Find the largest dimension. The spheres will be placed over its axis.
      2. The second largest dimension defines the spheres radius 
      3. Caculate how many spheres will be placed over the largest dimension axis. Ratio (axis_size/radius).
      
    Input:
      (*) p = Generated random state
      
    Output:
      (*) True = A collision was detected
      (*) False = Collison free
    '''
    
    # Sort the Box Dimensions
    a = sorted(self.Dimensions)
    axis_index = self.Dimensions.index(a[2])
    axis_size = a[2]
    sphere_diameter = a[1]
    
    ratio = int(axis_size/(sphere_diameter/2)) + 1
    
    s = np.array(self.CenterOfMass)
    s[axis_index] -= axis_size/2
    
    for j in range(ratio):
      s[axis_index] += j*sphere_diameter/2
      if math.sqrt( (p[0]-s[0])**2 + (p[1]-s[2])**2 + (p[2]-s[2])**2 ) <= 1.1*(sphere_diameter/2):
        return True
      
    return False