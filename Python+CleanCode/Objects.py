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
    
class Environment:
  def __init__(self,txt):
    '''
    Description:
    Environment constructor.
    
    This class receives the center of mass and dimensions from a .txt file.
    '''
    
    self.boxes = np.zeros(6)
    with open(txt, "r") as f:
      for line in f:
        box = str(line).split(" ")[0:6]
        box = np.asarray(box)
        box = box.astype(np.float)
        self.boxes = np.vstack([self.boxes,box])

    self.boxes = np.delete(self.boxes, (0), axis=0)
    
  def collide_environment(self, state):
    '''
    Description:
    Collision check between the box center of mass and a given point (x,y,z).
    
    Input:
      (*) state
      
    Output:
      (*) True = A collision was detected
      (*) False = Collison free
    '''
    
    l = .5
    w = .5
    d = .5
    
    n_boxes = len(self.boxes[:,0])
    r = math.sqrt((l/2.)**2 + (w/2.)**2 + (d/2.)**2)
    x = state[0]; y = state[1]; z = state[2];
    
    for i in range(n_boxes):
        
        xc = self.boxes[i,0]; yc = self.boxes[i,1]; zc = self.boxes[i,2]
        dx = self.boxes[i,3]; dy = self.boxes[i,4]; dz = self.boxes[i,5]
        
        if (x+r) >= (xc - .5*dx) and (x-r) <= (xc + .5*dx) and \
           (y+r) >= (yc - .5*dy) and (y-r) <= (yc + .5*dy) and \
           (z+r) >= (zc - .5*dz) and (z-r) <= (zc + .5*dz):
            return True
            
    return False
