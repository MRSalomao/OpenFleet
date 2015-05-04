#!/usr/bin/env python
import numpy as np
import Planner
import Objects
import Graph
import math

PI = math.pi

def main():
  # SmallSat Blondie
  blondie = Objects.SmallSat()
  blondie.Initial_State[0:3] = np.zeros(3) + np.array([1,1,1])
  blondie.Initial_State[3] = 1
  blondie.Initial_State[10:13] = np.zeros(3) + .25*PI
  
  blondie.Final_State[0:3] = np.array([7, 8, 8])
  blondie.Final_State[3] = 1
  
  # SmallSat Tuco
  tuco = Objects.SmallSat()
  tuco.Initial_State[0:3] = np.zeros(3) + np.array([3,3,3])
  tuco.Initial_State[3] = 1
  tuco.Initial_State[10:13] = np.zeros(3) + .25*PI
  
  tuco.Final_State[0:3] = np.array([8, 8, 7])
  tuco.Final_State[3] = 1
  
  initial_fleet = np.vstack([blondie.Initial_State, tuco.Initial_State])
  final_fleet = np.vstack([blondie.Final_State, tuco.Final_State])
  
  # ISS is represented by a list of bounding boxes
  ISS = []
  # Read Bounding Boxes TXT file
  with open('./AABB-ISS.txt') as f:
    for line in f:
      # Place ISS at (5,5,5)
      a = Objects.BoundingBox( line, [5,5,5] )
      ISS.append(a)
  
  '''
   Execute the Planner.
   Paramenter:
     N = Number of solution trajectories
     ISS = Bounding Boxes
  '''
  N = 10
  Bid_RRT = Planner.Planner(N,ISS)
  path = Bid_RRT.Fleet_Simple(initial_fleet, final_fleet, '2WAY')
  
  Graph.plot_fleet(path)
  # Generate a movie
  # Graph.animate_fleet(path, './Movie.mp4')
  
  print "Done!"
  
if __name__ == "__main__":
  main()