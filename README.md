# 3DsailSim
Author Info: 

* Jesse Miller, jam643@cornell.edu, (845) 428-1532
* Please contact me with any questions, concerns, or recommendations

## Introduction
![](sailSim.gif)
[Video Demonstration](https://www.youtube.com/watch?v=b1fbiitYMzA)

SailSim is a 3D dynamic simulation of a sailboat. For more information on
dynamic model, read the PDF paper in this folder. Sailsim allows the user
to control the sail and rudder angles in real time using the arrow keys.
Furthermore, the user can quickly modify the size, weight, and location
of various parts of the boat, such as the rudder, to see its effect on the
boat's performance. This simulation was originally designed to test the
directional stability of a 1 meter long sailboat by varying the location
of the rudder. We found that attaching the rudder to the back of the sail
improved the directional stability of the sailboat. The results of this 
simulation (namely the placement of the rudder behind the sail and the
optimal dimensions of the sail, keel, and rudder) were used to build a
functional autonomous sailboat by the CUSail team at Cornell University.

Some of the modelling assumptions used in this simulation include:
* No waves in the water
* Uniform wind velocity
* The water is stagnant with no currents
* Wind does not have an effect on the hull of the boat
* The hull is assumed to be 1 meter in length. This is because the hull 
  resistance is calculated based on experimental data of a 1 meter long
  hull.
* The cross-sectional shape of the hull is assumed circular meaning that 
  heeling stability is due to the location of the center of mass relative
  to the center of buoyancy rather than the shape of the hull.
* The sail and tail are currently assumed to both experience the free 
  stream relative wind speeds. However, in reality, the flow upstream of 
  the tail is disturbed by the sail. This induced flow could have a 
  significant effect on the performance of the tail.

## Running The Simulation
* Run makeBoat.m:
  The simulation requires a 'boat file' that specifies the geometry of the
  boat design. This .mat file is created by running the makeBoat.m function
  with the desired file name as an argument. First, modify the makeBoat.m
  with the desired parameters (such as the size and placement of the sail,
  keel, and rudder). Then run this function by typing in the command
  window, for example:
  makeBoat('sailboat1')
  Which will save a .mat file 'sailboat1.m' that can then be used in
  simulation. Three premade boat files are included as examples. Each boat
  has a different type of rudder.
* Modify setBoatParam.m:
  Before running the simulation, the simulation options can be changed in
  the setBoatParam.m function. Simply open up this function, change the
  desired options, and save this function before running the simulation. 
  The options that can be modified include; initial conditions of the boat,
  wind speed, parameters effecting the accuracy of the simulation, etc.
* Run main.m:
  Running the main.m function with the desired .mat boat file runs the
  simulation. For instance, if you have created a sailboat1.mat file using
  makeBoat.m, you can run the simulation by entering main('sailboat1') in
  the command window.
* Use the left/right and up/down arrow keys to control the sail and rudder
  respectively. Use the 'ASDW' keys to control the perspective. Try
  various sailboat maneuvers such as tacking and jibing. How fast can you
  get the boat to move in various directions? Can you get the sailboat to
  sail along a straight trajectory without actuating the sail/rudder? How
  does changing the size/position of the sail, keel, and rudder affect it's
  performance? Have fun!

## How You Can Improve
* Improving the accuracy of the simulation:
  The assumptions listed above were made for convenience but may still have
  an impact on the stability/performance of the boat. For instance, adding
  waves, water currents, and wind vector fields could create a more 
  realistic simulation of the actual ocean.
* Adding functionality:  
  You could add path planning and feedback controls to simulate an
  autonomous boat.
* Awesome final goal:
  Putting this all together, you could simulate autonomously sailing a
  course across the ocean only actuating the motors occasionally to
  simulate the effect of limited solar energy.

Additional zip files are included in this folder:

* sailboatSimulation2D.zip is a 2D predecessor to this simulation I wrote
  during my first semester on the team.
* 3DsailSim_old.zip is an older version of the 3D sailboat simulation with
  a 2D approximation of the 3D dynamics (less realistic).
* optimizeSailboat.zip optimizes the size of the sail/keel/rudder. This
  optimization is discussed in my MEng report PDF. However, the code is
  poorly documented. For more info, contact me and I will write a README
  for the code and better explain it.
