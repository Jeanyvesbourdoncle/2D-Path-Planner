## Path-Planning 2D
-----------------------------
   
### Goals 
The goal of this project is to build a path planner to have a vehicle, which can navigate around a virtual highway with other vehicles, all going different speeds (with the maximum speed : 50 MPH).
The Path Planner must create a smooth, safe path for the car to follow along a 3 lane highway. 
The implementation will be done in C++.

-----------------------------
### Input Data 
The simulator provides :
- the car's localization,
- the sensor fusion data with the other car's position and velocity. 

The map of the highway is in ./data/highway_map.txt. 
Each waypoint has an (x,y) global map position, and a Frenet 's' value and Frenet 'd' unit normal vector (split up into the x component, and the y component). 
The 's' value is the distance along the direction of the road.
The 'd' vector can be used to calculate lane positions. 
The highway has 6 lanes total - 3 heading in each direction. Each lane is 4 m wide.

------------------------------
### Requirements
Our restrictions are the following :
- the car must go as close as possible to the 50 MPH speed limit, which means passing slower traffic when possible,
- the car should avoid hitting other cars,
- the car should not experience total acceleration over 10 m/s^2 and jerk that is greater than 10 m/s^3.

#------------------------------
### Basic Definition
##### Main car's localization Data
The car transmits its location. 
["x"] The car's x position in map coordinates
["y"] The car's y position in map coordinates
["s"] The car's s position in frenet coordinates
["d"] The car's d position in frenet coordinates
["yaw"] The car's yaw angle in the map
["speed"] The car's speed in MPH

###### Previous path data given to the Planner
["previous_path_x"] The previous list of x points previously given to the simulator
["previous_path_y"] The previous list of y points previously given to the simulator

###### Previous path's end s and d values 
["end_path_s"] The previous list's last point's frenet s value
["end_path_d"] The previous list's last point's frenet d value

###### Data sent to the Simulator
The C++ program must compute and send to the simulator a list of x and y global map coordinates. 
Each pair of x and y coordinates is a point, and all of the points together form a trajectory. 
The car moves to a new waypoint every 20ms.

---------------------------------
### PIPELINE 
This picture resume the SW pipeline :
<p align="center">
<img src="./Stages_Behavior_Control.png" width="50%>
</p> 


#### Step 1 : Sensor Data Fusion
The "sensor fusion" variable contains all the estimation information about the cars on the right-hand side of the road.
The data format for each car is [id,x,y,vx,vy,s,d] :
- ["ID"] : car's unique identifier, 
- ["x"] : car's position in global map coordinates, 
- ["y"] : car's position in global map coordinates, 
- ["vx"] : car's velocity in m/s in global map coordinates, 
- ["vy"] : car's velocity in m/s in global map coordinates, 
- ["s"] : car's position in local Frenet coordinates,
- ["d"] : car's position in local Frenet coordinates. 

The vx,vy values are useful for the prediction where the cars will be in the future.
The future predicted Frenet 's' value will be its current 's' value plus its transformed total velocity(m/s) multiplied by the time elapsed into the future(s). 


#### Step 2 : Behavior Control
The "behavior control" is composed about 3 steps : 
Step 1 : ACC = Adaptive Cruise Control : control the safety distance between the current vehicle and the vehicle front, 
Step 2 : Finite State Machine : Keep Lane State/ Prepare change Lane Right / Prepare Change Lane left, depends of the position of the vehicle on the different lanes, 
Step 3 : Lane Changer accepted or not (depends of the result of the step 2).

<p align="center">
<img src="./Machine_Etat_Fini.png" width="50%>
</p> 


#### Step 3 : Spline Creation
The target is to create paths to smoothly change lanes.
It's the creation of 5 points that the spline will use to generate the function that will connect such point in a smooth trajectory.
For the creation of a smooth trajectories, the spline tool is : http://kluge.in-chemnitz.de/opensource/spline/
The spline fitting guarantees that the generated function passes through every point.
The acceleration and jerk part are solved from linear equations for 's' and 'd' functions with the provided Eigen 3-3 library.
The last part is the function to transform the (s,d) points to (x,y) points for the returned path.


#### Step 4 : Waypoint Planner
The target is to create each of the individuals points in x that the car fill follow feed them into the spline function to look for the corresponding y.
The first waypoint from the previous path are used and the rest of the new path based on the new data from the car's sensor fusion.       

------------------------------------
### Details
There will be some latency between the simulator running and the path planner returning a path with optimized code usually its not very long maybe just 1-3 time steps. 
During this delay the simulator will continue using points that it was last given, its a good idea to store the last points you have used so you can have a smooth transition. 
[Previous_path_x] and [Previous_path_y] are helpful for this transition since they show the last points given to the simulator controller with the processed points already removed. 
You would either return a path that extends this previous path or make sure to create a new path that has a smooth transition with this last path.

-----------------
### Ressources
For the creation of a smooth trajectories (Step : Spline Creation) : http://kluge.in-chemnitz.de/opensource/spline/

------------------------------
### Basic Build Instructions
1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./path_planning`.


