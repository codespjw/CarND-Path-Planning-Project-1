# CarND-Path-Planning-Project
Self-Driving Car Engineer Nanodegree Program
   
## Simulator
You can download the Term3 Simulator which contains the Path Planning Project from the [releases tab](https://github.com/udacity/self-driving-car-sim/releases).

## Project Description & Requirements
Please refer to the [PROJECT.md](./project.md) for details.

## Implementation Overview

- The implementation is within the [single file](https://github.com/dolaameng/CarND-Path-Planning-Project/blob/master/src/main.cpp) for the minimum change from project template and organization. I found it acceptable because the code is still relatively short.

- The main components and their relations are implemented as the following several classes
  - [`SelfDrivingCar`](https://github.com/dolaameng/CarND-Path-Planning-Project/blob/master/src/main.cpp#L58): the state of SDC, including speed, position and etc.
  - [`PeerCar`](https://github.com/dolaameng/CarND-Path-Planning-Project/blob/master/src/main.cpp#L79): the state of other cars on the same side of road (peer cars).
  - [`Map`](https://github.com/dolaameng/CarND-Path-Planning-Project/blob/master/src/main.cpp#L96): it encapusulates the waypoint information into smooth trajectories, which are mappings from coordinates from car's view (a.k.a Frenet coordinates) to map view (x, y). The trajectory mapping is implemented by using the third-party [`spline` library](https://github.com/dolaameng/CarND-Path-Planning-Project/blob/master/src/spline.h).
  - [`Path`](https://github.com/dolaameng/CarND-Path-Planning-Project/blob/master/src/main.cpp#L221): an encapsulation of a set of steps in (x, y) map coordinates. This is the main protocol for the planner to pass the commands to the actual controller.
  - [`PathPlanner`](https://github.com/dolaameng/CarND-Path-Planning-Project/blob/master/src/main.cpp#L233): the main class for path planning. 

- The main methods implemented in `PathPlanner` class are,
  - [`plan()`](https://github.com/dolaameng/CarND-Path-Planning-Project/blob/master/src/main.cpp#L241): it is the main access point of the planner. It uses the self-driving-car and other cars information, combined with previous planned path (for smoothness), to plan for the next path. In general it estimates how the car should behaves at the next step, by considering a strategy for different implemented behaviors. Currently the strategy is implemented as [a set of rules](https://github.com/dolaameng/CarND-Path-Planning-Project/blob/master/src/main.cpp#L260). It might be implemented by other search algorithms e.g. `a-star` for more complicated cases.
  - [`keep_lane()`](https://github.com/dolaameng/CarND-Path-Planning-Project/blob/master/src/main.cpp#L345): it implements the car's staying in the current lane behavior
  - [`change_lane()`](https://github.com/dolaameng/CarND-Path-Planning-Project/blob/master/src/main.cpp#L419): the other behavior of the car currently implemented.
  - [`accelerate()`](https://github.com/dolaameng/CarND-Path-Planning-Project/blob/master/src/main.cpp#L516): it controls car's speed following the acceleration and jerk requirements. 

## Project Rubics Walk-through

### 1. Build and Compilation
The project compiles and builds by following,
1. Make a build directory: `mkdir build && cd build`
2. Compile: `cmake .. && make`
3. Run it: `./path_planning`.

### 2. The car is able to drive > 4.32 miles without incident. 
It requires the following cases in details.

#### 2.1 The car drives according to the speed limit, which is `50+/-10 MPH`
In the implementation, the car is setting its target speed dynamically according to its environment, e.g.,

- The maximum speed of the car is set as [a parameter of `PathPlanner`](https://github.com/dolaameng/CarND-Path-Planning-Project/blob/master/src/main.cpp#L331), which is `20 m/s` ~ `44.7 MPH`.
- The car dynamically adjust its target speed based on the car on the target lane ahead, e.g., when the car is [keeping on its current lane](https://github.com/dolaameng/CarND-Path-Planning-Project/blob/master/src/main.cpp#L371), or [changing to another lane](https://github.com/dolaameng/CarND-Path-Planning-Project/blob/master/src/main.cpp#L491).

#### 2.2 Max acceleration and jerk are not exceeded
This is implemented in the [accelerate method of planner](https://github.com/dolaameng/CarND-Path-Planning-Project/blob/master/src/main.cpp#L516).

In general it considers the difference between current and target and calculates the acceleration by not violating the constraints.

#### 2.3 Car does not have collisions
Collisions are avoided by,
- Being safe about the speed and distance estimate, e.g., [assuming the car in front could brake suddenly](https://github.com/dolaameng/CarND-Path-Planning-Project/blob/master/src/main.cpp#L365).
- Giving enough time for lane changing, e.g., always [checking if a lane-changing is safe](https://github.com/dolaameng/CarND-Path-Planning-Project/blob/master/src/main.cpp#L524).

#### 2.4 The car stays in its lane, except for the time between changing lanes
This is done by [building a smooth trajectory mapping using the waypoints](https://github.com/dolaameng/CarND-Path-Planning-Project/blob/master/src/main.cpp#L119). I am currently implementing this as a global trajectory for the whole map. But in practice it might make more sense to build local trajectory around the car for more complicated cases.

#### 2.5 The car is able to change lanes
The lane-chaning is implememted in two parts,
- In the [`plan()` method](https://github.com/dolaameng/CarND-Path-Planning-Project/blob/master/src/main.cpp#L288), a set of rules is implemented to decide whether a lane-changing is feasible when necessary.
- The actual path generation is done in the [`change_lane()` method](https://github.com/dolaameng/CarND-Path-Planning-Project/blob/master/src/main.cpp#L419), which is one of the car's preset behaviors. 

### 3. There is reflection on how to generate paths
- As discussed above, the actual path generating is delegated to the different "behavior" methods of the planner, including `keep_lane` and `change_lane`.  
- For [`keep_lane`](https://github.com/dolaameng/CarND-Path-Planning-Project/blob/master/src/main.cpp#L345), two scenarios are considered,
  - [when the lane is clear](https://github.com/dolaameng/CarND-Path-Planning-Project/blob/master/src/main.cpp#L361), the car will set its target speed as maximum and come up with a smooth trajectory both spatially and temporally.
  - [when there is a car ahead](https://github.com/dolaameng/CarND-Path-Planning-Project/blob/master/src/main.cpp#L371), the car will set its safe distance from the car ahead and set its target speed accordingly. If the speed on this lane is too slow, the car will try to plan to [change to another lane](https://github.com/dolaameng/CarND-Path-Planning-Project/blob/master/src/main.cpp#L288) by evaluating their feasibility and advantage over current lane.
- For [`change_lane`](https://github.com/dolaameng/CarND-Path-Planning-Project/blob/master/src/main.cpp#L419),
  - the car will first evaluate whether it is [safe to change to the target lane](https://github.com/dolaameng/CarND-Path-Planning-Project/blob/master/src/main.cpp#L431). If not, it wills stay on the same lane.
  - otherwise, the car will plan a smooth local trajectory by [combining](https://github.com/dolaameng/CarND-Path-Planning-Project/blob/master/src/main.cpp#L436) half from the source lane and the other half from the target lane.
  - at the meantime, since changing lane may take longer than one plan cycle to finish, the planner will [monitor the lane changing state](https://github.com/dolaameng/CarND-Path-Planning-Project/blob/master/src/main.cpp#L250) to avoid interruption.

### 4. Things that can be improved
- Since the car's Frenet will be reset at the end of loop, I had some difficulties with modelling the end part of the loop. I fixed it by [looping the map again](https://github.com/dolaameng/CarND-Path-Planning-Project/blob/master/src/main.cpp#L131). This solves the issue for the current simulation, but a better solution might be just using a ___local___ trajectory model along with the travelling.
- The current lane_changing is implemented by planning [a longer path](https://github.com/dolaameng/CarND-Path-Planning-Project/blob/master/src/main.cpp#L493) (6 times longer in fact). And during the lane changing [there won't be any further planning until it is finished](https://github.com/dolaameng/CarND-Path-Planning-Project/blob/master/src/main.cpp#L250). Safety is considered by employing [strict rules](https://github.com/dolaameng/CarND-Path-Planning-Project/blob/master/src/main.cpp#L527) on whether it is safe to change. This does make the lane-changing plan more conservative in my implementation. A better way might be continuing planning even during the lane-changing, e.g., going back to the previous lane in case of emergency. 


## Result
[![MPC Simulation](https://img.youtube.com/vi/kXGTQ4S4DVI/0.jpg)](https://www.youtube.com/watch?v=kXGTQ4S4DVI)
