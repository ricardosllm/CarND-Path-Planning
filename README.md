# CarND-Path-Planning-Project
Self-Driving Car Engineer Nanodegree Program

![alt text](assets/path-planner.mp4.gif "Result")

### Goals
In this project the goal is to safely navigate around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit. We’re provided with the car's localization and sensor fusion data, there is also a sparse map list of waypoints around the highway. The car should try to go as close as possible to the 50 MPH speed limit, which means passing slower traffic when possible, note that other cars will try to change lanes too. The car should avoid hitting other cars at all cost as well as driving inside of the marked road lanes at all times, unless going from one lane to another. The car should be able to make one complete loop around the 6946m highway. Since the car is trying to go 50 MPH, it should take a little over 5 minutes to complete 1 loop. Also the car should not experience total acceleration over 10 m/s^2 and jerk that is greater than 10 m/s^3.

#### The map of the highway is in data/highway_map.txt
Each waypoint in the list contains  [x,y,s,dx,dy] values. x and y are the waypoints map coordinate position, the s value is the distance along the road to get to that waypoint in meters, the dx and dy values define the unit normal vector pointing outward of the highway loop.

The highway's waypoints loop around so the frenet s value, distance along the road, goes from 0 to 6945.554.

---

## The Model 

The model is described in a set of classes, for better code organization:
* **Planner** Describes the actual path planner, includes methods like `update`, `get_path`, `set_path`, and others to set speed, detect vehicles on the lanes, execute a lane change while detecting the optimal path and lane for it.
* **OtherVehicle** Abstracts the position of other vehicles on the road.
* **Map** Abstracts the given map, both in cartesian and in frenet coordinates 
* **Lane** Abstracts the lanes in the road.
* **Helper** Contains helper methods, for example to convert cartesian to frenet coordinates, get the closest and next waypoint, etc.
* **spline** Used to interpolate points between the given waypoints in the map.
* **main** Responsible for the interaction between the model and the provided simulator.

The vehicle is able to follow a given path, either to keep in the lane or execute a lane change.

Regarding **keeping within the lane** the vehicle is able to do so by maintaining the `s` coordinate on the frenet space. It looks for other vehicles on the same lane in front and makes sure they are kept at a `secure distance` which is dependent on speed, the faster it is going the higher the `secure distance` will be, if this is not possible it will reduce speed and look for a possible `lane change` ([ref:Planner.cpp#L18](https://github.com/ricardosllm/CarND-Path-Planning/blob/master/src/Planner.cpp#L181)). 

In order to perform a lane change it looks at the possible `target lanes`, either a single one when on one of the 2 edge lanes, or to both if in the middle lane. When evaluating the target lanes it detects if there are any other vehicles within the `secure distance + 20m` in front and `10m` behind its current position ([ref:Planner.cpp#L157](https://github.com/ricardosllm/CarND-Path-Planning/blob/master/src/Planner.cpp#L157)). 

To accomplish a **lane change** it first calculates a change lane path by calculating the ideal `s` values based on the current and target lanes and the other vehicles in both lanes. It takes into account current `s - 20m` in the current lane as the start and current `s + 40m` in the target lane as the finish, then calculating a spline between the two points to get the change lane trajectory ([ref:Planner.cpp#L303](https://github.com/ricardosllm/CarND-Path-Planning/blob/master/src/Planner.cpp#L303)). 

## Results

The model, although it is able to perform the required `4.32 miles` without incidents, it far from perfect. In a real world scenario with unpredictable vehicle behaviour it would definitely run into accidents. 

---

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./path_planning`.

---

## Simulator.
You can download the Term3 Simulator which contains the Path Planning Project from the [releases tab (https://github.com/udacity/self-driving-car-sim/releases).

Here is the data provided from the Simulator to the C++ Program

### Main car's localization Data (No Noise)

* ["x"] The car's x position in map coordinates
* ["y"] The car's y position in map coordinates
* ["s"] The car's s position in frenet coordinates
* ["d"] The car's d position in frenet coordinates
* ["yaw"] The car's yaw angle in the map
* ["speed"] The car's speed in MPH

### Previous path data given to the Planner

* ["previous_path_x"] The previous list of x points previously given to the simulator
* ["previous_path_y"] The previous list of y points previously given to the simulator

### Previous path's end s and d values 

* ["end_path_s"] The previous list's last point's frenet s value
* ["end_path_d"] The previous list's last point's frenet d value

### Sensor Fusion Data, a list of all other car's attributes on the same side of the road. (No Noise)

* ["sensor_fusion"] A 2d vector of cars and then that car's 
  * car's unique ID, 
  * car's x position in map coordinates, 
  * car's y position in map coordinates, 
  * car's x velocity in m/s, 
  * car's y velocity in m/s, 
  * car's s position in frenet coordinates, 
  * car's d position in frenet coordinates. 

### Details

1. The car uses a perfect controller and will visit every (x,y) point it receives in the list every .02 seconds. The units for the (x,y) points are in meters and the spacing of the points determines the speed of the car. The vector going from a point to the next point in the list dictates the angle of the car. Acceleration both in the tangential and normal directions is measured along with the jerk, the rate of change of total Acceleration. The (x,y) point paths that the planner receives should not have a total acceleration that goes over 10 m/s^2, also the jerk should not go over 50 m/s^3. (NOTE: As this is BETA, these requirements might change. Also currently jerk is over a .02 second interval, it would probably be better to average total acceleration over 1 second and measure jerk from that.

2. There will be some latency between the simulator running and the path planner returning a path, with optimized code usually its not very long maybe just 1-3 time steps. During this delay the simulator will continue using points that it was last given, because of this its a good idea to store the last points you have used so you can have a smooth transition. previous_path_x, and previous_path_y can be helpful for this transition since they show the last points given to the simulator controller with the processed points already removed. You would either return a path that extends this previous path or make sure to create a new path that has a smooth transition with this last path.

---

## Dependencies

* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Run either `install-mac.sh` or `install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets 
    cd uWebSockets
    git checkout e94b6e1
    ```

## Code Style

Please (do your best to) stick to [Google's C++ style guide](https://google.github.io/styleguide/cppguide.html).


