# CarND-Path-Planning-Project

## Introduction

The scope of the project is to autonomously drive the car around the highway track. The track involves an highway with 3 lanes on each side. The simulator provides sensor fusion data about all the other car objects around the own vehicle. The project involves in the car driving at a reasonable speed but not exceeding the speed limits or not too slow unless obstructed by traffic around.  The car has to make necessary lane changes as needed in a safe manner. The total accelration must not exceed 10 m/s2 and total jerk cannot exceed 10 m/s3. The other key criterias for the car to follow are to not to cause collision with any other car in the highway and not to spend more than 3 seconds outside the lanes or on the lane markings.

## Implementation Steps

The implementation of path planning logic is done in the following steps.

- Create the waypoints of the near by area from the map data
- Determine the ego vehicle parameters and predict the path of the car.
- Based on the sensor fusion data predict the path of the other cars on the road.
- Determine the best possible trajectory for the ego car.
- Generate the optimum trajectory and feed it to the simulator.

### Generation of nearby waypoints

The waypoints for the entire track is included as a map data in the ```highway_map.csv``` file. The waypoints are 30 meters apart. The waypoints are middle of the yellow line in the center of the highway. There are a total of 181 waypoints. The total distance in frenet co-ordinate system is 6945.554 meters or 4.32 miles. Each lane is 4 meter wide and the frenet d co-ordinates are based from the center of the yellow lines.  The map data is represented as global x, global y, frenet s, frenet dx and frenet dy. The interpolated data is created with 0.5 meters apart and 8 waypoints ahead and behind the ego vehicle. The ```getFrenet``` method is used to compensate for the begining and end of the track.

### Determine Ego vehicle parameters and generation of Vehicle object

The telemetry data for the ego vehicle is sent by the simulator instaneously. The data also contains the portion of the previuosly generated path that was used by the car to travel in the highway. This data is used to predict the future state of the car and planning for the further states is calculated.  By using the previous path along with the current path, a smooth transition for the car can be achieved.
The Ego vehicle has three plaussible states.
- Keep Lane
- Lane Change Left
- Lane Change Right


### Other vehicle path based on Sensor Fusion data

The sensor fusion data for the cars are sent by the simulator for each cycle. This data contains the following information about each car. The data consists of Vehicle ID, Global X, Global Y, Global vx, (m/s), Global vy, (m/s), Frenet s and Frenet d. This data is extraced and trajectories for each car is generated based on the sensor fusion data. The trajectories of the ego car data and other vehicle data is used with a set of cost functions based on the project scope to determine the optimised state for the vehicle.

### Determine optimised trajectory for ego car

The optimised trajectory is generated based on 5 different cost penalties and weights.
- collision cost - This cost penalises the trajectory that collides with other cars
- buffer cost - This cost penalises the trajectory that is beyond a certain allowable distance any other cars.
- own lane buffer cost - This cost is to penalise if the following distance to the vehicle in front is not reached by the trajectory.
- efficiency cost - This cost focuses on the trajectories that are not meeting the target velocity.
- middle lane cost - The middle lane is an optimal lane for the ego vehicle so it always has multiple state options. Hence a cost to penalise the trajectory if the lane is not middle lane.

A quintic polynomial, jerk minimising trajectory is calcuated for each available state. The final trajectory is based on the lowest of the total cost of each possible trajectory.

### Generation of new path

The new path is based on the combination of points from the previous path and optimium path received from the calcuations. The spline for the potential new path is generated based on 3 new points 30, 60 and 90 meters ahead. These s and d co-odrinates are used to produce a smooth spline. The number of points in the spline and veoclity increment controls the execssive acceleration or jerk limits.

### Conclusion

The optimised trajectory generator work well in most of the situation. Its is able to navigate the highway incident free for 10 to 12 miles based on several tests. The maximum incident free run that was been able to achieve was xx miles. 
   
### Simulator.
You can download the Term3 Simulator which contains the Path Planning Project from the [releases tab (https://github.com/udacity/self-driving-car-sim/releases).

#### The map of the highway is in data/highway_map.txt
Each waypoint in the list contains  [x,y,s,dx,dy] values. x and y are the waypoint's map coordinate position, the s value is the distance along the road to get to that waypoint in meters, the dx and dy values define the unit normal vector pointing outward of the highway loop.

The highway's waypoints loop around so the frenet s value, distance along the road, goes from 0 to 6945.554.

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./path_planning`.

Here is the data provided from the Simulator to the C++ Program

#### Main car's localization Data (No Noise)

["x"] The car's x position in map coordinates

["y"] The car's y position in map coordinates

["s"] The car's s position in frenet coordinates

["d"] The car's d position in frenet coordinates

["yaw"] The car's yaw angle in the map

["speed"] The car's speed in MPH

#### Previous path data given to the Planner

//Note: Return the previous list but with processed points removed, can be a nice tool to show how far along
the path has processed since last time. 

["previous_path_x"] The previous list of x points previously given to the simulator

["previous_path_y"] The previous list of y points previously given to the simulator

#### Previous path's end s and d values 

["end_path_s"] The previous list's last point's frenet s value

["end_path_d"] The previous list's last point's frenet d value

#### Sensor Fusion Data, a list of all other car's attributes on the same side of the road. (No Noise)

["sensor_fusion"] A 2d vector of cars and then that car's [car's unique ID, car's x position in map coordinates, car's y position in map coordinates, car's x velocity in m/s, car's y velocity in m/s, car's s position in frenet coordinates, car's d position in frenet coordinates. 

## Details

1. The car uses a perfect controller and will visit every (x,y) point it recieves in the list every .02 seconds. The units for the (x,y) points are in meters and the spacing of the points determines the speed of the car. The vector going from a point to the next point in the list dictates the angle of the car. Acceleration both in the tangential and normal directions is measured along with the jerk, the rate of change of total Acceleration. The (x,y) point paths that the planner recieves should not have a total acceleration that goes over 10 m/s^2, also the jerk should not go over 50 m/s^3. (NOTE: As this is BETA, these requirements might change. Also currently jerk is over a .02 second interval, it would probably be better to average total acceleration over 1 second and measure jerk from that.

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

## Editor Settings

We've purposefully kept editor configuration files out of this repo in order to
keep it as simple and environment agnostic as possible. However, we recommend
using the following settings:

* indent using spaces
* set tab width to 2 spaces (keeps the matrices in source code aligned)

## Code Style

Please (do your best to) stick to [Google's C++ style guide](https://google.github.io/styleguide/cppguide.html).
