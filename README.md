# CarND-Controls-MPC (Model Predictive Control)
Self-Driving Car Engineer Nanodegree Program
---

## Introduction

The goal of this project is to develop/implement a Model Predictive Controller (MPC) in C++ that allows a virtual vehicle to be driven in a Unity simulation environment. This MPC utilizes the Global Kinematic Model as the framework to determine the state of the vehicle at all times. All model parameters were tuned manually by trial and error.

## The Model

A simplified Global Kinematic Model was utilized to predict the state of the vehicle.

Vehicle Position: (x,y)
Vehicle Heading: (ψ)
Velocity (v)

Vehicle State example: [x, y, ψ, v]

Only two actuators were necessary to control the vehicle.

* Steering Wheel Angle
Denoted by "δ", limits were set to allow for a minimum angle of -25° and a maximum angle of 25°

* throttle
Denoted by "a", limits were set to allow for a maximum negative acceleration (braking) of -1.0
and a maximum positive acceleration of 1.0

This simplified model does a good job of simulating vehicle dynamics but does not take into account road surface interactions(tires), gravity, inertia, throttle response, vehicle drag coefficient, etc.

This kinematic model is able to predict the vehicle's state at the next time step based on the vehicle's current state and actuator inputs. The following first four equations are used to predict the vehicle's future state. (Lf is the distance from the front of the vehicle to its center of gravity and was provided by Udacity as part of the starter code for this project.)

![Kinematic Model Equations](MPC_equations.png)

The final two equations, cross track error (cte) and error psi (epsi), were used to compute the cost of the vehicle moving away from the center of the track. As previously stated, these equations were updated at each time step in order to predict the future cte and epsi of the vehicle's current trajectory.   

## Timestep Length (N) & Elapsed Duration between timesteps (dt)

### Final Parameter Values:
* Timestep Length (N) = 10
* Elapsed Duration (dt) = 0.1

These values were initially chosen after watching Udacity's MPC Project Q&A webinar. The other values that were also tested are listed below. I noticed that as the timestep increased the vehicle moved more and more erratically. I believe this is primarily a hardware limitation due to my small laptop with 1GB of RAM and an Atom processor. If I had more RAM and a faster processor then I believe I could set a larger timestep which would also allow for a higher reference velocity.  

* Additional Test parameters
N = 8, 15, 18, 20, 25
dt = 0.08, 0.15, 0.1, 0.2, 0.2

##  MPC Preprocessing / Polynomial Fitting / MPC with 100ms Latency

First, the N waypoint coordinates were converted into the vehicle coordinate space with the vehicle located at the origin and a heading angle (ψ) of 0°. This provided a clean way to orient the waypoints in relation to the vehicle's state. Then, a 3rd order polynomial was fit to the N waypoint coordinates to predict the future path of the vehicle including the 100ms latency which was built into the kinematic vehicle model in main.cpp.

Latency was introduced into the project to simulate the delay of a human or automated driver and by building the latency into the model it makes for a more robust and stable model.  

The cost function reference and weight parameters were tuned using trial and error and the final values are written below as well as stored in the header file MPC.h.

```cpp
REF_CTE 0.0
REF_EPSI 0.0
REF_V 45.0 // mph

CTE_WEIGHT 1.8
EPSI_WEIGHT 21.0
V_WEIGHT 1.0
DELTA_WEIGHT 1.0e5
A_WEIGHT 15
DDELTA_WEIGHT 0.0
DA_WEIGHT 0.0
```
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
    Some function signatures have changed in v0.14.x. See [this PR](https://github.com/udacity/CarND-MPC-Project/pull/3) for more details.
* Fortran Compiler
  * Mac: `brew install gcc` (might not be required)
  * Linux: `sudo apt-get install gfortran`. Additionall you have also have to install gcc and g++, `sudo apt-get install gcc g++`. Look in [this Dockerfile](https://github.com/udacity/CarND-MPC-Quizzes/blob/master/Dockerfile) for more info.
* [Ipopt](https://projects.coin-or.org/Ipopt)
  * Mac: `brew install ipopt`
  * Linux
    * You will need a version of Ipopt 3.12.1 or higher. The version available through `apt-get` is 3.11.x. If you can get that version to work great but if not there's a script `install_ipopt.sh` that will install Ipopt. You just need to download the source from the Ipopt [releases page](https://www.coin-or.org/download/source/Ipopt/) or the [Github releases](https://github.com/coin-or/Ipopt/releases) page.
    * Then call `install_ipopt.sh` with the source directory as the first argument, ex: `bash install_ipopt.sh Ipopt-3.12.1`.
  * Windows: TODO. If you can use the Linux subsystem and follow the Linux instructions.
* [CppAD](https://www.coin-or.org/CppAD/)
  * Mac: `brew install cppad`
  * Linux `sudo apt-get install cppad` or equivalent.
  * Windows: TODO. If you can use the Linux subsystem and follow the Linux instructions.
* [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page). This is already part of the repo so you shouldn't have to worry about it.
* Simulator. You can download these from the [releases tab](https://github.com/udacity/self-driving-car-sim/releases).
* Not a dependency but read the [DATA.md](./DATA.md) for a description of the data sent back from the simulator.


## Basic Build Instructions


1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./mpc`.

## Tips

1. It's recommended to test the MPC on basic examples to see if your implementation behaves as desired. One possible example
is the vehicle starting offset of a straight line (reference). If the MPC implementation is correct, after some number of timesteps
(not too many) it should find and track the reference line.
2. The `lake_track_waypoints.csv` file has the waypoints of the lake track. You could use this to fit polynomials and points and see of how well your model tracks curve. NOTE: This file might be not completely in sync with the simulator so your solution should NOT depend on it.
3. For visualization this C++ [matplotlib wrapper](https://github.com/lava/matplotlib-cpp) could be helpful.
