# Model Predictive Control (MPC)

[![Udacity - Self-Driving Car NanoDegree](https://s3.amazonaws.com/udacity-sdc/github/shield-carnd.svg)](http://www.udacity.com/drive)

<img src="imgs/output.gif" width="480" alt="Output" />

## Overview
This project implements a non-linear model predictive controller to control the steering angle and throttle of a self driving car as it goes around the race track.

## Dependencies
* [Simulator](https://github.com/udacity/self-driving-car-sim/releases)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
* cmake
* gcc/g++
* make
* openssl
* libuv
* zlib
* Ipopt - non-linear optimization library
```
brew tap udacity/CarND-MPC-Project https://github.com/udacity/CarND-MPC-Project
brew install ipopt --with-openblas
```

* CppAD - library for automatic differentiation
```
brew install cppad
```

## Build Instructions
1. Clone this repo.
2. Make a build directory: `mkdir build && cd $_`
3. Compile: `cmake .. && make`

## Run
* To run `./mpc`

---

## Details

### Preprocessing
The waypoints are preprocessed by transforming them to the vehicle's coordinate frame. This simplifies the process to fit a polynomial to the waypoints because the vehicle's x and y coordinates are now at the origin (0, 0) and the orientation angle is also zero.

### The model
The kinematic model includes the vehicle's x and y coordinates, orientation angle (psi), and velocity, as well as the cross-track error and psi error (epsi).
Actuator outputs are acceleration and delta (steering angle). The model combines the state and actuations from the previous timestep to calculate the state for the current timestep based on the equations below:
<img src="imgs/equations.png" width="480" alt="Output" />

### Cost functions
The following cost functions are used for optimal behaviour:
* Cross track error cost to minimize distance for trajectory
* PSI error cost to minimize orientation error
* Velocity error cost to get the car to move at desired velocity
* Actuation costs for steering angle and acceleration
* Actuation smoothness costs for steering and acceleration rates
* Cost of product of steering angle and velocity to allow for smooth turns 
