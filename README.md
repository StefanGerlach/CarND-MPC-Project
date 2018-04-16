# CarND-Controls-MPC
Self-Driving Car Engineer Nanodegree Program
[![Udacity - Self-Driving Car NanoDegree](https://s3.amazonaws.com/udacity-sdc/github/shield-carnd.svg)](http://www.udacity.com/drive)

[//]: # (Image References)

[image1]: ./documentation/YT_Link.png "YT Link"
[image2]: ./documentation/polynomial_Lession19.png "Poly"
[image3]: ./documentation/MPC_Overview_Lession20.png "Kinetmatic Model"
[image4]: ./documentation/Smooth_Steer_Lession20.png "Smooth Steering"

[![YTLinkThumbnail][image1]](https://youtu.be/jsNbhTJOigY "Video Title")
(Click on this image to open Youtube video.)


Overview
---

This repository contains my solution of the Udacity Self Driving Car Nanodegree Model Predictive Control Project. The target of this project is to create a Model Predictive Control module that smoothly can drive the tracks in the simulator application. 

To solve this problem, the simulator sends the waypoints (which are the simulated output of the path planning module) of the track to the C++ programm, which are then preprocessed and fed into the MPC module to control the vehicles actuators 'steer' and 'throttle'.

In the video, the ground truth waypoints are displayed as a yellow line, whereas the predicted trajectory of the MPC is painted as the green line. 


Programm and Processing Pipeline
---

Since the Udacity Self Driving Car Nanodegree Simulator is sending the grund truth waypoints to follow, it includes the vehicle perception, localization and path planning in this project. The main part of the C++ programm is to control the vehicles actuators 'steering' and 'throttle'. 

These controls are being constrained within the range of [-25 °, +25 °] for the steering angle and [-1, +1] for the throttle control (where +1 means full acceleration and -1 full brake).

To control the vehicle based on the simulator given waypoints the following steps are done:


### Waypoint Preprocessing

Since the waypoints are in the simulator map coordinate system, these points are first of all transformed into the car coordinate system. To do this, a homogenous transformation in 2D is applied. A good tutorial for this was mentioned in the course material: [Coordinate Transformations](https://www.miniphysics.com/coordinate-transformation-under-rotation.html). 

### Polynonmial Fitting 

After transformation of the waypoints, a polinomial of 3rd order is fitted into them. The given function polyfit(), which is adapted from [this source](https://github.com/JuliaMath/Polynomials.jl/blob/master/src/Polynomials.jl#L676-L716), is used for this.

![PolyThumbnail][image2]
(Image from Udacity Self Driving Car Nanodegree course material lession 19)

### State definition

The current state of the vehicle is described by the current velocity, the current cross track error and the current orientation error. The simulator is sending the current velocity via 'speed' entry, so only the cross track error (CTE) and orientation error have to be computed. 

The CTE is simply computed by evaluating the polynomial coefficients at x position 0. The orientation error is computed using the given formula of the course material. 

### Kinematic Model definition 

The Model Predictive Control uses a kinetmatic model to predict where the vehicle will move in 'future' time steps. This model is a really rough approximate of the reality and thus evaluated again for every timestep. The following picture shows, how the state, which means: position, the steering angle psi, the CTE and orientation error, will be updated with respect to delta t. The actuator variables delta (for steering angle) and a for acceleration are constrained within a specific interval. 

![ModelThumbnail][image3]
(Image from Udacity Self Driving Car Nanodegree course material lession 19)

### Model Predictive Control step with Parameter Optimization

For the future time steps (horizon) is use N=10 with delta_t=0.1, since I wanted the algorithm to run very fast and the motion model is quite rough. It is always desirable to have a large N with small delta_t to 'look' far (some seconds) with a high resolution (small delta_t) in the future horizon. When dealing with N > 20, the algorithm (specifically the optimizer) starts to slow down, and the Cost-Function needs to be adopted to a large future horizon.

So I initialized the variables vector with all necessary elements (state and actuators), and created the constraints vector in the MPC.cpp. This is adopted from the course material implementations.

The main part of this algorithm is located in the FG_eval-class, where the automatic differentiation - API for C++ is used. In this class, the kinematic model and Cost-Function is defined, which will be the one to be minimized.

The definition and fine-tuning of parameters in the Cost-Function will lead to a good or bad optimizer result. I did choose the following penalities with respective weights:

- penalize_epsi = 5.0
- penalize_cte = 0.5
- penalize_ref_v_diff = 1.0

- penalize_steer_usage = 0.5
- penalize_pedal_usage = 0.1

- penalize_sequencial_steer_diffs = 200.0
- penalize_sequencial_v_diffs = 1.0

For example, penalize_steer_usage and penalize_sequencial_steer_diffs are used to make steering smooth like in the following picture: 

![SteeringThumbnail][image4]

(Image from Udacity Self Driving Car Nanodegree course material lession 20)

With reference velocity set to 45 the car drives smoothly over the track. The maximum speed was a velocity of 70, but this may not be quite save any more.

### Deal with Latency

In real applications, there is a delay between computing the actuator controls and the real execution of the control commands. In this project, a virtual latency of 100 ms was implemented using C++ sleep_for()-function. To handle this, I did not use the first actuator result of the optimizer, but the second! 
Since my delta_t is exactly 100 ms, with taking the second entry (second step in future horizon) for actuators, I managed to deal with that delay.


Video of Automatic Driving around the track
---
Please have a look at [this Youtube video](https://youtu.be/jsNbhTJOigY) to watch the car drive at velocity_ref = 45.



## Dependencies

* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1(mac, linux), 3.81(Windows)
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

* **Ipopt and CppAD:** Please refer to [this document](https://github.com/udacity/CarND-MPC-Project/blob/master/install_Ipopt_CppAD.md) for installation instructions.
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
3. For visualization this C++ [matplotlib wrapper](https://github.com/lava/matplotlib-cpp) could be helpful.)
4.  Tips for setting up your environment are available [here](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/23d376c7-0195-4276-bdf0-e02f1f3c665d)
5. **VM Latency:** Some students have reported differences in behavior using VM's ostensibly a result of latency.  Please let us know if issues arise as a result of a VM environment.

## Editor Settings

We've purposefully kept editor configuration files out of this repo in order to
keep it as simple and environment agnostic as possible. However, we recommend
using the following settings:

* indent using spaces
* set tab width to 2 spaces (keeps the matrices in source code aligned)

## Code Style

Please (do your best to) stick to [Google's C++ style guide](https://google.github.io/styleguide/cppguide.html).

## Project Instructions and Rubric

Note: regardless of the changes you make, your project must be buildable using
cmake and make!

More information is only accessible by people who are already enrolled in Term 2
of CarND. If you are enrolled, see [the project page](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/f1820894-8322-4bb3-81aa-b26b3c6dcbaf/lessons/b1ff3be0-c904-438e-aad3-2b5379f0e0c3/concepts/1a2255a0-e23c-44cf-8d41-39b8a3c8264a)
for instructions and the project rubric.

## Hints!

* You don't have to follow this directory structure, but if you do, your work
  will span all of the .cpp files here. Keep an eye out for TODOs.

## Call for IDE Profiles Pull Requests

Help your fellow students!

We decided to create Makefiles with cmake to keep this project as platform
agnostic as possible. Similarly, we omitted IDE profiles in order to we ensure
that students don't feel pressured to use one IDE or another.

However! I'd love to help people get up and running with their IDEs of choice.
If you've created a profile for an IDE that you think other students would
appreciate, we'd love to have you add the requisite profile files and
instructions to ide_profiles/. For example if you wanted to add a VS Code
profile, you'd add:

* /ide_profiles/vscode/.vscode
* /ide_profiles/vscode/README.md

The README should explain what the profile does, how to take advantage of it,
and how to install it.

Frankly, I've never been involved in a project with multiple IDE profiles
before. I believe the best way to handle this would be to keep them out of the
repo root to avoid clutter. My expectation is that most profiles will include
instructions to copy files to a new location to get picked up by the IDE, but
that's just a guess.

One last note here: regardless of the IDE used, every submitted project must
still be compilable with cmake and make./

## How to write a README
A well written README file can enhance your project and portfolio.  Develop your abilities to create professional README files by completing [this free course](https://www.udacity.com/course/writing-readmes--ud777).
