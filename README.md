# CarND-Controls-MPC
Self-Driving Car Engineer Nanodegree Program



## MPC 

[//]: # (Image References)

[constrain]:./img/constraint.png "parameter"


## 1. What is MPC ?
MPC stands for Model Predictive Control system which is one of advanced method controlling a series of proccess 
while satisfying a set of constrains. It has been used in the plan control like chemical industory or Electric insdustory
long time since the 1980s. 
Thiugh it might be most popular control for some engineers, why MPC has be used for autonomous driving technology ?
Lets say about the story of the car driving at night scene (this example would fit to our next project aim), 
our human has an ability predicting the road with tiny spot lights brinking at the pedestrian deck in the street. 
In addition we are able to keep the car in the limited lane space
to avoid corrision accident with other cars running other sided roads. 
Its action and feedback is must item for self-driving car which needs autonomous control without any human interactions. 
Our main focus thema in this course, MPC brings to our forefront solving predicions about fure trajectory based on the 
control actions.
With recent technology of having advanced capability of the computer, we are able to easily adopt this model for
complex real-world simulations like car-engineering and aerospace etc. 

## 2. Object
Here the last project of Nano Degree Term2 course, utilizing mordernized technical control MPC mentioned in the above, 
involves being able to plan optimal actions based on the waypoints of the tack and predict the forward car path
and finally drive a car on the simualation software adjusting actuators (steering and throttle).
Following is mathematical formulation we took account for this project.

![alt text][constrain]

Hence, 
* Lf measures the distance between the front of the vehicle and its center of gravity. 
* f(x) is the evaluation of the polynomial f at point x 
* psidest is the tangencial angle of the polynomial f evaluated at x.

## 3. Ipopt Library
Ipopt is GPL software to provide automatical derivatives calculations with C++ source codes. In order to minimize cost function in described above constraints formula, we build cusomized FG_Eval class to solve minimzed cost. This Class method is referred with below link.
This calculation Library is able to find optimal values from computation while keeping the constraints set directly to the actuators and the constraints defined by the vehicle model.

Reference link:
https://www.coin-or.org/CppAD/Doc/ipopt_solve_get_started.cpp.htm

Below is our cost function to be minized.

>  // Cost for CTE, psi error and velocity
  for (unsigned int t = 0; t < N; t++) {
    fg[0] += a * CppAD::pow(vars[cte_start + t] - ref_cte, 2);
    fg[0] += b * CppAD::pow(vars[epsi_start + t] - ref_espi, 2);
    fg[0] += c * CppAD::pow(vars[v_start + t] - ref_v, 2);
  }

>  // Costs for steering (delta) and acceleration (a)
  for (unsigned int t = 0; t < N-1; t++) {
    fg[0] += d * CppAD::pow(vars[delta_start + t], 2);
    fg[0] += e * CppAD::pow(vars[a_start + t], 2);
  }

>  // Costs related to the change in steering and acceleration (makes the ride smoother)
  for (unsigned int t = 0; t < N-2; t++) {
    fg[0] += f * pow(vars[delta_start + t + 1] - vars[delta_start + t], 2);
    fg[0] += g * pow(vars[a_start + t + 1] - vars[a_start + t], 2);
  }

__a b c d e f g are numerical parameters for cost function. Now those values are manually set to determine best simulation performance.__

For example, if we want to extremly minimize cost for CET psi, we intentionally set big number on a and b, c to penalize the error between the theoritical and real numerical data on the driving trajectory of the car. When we set 10000 for a and b, the simulation car slowly approaches each road corner and turn curves gently (=with slowest speed) so that car avoid any corrision and course out of the track road.  

### 4. Timestep Length and Elapsed Duration (N & dt)


We have done try and error to select an appropriate time values N and dt. Personally I started to select N values between 10 and 20 and for dt 0.05 and 0.1 respectively. As result of my experiemntal approach, N=10 and dt=0.1 was best combinaton to show best performance on the simlation car driving. Setting smaller than 0.1 for dt value, dt=0.05 and N = 20 showed not good outcome on the simulation. 

### 5. Polynomial Fitting and MPC Preprocessing

Based on the text book of UdaCity, it is said that the reference trajectory is typically passed to the control block as a polynomial. 
As this proves, 3rd order polynominal third order polynomials will fit trajectories for most roads.
For transforming global position to local x=0 y=0, we have used subtraction `ptsx - px`. 

> // Need Eigen vectors for polyfit
Eigen::VectorXd ptsx_car(ptsx.size());
Eigen::VectorXd ptsy_car(ptsy.size());

> // Transform method the points to the vehicle's orientation
// convert Global position to local position
for (unsigned int i = 0; i < ptsx.size(); i++) {
  double x = ptsx[i] - px;
  double y = ptsy[i] - py;
  ptsx_car[i] = x * cos(-psi) - y * sin(-psi);
  ptsy_car[i] = x * sin(-psi) + y * cos(-psi);
}

> // Fits a 3rd-order polynomial to the above x and y coordinates
auto coeffs = polyfit(ptsx_car, ptsy_car, 3);

__Also, the track waypoints is displayed in yellow in the simulation screen.__

### 6. Model Predictive Control with Latency

In a real world, as command execution has some time elapsed before the actual action is activated, so-called __Latemcy__ is overcomed in the controlled system. We set 100ms as realistic latency number so that we predict 100ms ahead to control future vehicle trajectory to be run automatically.

>  // Predict state after latency
  // x, y and psi are all zero after transformation above
  double pred_px = 0.0 + v * dt; // 0 + v x dt
  const double pred_py = 0.0; // = 0
  double pred_psi = 0.0 + v * -delta / Lf * dt; // 
  double pred_v = v + a * dt;
  double pred_cte = cte + v * sin(epsi) * dt;
  double pred_epsi = epsi + v * -delta / Lf * dt;

__Then, predicted values are moved into state vector and then pass to cost function.__

>  // Feed in the predicted state values
  Eigen::VectorXd state(6);
  state << pred_px, pred_py, pred_psi, pred_v, pred_cte, pred_epsi;



---

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
