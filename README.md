# Extended Kalman Filter Project Starter Code
Self-Driving Car Engineer Nanodegree Program
This file is updated based on the assignment submission.

[//]: # (Image References)
//]: # (Image References)
[image1]: ./images/d1-nax_9-nay_9-LR.png
[image2]: ./images/d2-nax_9-nay_9-LR.png


In this project you will utilize a kalman filter to estimate the state of a moving object of interest with noisy lidar and radar measurements. Passing the project requires obtaining RMSE values that are lower than the tolerance outlined in the project rubric. 

This project involves the Term 2 Simulator which can be downloaded [here](https://github.com/udacity/self-driving-car-sim/releases)

This repository includes two files that can be used to set up and install [uWebSocketIO](https://github.com/uWebSockets/uWebSockets) for either Linux or Mac systems. For windows you can use either Docker, VMware, or even [Windows 10 Bash on Ubuntu](https://www.howtogeek.com/249966/how-to-install-and-use-the-linux-bash-shell-on-windows-10/) to install uWebSocketIO. Please see [this concept in the classroom](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/16cf4a78-4fc7-49e1-8621-3450ca938b77) for the required version and installation scripts.

Once the install for uWebSocketIO is complete, the main program can be built and run by doing the following from the project top directory.

## Relevant Implementation Details.
This section describes the files modified / extended to implement the Extended Kalman filter.

### `main.cpp`
Two boolean variables `use_radar_` and `use_laser_` are defined and initialized to control whether data from both, laser and radar or individual sensors to be processed.
These variables give flexibility to debug, test and observe the RMSE related to indivual sensor processing as well as the improvement upon sensor fusion, which is the main advantage of Kalman Filter.

### `FusionEKF.cpp`
This file contains main sensor data processing objects and associated functions. The main implementation logic includes the following.
1. Variables and matrices initialization including (x, F, H_laser, H_radar, R_laser, R_radar, noise parameters, etc.)
2. Handling first frame with appropriate position values assignment
3. Updating F and Q matrices based on elapsed time between measurements, in prediction step
4. Invoking relevant update step as per sensor

### `kalman_filter.cpp`
This file implements Extended Kalman filter class with `Predict()` and `Update()` functions. There is a separate `Update()` function for each sensor.
There are two important items were taken care.
1. Avoid division by zero: Ensure that position values are non_zero (atleast one of the position coordinate either px or py shall be non_zero) to avoid division by zero.
2. Normalize Angles: The angle part of  y (difference between measurement and prediction) shall be normalized between +pi and -pi for correct RMSE value computation.

Rest of the implementation is taken from class quizzes.

### `tools.cpp`
This file contains essentially functions to compute Jacobian for radar data processing and RMSE error computation.


## Buid
I used Windows Linux Subsystem for Ubuntu environment and associated instructions to build the program and to ensure communication between simulator and the EKF program.


## Observations
I exercised the implementation in various scenarios as described in this section.

### Observation 1:  Data set 1, with sensor fusion (laser + radar); noise_ax = 9; noise_ay = 9

The RMSE values observed were well within the limits of RMSE <= [Px:.11, Py:.11, Vx:0.52, Vy:0.52], i.e. ** [Px:0.0964	Py:0.0853	Vx:0.4154	Vy:0.4316]**, as captured in the following picture.
![alt text][image1]

### Observation 2:  Data set 2, with sensor fusion (laser + radar); noise_ax = 9; noise_ay = 9
Similar to the Observation 1, the RMSE values are well below the specified limits for Data set 2 as well.
![alt text][image2]

### Observation 3: Observations with Fusion (Laser (R) + Radar (R)), Laser only ('L') and  Radar only ('R').
I observed three combinations of RMSE measurement process with L, R and Fusion (L+R).  The observations are captured in the following table.
|Num|Noise_ax|Noise_ay|Initial (vx, vy)|Mode (L, R, L+R)|Dataset|RMSE-Px|RMSE-Py|RMSE-Vx|RMSE-Vy|
|1|9|9|(0,0)|L+R|1|0.0974|0.0855|0.4517|0.4407|
|2|9|9|(0,0)|L|1|0.1838|0.1542|0.6051|0.4858|
|3|9|9|(0,0)|R|1|0.2323|0.3354|0.6178|0.6786|
|4|9|9|(0,0)|L+R|2|0.0726|0.0967|0.4579|0.4966|
|5|9|9|(0,0)|L|2|0.1673|0.1568|0.6063|0.4955|
|6|9|9|(0,0)|R|2|0.2415|0.3373|0.6025|0.7565|


### Observation 4: Observations with Fusion with differenent Vx and Vy initialization.
I did a quick experiment with Vx and Vy initialized close to ground truths. The RMSE went down. A priori knowledge about approximate velocity of object would be helpful.


|1|9|9|(0,0)|L+R|1|0.0974|0.0855|0.4517|0.4407|
|2|9|9|(1,1)|L+R|1|0.0964|0.0853|0.4154|0.4316|
|3|9|9|(5.19,0)|L+R|1|0.0945|0.0848|0.3305|0.4097|

### Observation 5: Observations with different acceleration noise values
I did a quick experiment with Vx and Vy initialized close to ground truths. The RMSE went down. A priori knowledge about approximate velocity of object would be helpful.
Not much helpful. I guess the noise values have to be provided by equipment vendors. I guess the optimum values also depend on the quality of environment.
|Num|Noise_ax|Noise_ay|Initial(vx, vy)|Mode (L, R, L+R)|Dataset|RMSE-Px|RMSE-Py|RMSE-Vx|RMSE-Vy|
|1|4|4|(0,0)|L+R|1|0.1130|0.1021|0.4920|0.5143|
|2|9|9|(0,0)|L+R|1|0.0974|0.0855|0.4517|0.4407|
|3|16|16|(0,0)|L+R|1|0.0921|0.0832|0.4419|0.4105|
|4|25|25|(0,0)|L+R|1|0.0898|0.0844|0.4428|0.4012|
|4|36|36|(0,0)|L+R|1|0.0908|0.0892|0.4922|0.6660|
|5|4|4|(0,0)|L+R|2|0.0814|0.1219|0.5006|0.5695|
|6|9|9|(0,0)|L+R|2|0.0726|0.0967|0.4579|0.4966|
|7|16|16|(0,0)|L+R|2|0.0723|0.0890|0.4484|0.4678|
|8|25|25|(0,0)|L+R|2|0.7360|0.0868|0.4502|0.4578|
|9|36|36|(0,0)|L+R|2|0.0752|0.0869|0.4574|0.4596|

Overal, this assignment offered me good grasp the implmentation of EKF and helps further understanding theory behind.


## General build instrctions 

1. mkdir build
2. cd build
3. cmake ..
4. make
5. ./ExtendedKF

Tips for setting up your environment can be found [here](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/23d376c7-0195-4276-bdf0-e02f1f3c665d)

Note that the programs that need to be written to accomplish the project are src/FusionEKF.cpp, src/FusionEKF.h, kalman_filter.cpp, kalman_filter.h, tools.cpp, and tools.h

The program main.cpp has already been filled out, but feel free to modify it.

Here is the main protcol that main.cpp uses for uWebSocketIO in communicating with the simulator.


INPUT: values provided by the simulator to the c++ program

["sensor_measurement"] => the measurement that the simulator observed (either lidar or radar)


OUTPUT: values provided by the c++ program to the simulator

["estimate_x"] <= kalman filter estimated position x
["estimate_y"] <= kalman filter estimated position y
["rmse_x"]
["rmse_y"]
["rmse_vx"]
["rmse_vy"]

---

## Other Important Dependencies

* cmake >= 3.5
  * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1 (Linux, Mac), 3.81 (Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools](https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make` 
   * On windows, you may need to run: `cmake .. -G "Unix Makefiles" && make`
4. Run it: `./ExtendedKF `

## Editor Settings

We've purposefully kept editor configuration files out of this repo in order to
keep it as simple and environment agnostic as possible. However, we recommend
using the following settings:

* indent using spaces
* set tab width to 2 spaces (keeps the matrices in source code aligned)

## Code Style

Please (do your best to) stick to [Google's C++ style guide](https://google.github.io/styleguide/cppguide.html).

## Generating Additional Data

This is optional!

If you'd like to generate your own radar and lidar data, see the
[utilities repo](https://github.com/udacity/CarND-Mercedes-SF-Utilities) for
Matlab scripts that can generate additional data.

## Project Instructions and Rubric

Note: regardless of the changes you make, your project must be buildable using
cmake and make!

More information is only accessible by people who are already enrolled in Term 2
of CarND. If you are enrolled, see [the project resources page](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/382ebfd6-1d55-4487-84a5-b6a5a4ba1e47)
for instructions and the project rubric.

## Hints and Tips!

* You don't have to follow this directory structure, but if you do, your work
  will span all of the .cpp files here. Keep an eye out for TODOs.
* Students have reported rapid expansion of log files when using the term 2 simulator.  This appears to be associated with not being connected to uWebSockets.  If this does occur,  please make sure you are conneted to uWebSockets. The following workaround may also be effective at preventing large log files.

    + create an empty log file
    + remove write permissions so that the simulator can't write to log
 * Please note that the ```Eigen``` library does not initialize ```VectorXd``` or ```MatrixXd``` objects with zeros upon creation.

## Call for IDE Profiles Pull Requests

Help your fellow students!

We decided to create Makefiles with cmake to keep this project as platform
agnostic as possible. Similarly, we omitted IDE profiles in order to ensure
that students don't feel pressured to use one IDE or another.

However! We'd love to help people get up and running with their IDEs of choice.
If you've created a profile for an IDE that you think other students would
appreciate, we'd love to have you add the requisite profile files and
instructions to ide_profiles/. For example if you wanted to add a VS Code
profile, you'd add:

* /ide_profiles/vscode/.vscode
* /ide_profiles/vscode/README.md

The README should explain what the profile does, how to take advantage of it,
and how to install it.

Regardless of the IDE used, every submitted project must
still be compilable with cmake and make.

## How to write a README
A well written README file can enhance your project and portfolio.  Develop your abilities to create professional README files by completing [this free course](https://www.udacity.com/course/writing-readmes--ud777).

