# Quaternion_based_UKF
This repository contains a C++ implementation of a quaternion based Unscented Kalman Filter for Orientatin Tracking and sample data collected on drones. The algorithm is explained in this [paper](https://ieeexplore.ieee.org/document/1257247).

## Dataset 1
<img src="https://github.com/Ravi3191/Quaternion_based_UKF/blob/main/images/pitch_1.png?raw=true" width="300" height="300"> <img src="https://github.com/Ravi3191/Quaternion_based_UKF/blob/main/images/roll_1.png?raw=true" width="300" height="300"> <img src="https://github.com/Ravi3191/Quaternion_based_UKF/blob/main/images/yaw_1.png?raw=true" width="300" height="300">

## Dataset 2
<img src="https://github.com/Ravi3191/Quaternion_based_UKF/blob/main/images/pitch_2.png?raw=true" width="300" height="300"> <img src="https://github.com/Ravi3191/Quaternion_based_UKF/blob/main/images/roll_2.png?raw=true" width="300" height="300"> <img src="https://github.com/Ravi3191/Quaternion_based_UKF/blob/main/images/yaw_2.png?raw=true" width="300" height="300">


## Dependencies for Running Locally
* cmake >= 3.1
  * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1 (Linux, Mac), 3.81 (Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 7.5.0
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools](https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* Eigen >= 3.3.7
  * Download the latest stable version - [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page#Download)
  * Extract the zip file in your home directory and rename the folder to Eigen3.


## Basic Build Instructions
1. Clone this repo.
2. Make a build directory in the top level directory: `mkdir build`
3. Compile: `cd build && cmake .. && make`
4. Run the binary with appropriate file no (1 or 2): `./filter (file_no)`.
5. To Visualize the plots run the plot_graph.py from home directory.
