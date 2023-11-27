# Robust Vision Based Object Tracking System
[**TANGO-ESRGAN**](https://github.com/haotiangu/TANGO_ESRGAN) is developed to defend against the white-box attack of vision based object tracking system. It contains the online image restoration CNN TANGO-ESRGAN, the real-time object detection and localization CNN[(YOLO 5.6.1)](https://github.com/haotiangu/yolo5.6.1), and the object tracking to move controller. Our method, which is a computationally efficient denoiser based on Real-ESRGAN, shows several desirable properties for real-time implementation on autonomous systems such as self-driving cars and aerial drones, including faster runtime, lower computational load, higher peak signal-to-noise ratio (PSNR) value for the reconstructed image, improved image resolution and the adaptability to handle a large range of perturbation levels with a fixed network model. Robost Vision Based Object Tracking System is TANGO-ESRGAN based framework.

**News**: 

- __Aug 15, 2023__: Code for white-box defender TANGO-ESRGAN is available now! Check this [repo]([https://github.com/haotiangu/TANGO_ESRGAN]) for more details.
- __Oct 20, 2022__: Multi-level Adaptive Safety Control Framework to assist the end to end automatical landing process for fixed wing type UAV. Check this [repo](https://github.com/SASLabStevens/MASC-Architecture) for more details.

__Authors__: [Haotian Gu](http://sysu-star.com) and [Hamid Jafarnejad Sani](https://www.stevens.edu/profile/hjafarne) from the [SIT Safe Autonomous System Lab](https://saslabstevens.github.io/).

<!-- - __B-spline trajectory optimization guided by topological paths__:
<p align="center">
  <img src="https://github.com/HKUST-Aerial-Robotics/TopoTraj/blob/master/files/icra20_1.gif" width = "420" height = "237"/>
  <img src="https://github.com/HKUST-Aerial-Robotics/TopoTraj/blob/master/files/icra20_2.gif" width = "420" height = "237"/>
</p> -->

<p align="center">
  <img src="files/Online_image_attack.gif" width = "400" height = "225" alt="Alt text" title="Optional title"/>
  <img src="files/image_attack_denoiser.gif" width = "400" height = "225" alt="Alt text" title="Optional title"/>
  <img src="files/driving_attack.gif" width = "400" height = "225" alt="Alt text" title="Optional title"/>
  <img src="files/driving_denoiser.gif" width = "400" height = "225" alt="Alt text" title="Optional title"/>

  <!-- <img src="files/icra20_1.gif" width = "320" height = "180"/> -->
</p>


Left column represent the attacked dynamic and static object tracking system. Right column represents the test result of TANGO-ESRGAN embedded robust object tracking system. Please click the video introdection [here](https://www.youtube.com/watch?v=AeSy51E6A5Q): 
Demonstrations about this work have been reported on the ICRA 2024: [page1](), [page2](),[page3]().
To run this project in minutes, check [Quick Start](#1-Quick-Start). Check other sections for more detailed information.
Please kindly star :star: this project if it helps you. We take great efforts to develope and maintain it :grin::grin:.


## Table of Contents
* [Quick Start](#1-Quick-Start)
* [Algorithms and Papers](#2-Algorithms-and-Papers)
* [Setup and Config](#3-Setup-and-Config)
* [Run Simulations](#4-run-simulations)
* [Use in Your Application](#5-use-in-your-application)
* [Updates](#6-updates)
* [Known issues](#known-issues)

## 1. Quick Start
Before starting, we recommend you to follow [wiki](https://github.com/haotiangu/Robost_Vision_Based_Object_Tracking_System/wiki/The-General-Configuring-Tutorial-of-The-Simulation-Environment) to configure the simulation environment.
Activate the ros environment:
```
  conda activate ros_env
```
The project has been tested on Ubuntu 18.04(ROS Melodic). Take Ubuntu 18.04 as an example, run the following commands to setup:
```
  cd ${YOUR_WORKSPACE_PATH}/src
  git clone https://github.com/Robotics/TANGO-ESRGAN.git
```
Install the dependent software of the TANGO-ESRGAN and YOLO 5.6.1
```
  cd fastdvdnet
  pip install -r requirements.txt
```
Compliling them:
```
  cd ${YOUR_WORKSPACE_PATH} 
  catkin_make
```
You may check the detailed [instruction](#3-setup-and-config) to setup the project. 
After compilation you can run a static object tracking demo: 
```
  source devel/setup.bash && roslaunch tcps_image_attack autoflight.launch # object tracking to move demo
```
run a dynamic object tracking demo: 
```
  source devel/setup.bash && roslaunch tcps_image_attack autodriving.launch # object tracking to move demo
```
Run the online adaptive white-box attack of static object tracking case and its corresponding defencing algorithm:
```
  roslaunch tcps_image_attack train.launch # attack the object localization  
  roslaunch tcps_image_attack train_denoiser_tangoesrgan.launch # object tracking to move when attack exist
```
Run the online adaptive white-box attack of dynamic object tracking case and its corresponding defencing algorithm:
```
  roslaunch tcps_image_attack train_w_car.launch # attack the object localization  
  roslaunch tcps_image_attack train_denoiser_car.launch # object tracking to move when attack exist
```
Please follow the tutorial in Wiki to configure the [simulation environment](https://github.com/haotiangu/Robost_Vision_Based_Object_Tracking_System_Demo/wiki/The-General-Configuring-Tutorial-of-The-Simulation-Environment).



## 2. Algorithms and Papers
The project contains a collection of robust and computationally efficient algorithms for object tracking to move:
* Kinodynamic path searching
* B-spline-based trajectory optimization
* Topological path searching and path-guided optimization
* Perception-aware planning strategy (to appear)
These methods are detailed in our papers listed below. 
Please cite at least one of our papers if you use this project in your research: [Bibtex](files/bib.txt).

- [__Multi-level Adaptation for Automatic Landing with Engine Failure under Turbulent Weather__](https://arxiv.org/abs/2209.04132?context=eess), Haotian Gu and Hamid Jafarnejad Sani, AIAA, 2022.
- [__TANGO-ESRGAN__](), Haotian Gu and Hamid Jafarnejad Sani, IEEE International Conference on Robotics and Automation (__ICRA__), 2024.



All planning algorithms along with other key modules, such as object detection CNN, real-time white-box defender and tracking controller, are implemented in __Robust_Vision_Based_Object_Tracking_Framework__:
- __adaptive_white_box_attacker__: The GAN functions based reinforcement learning agent as an online image generator is trained to misguide the vehicle according to the adversary’s objective. 
- __object_detector__: conduct the real-time object detection and localization in camera coordinate system(yolo 5.6.1). The detection outcome is post-processed into the list of bounding box coordinates (center, width, height) of the detected object sorted by the confidence of detection.
- __adaptive_white_box_defender__: TANGO-ESRGAN defend against the white-box attack in a vision-based object tracking system. This TANGO-ESRGAN is developed based on the Real-ESRGAN, a general state of art video and image restoration algorithm to reconstruct the high-resolution image for detector to label and localize the detected object with high confidence.
- __object_tracking_controller__:  Given the estimated target object by the object detector, the autonomous guidance system uses the tracking controller to keep the bounding box of the target at the center of the camera view and the size of the bounding box within a range.
- __bspline__: A implementation of the B-spline-based trajectory representation.
- __bspline_opt__: The gradient-based trajectory optimization using B-spline trajectory.


## 3. Setup and Config
### Prerequisites
1. Our software is developed and tested in Ubuntu 18.04(ROS Melodic). Follow the documents to install [Melodic](http://wiki.ros.org/melodic/Installation/Ubuntu) according to your Ubuntu version.
   
2. The proposed TANGO-ESRGAN and Robust Vision Based Object Tracking Framework Depend [Anaconda](https://www.anaconda.com/products/distribution#linux), [Pytorch](https://pytorch.org/get-started/locally/)
Simulator we use [Airsim](https://microsoft.github.io/AirSim/airsim_ros_pkgs/) and [Unreal Engine 4.27.1](https://github.com/EpicGames/UnrealEngine). To configure the machine learning environment, please follow WiKi to install [Nvidia Driver 495.29.05](https://www.nvidia.com/download/driverResults.aspx/181159/en-us/), [CUDA 11.3](https://developer.nvidia.com/cuda-11.3.0-download-archive) and [CUDNN 8.2.1](https://developer.nvidia.com/rdp/cudnn-archive).
### Build on ROS
After the prerequisites are satisfied, you can clone this repository to your catkin workspace and catkin_make. A new workspace is recommended:
```
  cd ${YOUR_WORKSPACE_PATH}/src
  git clone https://github.com/Robotics/TANGO-ESRGAN.git
  cd ../
  catkin_make
```
If you encounter problems in this step, please first refer to existing __issues__, __pull requests__ and __Google__ before raising a new issue.



## 4. Use in Your Application
If you have successfully run the simulation and want to use TANGO-ESRGAN and robust vision based object tracking framework in your project,
please explore the code files.

## 5. Updates
- __Oct 20, 2022__:
  
- __April 12, 2023__: 
- __July 5, 2023__: 
- __Jan 30, 2024__: 
## Known issues
### Unexpected crash
## Acknowledgements
## Licence
The source code is released under license.
## Disclaimer
This is research code, it is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of merchantability or fitness for a particular purpose.
