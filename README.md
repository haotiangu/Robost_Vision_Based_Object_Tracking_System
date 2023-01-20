# Robost Vision Based Object Tracking System Demo


This repository contains code for defending the black box attack for autonomous driving or flight. We embedded an attacker which attack the object localization part of the CNN object detecter. A FPRN proposed by us also has been embedded inside the package to defend this attack. The proposed FPRN make sure the vision based object tracking to move system perform normally even under the random black box attack.


Autoflight for tracking an detected object to move can be found here -> https://www.youtube.com/watch?v=E_bgRGCXYG4
Online image attack simulaiton video can be found here -> https://www.youtube.com/watch?v=mLpQ3nOqwrU
The FPRN for defending the black box attack can be found here -> https://www.youtube.com/watch?v=MOgg8s-5LVc



## Dependency

- Ubuntu 18.04
- [ROS melodic](http://wiki.ros.org/ROS/Installation)
- [Anaconda](https://www.anaconda.com/products/distribution#linux)
- [pytorch](https://pytorch.org/get-started/locally/)
- [Airsim](https://microsoft.github.io/AirSim/airsim_ros_pkgs/)
- [Unreal Engine](https://github.com/EpicGames/UnrealEngine)
  ```
  wget -O ~/Downloads/gtsam.zip https://github.com/borglab/gtsam/archive/4.0.0-alpha2.zip
  cd ~/Downloads/ && unzip gtsam.zip -d ~/Downloads/
  cd ~/Downloads/gtsam-4.0.0-alpha2/
  mkdir build && cd build
  cmake ..
  sudo make install
  ```

## Compile

You can use the following commands to download and compile the package.

```
cd ~/catkin_ws/src
git clone https://github.com/RobustFieldAutonomyLab/LeGO-LOAM.git
cd ..
catkin_make
```



## Run the package

1. Run the launch file:
```
roslaunch lego_loam run.launch
```


2. Play existing bag files:
```
rosbag play *.bag --clock --topic /velodyne_points /imu/data
```
