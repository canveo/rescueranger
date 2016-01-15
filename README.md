# rescueranger
Life raft quad copter design project at Chalmers Technological University, Sweden

Created by 4 MPSYS master students 2015, Björn, Tomas, Mikael and Gunnar.

The AR Parrot 2.0 drone currently tracks and follows a ArUco marker at a set reference position relative the marker.

## Object Tracker
To run the object tracker
Get the needed external packages listed in package.xml (or below)

Then run 
```bash
roslaunch rescueranger devel.launch
```
and run the following in a new bash window
```bash 
rosrun rescueranger ardrone_positioningPID.py
```
 to run the tracker.

## Dependencies
A list of seperate packages that are needed to run the rescueranger project.
ROS indigo is what the package was run and tested on, and atleast a version of ROS is required.

Most assume you are standing in your catkin workspace.

### Robot Operating System (ROS)
[ROS indigo](http://wiki.ros.org/indigo/Installation)

```bash
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver hkp://pool.sks-keyservers.net --recv-key 0xB01FA116
sudo apt-get update
sudo apt-get install ros-indigo-desktop-full
sudo rosdep init
rosdep update
```

### Python
```bash
sudo apt-get install python-rosinstall
```
Decent python libraries
```bash
sudo apt-get install python-numpy python-scipy python-matplotlib ipython ipython-notebook python-pandas python-sympy python-nose
```

### Ardrone Autonomy (ROS)
[ardrone_autonomy](https://github.com/tum-vision/ardrone_autonomy)

The main flight controller for the AR Parrot drone

```bash
sudo git clone git://github.com/tum-vision/ardrone_autonomy.git ardrone_autonomy
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:`pwd`/ardrone_autonomy
cd ardrone_autonomy
sudo ./build_sdk.sh
rosmake
```

### ArUco (ROS)
[aruco](https://github.com/pal-robotics/aruco_ros)

Used by marker_pose_detection, even though it is not listed as a dependancy for the package there.

```bash
cd ~/catkin_ws/src/
git clone https://github.com/pal-robotics/aruco_ros.git
cd ..
catkin_make install
```

### Image View (ROS)
[image_view](http://wiki.ros.org/image_view)

Used to feed a image stream to a window on a computer.

```bash
sudo apt-get install ros-indigo-image-view
```

### Keyboard (ROS)
[keyboard](https://github.com/lrse/ros-keyboard)

To intercept and interpret keyboard presses.

```bash
sudo apt-get install ros-indigo-keyboard
```

### Pal Vision Segmentation (ROS)
[vision_segmentation](http://wiki.ros.org/pal_vision_segmentation)

Also used by marker_pose_detection.

```bash
cd src/pal_vision_segmentation/
git clone https://github.com/pal-robotics/pal_vision_segmentation.git
cd ..
cd ..
catkin_make
```

### USB CAM (ROS)
[usb_cam](https://github.com/bosch-ros-pkg/usb_cam)

Not required, but is good for testing since you can use a usb camera (such as a webcam) instead of the onboard camera.

```bash
sudo apt-get install ros-indigo-usb-cam
```

### Marker Pose Detection (ROS)
[viewpoint_estimation](https://github.com/durovsky/marker_pose_detection)

Estimates a ArUco markers position and orientation in the image stream.

```bash
cd src/
git clone https://github.com/durovsky/marker_pose_detection.git
cd marker_pose_detection/
git checkout master
cd ..
cd ..
catkin_make
```

### TUM ARDRONE (ROS)
[tum_ardrone](http://wiki.ros.org/tum_ardrone)

Not required, but used for position estimation

```bash
sudo git clone git://github.com/tum-vision/tum_ardrone.git tum_ardrone
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:`pwd`/tum_ardrone
rosmake tum_ardrone
```

### 3D Visualization tool for ROS (Rviz)
[rviz](http://wiki.ros.org/rviz)

Not required, but decent to visualize positions of e.g. markers relative world/camera etc.
```bash
sudo apt-get install ros-indigo-rviz
```

