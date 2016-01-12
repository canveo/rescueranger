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

### Ardrone Autonomy (ROS)
[ardrone_autonomy](https://github.com/tum-vision/ardrone_autonomy)
The main flight controller for the AR Parrot drone

### ArUco (ROS)
[aruco](https://github.com/pal-robotics/aruco_ros)
Used by marker_pose_detection, even though it is not listed as a dependancy for the package there.

### Image View (ROS)
[image_view](http://wiki.ros.org/image_view)
Used to feed a image stream to a window on a computer.

### Keyboard (ROS)
[keyboard](https://github.com/lrse/ros-keyboard)
To intercept and interpret keyboard presses.

```bash
sudo apt-get install ros-indigo-keyboard
```

### Pal Vision Segmentation (ROS)
[vision_segmentation](http://wiki.ros.org/pal_vision_segmentation)
Also used by marker_pose_detection.

### USB CAM (ROS)
[usb_cam](https://github.com/bosch-ros-pkg/usb_cam)
Not required, but is good for testing since you can use a usb camera (such as a webcam) instead of the onboard camera.

### Marker Pose Detection (ROS)
[viewpoint_estimation](https://github.com/durovsky/marker_pose_detection)
Estimates a ArUco markers position and orientation in the image stream.
