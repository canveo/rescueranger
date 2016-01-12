# rescueranger
Life raft quad copter design project

## Dependencies

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
```bash
sudo apt-get install ros-indigo-keyboard
```
To intercept and interpret keyboard presses.

### Pal Vision Segmentation (ROS)
[vision_segmentation](http://wiki.ros.org/pal_vision_segmentation)
Also used by marker_pose_detection.

### USB CAM (ROS)
[usb_cam](https://github.com/bosch-ros-pkg/usb_cam)
Not required, but is good for testing since you can use a usb camera (such as a webcam) instead of the onboard camera.

### Marker Pose Detection (ROS)
[viewpoint_estimation](https://github.com/durovsky/marker_pose_detection)
Estimates a ArUco markers position and orientation in the image stream.

## Object Tracker
To run the object tracker
Get the needed external packages listed in package.xml (or above)

Then run 
```bash
roslaunch rescueranger devel.launch
```
and run the following in a new bash window
```bash 
rosrun rescueranger ardrone_positioningPID.py
```
 to run the tracker.
