# rescueranger
Life raft quad copter design project

## Dependencies
### Keyboard
```bash
sudo apt-get install ros-indigo-keyboard
```


## Object Tracker
To run the object tracker
First get VISP autotracker: http://wiki.ros.org/vision_visp
Then get usb_cam: https://github.com/bosch-ros-pkg/usb_cam
Then run 
```bash
roslaunch visp_auto_tracker tracklive_usb.launch
```
run
```bash 
rosrun rescueranger object_tracker.py
```
 to run the tracker.
