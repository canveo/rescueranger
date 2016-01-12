# rescueranger
Life raft quad copter design project

## Dependencies
### Keyboard
```bash
sudo apt-get install ros-indigo-keyboard
```


## Object Tracker
To run the object tracker
Get the needed external packages listed in package.xml

Then run 
```bash
roslaunch rescueranger devel.launch
```
and run the following in a new bash window
```bash 
rosrun rescueranger ardrone_positioningPID.py
```
 to run the tracker.
