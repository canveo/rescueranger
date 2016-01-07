#!/usr/bin/env python
import rospy
from std_msgs.msg import UInt8

def ardrone_set_camera():
    def __init__(self):
        print('Setting up camera')
        self.cameraID = rospy.get_param("/cameraID")
        print(self.cameraID)
        # Set up the publisher to select the camera
        self.setCam = rospy.Publisher('/ardrone/setcamchannel', UInt8, queue_size=10)
        rospy.init_node('ardrone_set_camera', anonymous=True)
        
    def run(self):
        print('Setting camera channel\n Publishing to "/ardrone/setcamchannel"\n')
        #rospy.sleep(10)
        self.setCam.Publish(self.cameraID)
        print('Published to setcamchannel\n')
        rospy.sleep(5)
        print('\n\nardrone_set_camera is terminating.\n')
        rospy.sleep(2)
        
if __name__ == '__main__':
    try:
        ardrone_set_camera()
    except rospy.ROSInterruptException:
        pass
