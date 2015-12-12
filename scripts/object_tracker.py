#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped


def callback(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data)


def listener():
    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber("/visp_auto_tracker/object_position", PoseStamped, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
