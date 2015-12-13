#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped, Twist

pub = rospy.Publisher('object_tracker/pref_pos', Twist, queue_size=10)
visp_pose = None
pred_pose = None

# Callback for VISP data subscription


def callback_visp(pose):
    global visp_pose
    visp_pose = pose


# Callback for predictedPose subscription
def callback_pred(pose):
    global pred_pose
    pred_pose = pose


def object_tracker():
    rospy.init_node('object_tracker', anonymous=True)

    rospy.Subscriber(
        "/visp_auto_tracker/object_position",
        PoseStamped,
        callback_visp)
    rospy.Subscriber("/ardrone/predictedPose", PoseStamped, callback_pred)

    # create a twist message, fill in the details
    twist = Twist()
    twist.linear.x = 0
    twist.linear.y = 0
    twist.linear.z = 0
    twist.angular.x = 0
    twist.angular.y = 0
    twist.angular.z = 0

    # Publish the estimated waypoint on object_tracker/pref_pos
    pub.publish(twist)
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


if __name__ == '__main__':
    try:
        object_tracker()
    except rospy.ROSInterruptException:
        pass
