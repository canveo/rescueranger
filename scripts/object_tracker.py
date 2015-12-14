#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped, Twist


class ObjectTracker(object):

    def __init__(self):
        print 'Rescuranger initilizing'
        self.visp_pose = PoseStamped()
        self.pred_pose = PoseStamped()
        self.pub = rospy.Publisher(
            'object_tracker/pref_pos',
            Twist,
            queue_size=10)
        print '...'
        rospy.init_node('object_tracker', anonymous=True)
        rospy.Subscriber(
            "/visp_auto_tracker/object_position",
            PoseStamped,
            self.callback_visp)
        print '...'
        rospy.Subscriber(
            "/ardrone/predictedPose",
            PoseStamped,
            self.callback_pred)
        print '...initilized\n'

    def callback_visp(self, pose):
        self.visp_pose = pose

    def callback_pred(self, pose):
        self.pred_pose = pose

    def run(self):
        print 'Rescuranger Started\n Publishing to "/object_tracker/pref_pos"\n'
        twist = Twist()
        # Publish the estimated waypoint on object_tracker/pref_pos
        r = rospy.Rate(100)  # in Hz
        while not rospy.is_shutdown():
            twist.linear.x = self.visp_pose.pose.position.x + \
                self.pred_pose.pose.position.x
            twist.linear.y = self.visp_pose.pose.position.y + \
                self.pred_pose.pose.position.y
            twist.linear.z = self.visp_pose.pose.position.z + \
                self.pred_pose.pose.position.z
            self.pub.publish(twist)
            r.sleep()
        print '\n\nRescuranger is terminating.\n'
        # spin() simply keeps python from exiting until this node is stopped


if __name__ == '__main__':
    try:
        tracker = ObjectTracker()
        tracker.run()
    except rospy.ROSInterruptException:
        pass
