#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import Empty       	 # for land/takeoff/emergency
from ardrone_autonomy.msg import Navdata  # for receiving navdata feedback
from keyboard_controller import KeyboardController


class DroneStatus(object):
    Emergency = 0
    Inited = 1
    Landed = 2
    Flying = 3
    Hovering = 4
    Test = 5
    TakingOff = 6
    GotoHover = 7
    Landing = 8
    Looping = 9


class ObjectTracker(object):

    def __init__(self):
        self.status = -1
        print('Rescueranger initilizing')
        self.visp_pose = PoseStamped()
        self.pred_pose = PoseStamped()
        self.pub = rospy.Publisher(
            'object_tracker/pref_pos',
            Twist,
            queue_size=10)
        # Subscribe to the /ardrone/navdata topic, of message type navdata, and
        # call self.ReceiveNavdata when a message is received
        self.subNavdata = rospy.Subscriber(
            '/ardrone/navdata',
            Navdata,
            self.NavdataCallback)

        # Allow the controller to publish to the /ardrone/takeoff, land and
        # reset topics
        self.pubLand = rospy.Publisher('/ardrone/land', Empty)
        self.pubTakeoff = rospy.Publisher('/ardrone/takeoff', Empty)
        self.pubReset = rospy.Publisher('/ardrone/reset', Empty)

        # Allow the controller to publish to the /cmd_vel topic and thus control
        # the drone
        self.pubCommand = rospy.Publisher('/cmd_vel', Twist)

        # Keyboard Controller
        self.keyboard = KeyboardController()

        print('...')
        rospy.init_node('object_tracker', anonymous=True)
        rospy.Subscriber(
            "/visp_auto_tracker/object_position",
            PoseStamped,
            self.callback_visp)
        print('...')
        rospy.Subscriber(
            "/ardrone/predictedPose",
            PoseStamped,
            self.callback_pred)
        print('...initilized\n')

    def NavdataCallback(self, navdata):
        # Although there is a lot of data in this packet, we're only interested
        # in the state at the moment
        if self.status != navdata.state:
            rospy.loginfo("Recieved droneState: %d", navdata.state)
            self.status = navdata.state

    def SendTakeoff(self):
        # Send a takeoff message to the ardrone driver
        # Note we only send a takeoff message if the drone is landed - an
        # unexpected takeoff is not good!
        if self.status == DroneStatus.Landed:
            self.pubTakeoff.publish(Empty())

    def SendLand(self):
        # Send a landing message to the ardrone driver
        # Note we send this in all states, landing can do no harm
        self.pubLand.publish(Empty())

    def SendEmergency(self):
        # Send an emergency (or reset) message to the ardrone driver
        self.pubReset.publish(Empty())

    def SetCommand(self, roll=0, pitch=0, yaw_velocity=0, z_velocity=0):
        # Called by the main program to set the current command
        self.command.linear.x = pitch
        self.command.linear.y = roll
        self.command.linear.z = z_velocity
        self.command.angular.z = yaw_velocity

    def SendCommand(self, event):
        # The previously set command is then sent out periodically if the drone
        # is flying
        if (
            self.status == DroneStatus.Flying or
            self.status == DroneStatus.GotoHover or
            self.status == DroneStatus.Hovering
        ):
            self.pubCommand.publish(self.command)

    def callback_visp(self, pose):
        self.visp_pose = pose

    def callback_pred(self, pose):
        self.pred_pose = pose

    def run(self):
        print(
            'Rescuranger Started\n Publishing to "/object_tracker/pref_pos"\n')
        twist = Twist()
        # Publish the estimated waypoint on object_tracker/pref_pos
        r = rospy.Rate(100)  # in Hz
        # 0=wait, 1=takeoff, 2=hover over marker,  3= search for marker, 4=
        # aproach marker, 5= land, 6= do nothing
        state = 0
        while not rospy.is_shutdown():
            key = self.keyboard.get_key()
            if key == self.keyboard.cmd_map['emergency']:
                self.SendEmergency()

            if state == 0:
                # wait for start command
                if True:
                    state = 1
                    rospy.loginfo("Taking off")
            if state == 1:  # takeoff
                pass
                # self.SendTakeoff()
                # if (
                # (self.status == DroneStatus.Flying) or
                # (self.status == DroneStatus.Hovering)
                # ):
                #    state = 2
            if state == 2:  # hover over marker
                pass
            if state == 3:  # search for marker
                # SetCommand(self,roll=0,pitch=0,yaw_velocity=0,z_velocity=0)
                # publish go forward one meter
                if True:  # vispFoundMarker
                    state = 5  # change later
                    rospy.loginfo("Landing")
            if state == 5:
                self.SendLand()
                if self.status == DroneStatus.Landed:
                    state = 6
            if state == 6:
                pass  # do nothing
            twist.linear.x = self.visp_pose.pose.position.x + \
                self.pred_pose.pose.position.x
            twist.linear.y = self.visp_pose.pose.position.y + \
                self.pred_pose.pose.position.y
            twist.linear.z = self.visp_pose.pose.position.z + \
                self.pred_pose.pose.position.z

            self.pub.publish(twist)
            r.sleep()
        print('\n\nRescuranger is terminating.\n')
        # spin() simply keeps python from exiting until this node is stopped


if __name__ == '__main__':
    try:
        tracker = ObjectTracker()
        tracker.run()
    except rospy.ROSInterruptException:
        pass
