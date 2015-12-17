#!/usr/bin/env python
import math
import rospy
from geometry_msgs.msg import Twist
from tf2_msgs.msg import TFMessage
from std_msgs.msg import Empty       	 # for land/takeoff/emergency
from std_msgs.msg import Int8
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
        self.state = 0
        self.ardrone_state = -1
        print('Rescueranger initilizing')
        self.vision_pose = TFMessage()
        self.marker_seen = False
        self.pub = rospy.Publisher(
            'object_tracker/pref_pos',
            Twist,
            queue_size=10)
        # Subscribe to the /ardrone/navdata topic, of message type navdata, and
        # call self.ReceiveNavdata when a message is received
        self.subNavdata = rospy.Subscriber(
            '/ardrone/navdata',
            Navdata,
            self.callback_ardrone_navdata)

        # Allow the controller to publish to the /ardrone/takeoff, land and
        # reset topics
        self.pubLand = rospy.Publisher('/ardrone/land', Empty)
        self.pubTakeoff = rospy.Publisher('/ardrone/takeoff', Empty)
        self.pubReset = rospy.Publisher('/ardrone/reset', Empty)

        # Allow the controller to publish to the /cmd_vel topic and thus control
        # the drone
        self.command = Twist()
        self.pubCommand = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        # Keyboard Controller
        self.keyboard = KeyboardController()
        self.keyboard.tracker = self

        print('...')
        rospy.init_node('object_tracker', anonymous=True)
        rospy.Subscriber(
            "/tf",
            TFMessage,
            self.callback_vision_pose)

        # VISP STATE
        self.visp_state = -1
        rospy.Subscriber(
            '/visp_auto_tracker/status',
            Int8,
            self.callback_visp_state)

        self.visp_state_map = {
            'Waiting': 0,  # Not detecting any pattern, just recieving images
            'Detect_flash': 1,  # Pattern detected
            # Model successfully initialized (from wrl & xml files)
            'Detect_model': 2,
            'Track_model': 3,   # Tracking model
            # Detecting pattern in a small region around where the pattern was
            # last seen
            'Redetect_flash': 4,
            'Detect_flash2': 5  # Detecting pattern in a the whole frame
        }

        print('...')
        # rospy.Subscriber(
        #     "/ardrone/predictedPose",
        #     PoseStamped,
        #     self.callback_ardrone_prediction)
        print('...initilized\n')
        rospy.on_shutdown(self.ardrone_send_land)

    def callback_ardrone_navdata(self, navdata):
        # Although there is a lot of data in this packet, we're only interested
        # in the state at the moment
        if self.ardrone_state != navdata.state:
            print("Recieved droneState: %d" % navdata.state)
            self.ardrone_state = navdata.state

    def ardrone_send_takeoff(self):
        # Send a takeoff message to the ardrone driver
        # Note we only send a takeoff message if the drone is landed - an
        # unexpected takeoff is not good!
        if self.ardrone_state == DroneStatus.Landed:
            self.pubTakeoff.publish(Empty())

    def ardrone_send_land(self):
        # Send a landing message to the ardrone driver
        # Note we send this in all states, landing can do no harm
        self.pubLand.publish(Empty())

    def ardrone_send_emergency(self):
        # Send an emergency (or reset) message to the ardrone driver
        self.pubReset.publish(Empty())
        print("Publishing reset")

    def ardrone_set_xyzy(self, x=0, y=0, z=0, yaw=0):
        # Called by the main program to set the current command
        self.command.linear.x = x
        self.command.linear.y = y
        self.command.linear.z = z
        self.command.angular.z = yaw

    def ardrone_update_rpyz(self, event):
        # The previously set command is then sent out periodically if the drone
        # is flying
        if (
            self.ardrone_state == DroneStatus.Flying or
            self.ardrone_state == DroneStatus.GotoHover or
            self.ardrone_state == DroneStatus.Hovering
        ):
            self.pubCommand.publish(self.command)

    def ardrone_send_command(self,event):
        # The previously set command is then sent out periodically if the drone is flying
        if self.ardrone_state == DroneStatus.Flying or self.ardrone_state == DroneStatus.GotoHover or self.ardrone_state == DroneStatus.Hovering:
            self.pubCommand.publish(self.command)

    def callback_vision_pose(self, pose):
        if pose.transforms[0].child_frame_id == "ardrone_base_frontcam":
            if pose.transforms[0].header.frame_id != "ar_marker":
                self.marker_seen = False
            else:
                self.marker_seen = True
                self.vision_pose = pose

    def callback_visp_state(self, data):
        if data.data != self.visp_state:
            self.visp_state = data.data

            if data.data == 0:
                print("ViSP: Not detecting any pattern, just recieving images")
            if data.data == 1:
                print("ViSP: Pattern detected")
            if data.data == 2:
                print(
                    "ViSP: Model successfully initialized (from wrl & xml files)")
            if data.data == 3:
                print("ViSP: Tracking model")
            if data.data == 4:
                print(
                    "ViSP: Detecting pattern in a small region around where the pattern was last seen")
            if data.data == 5:
                print("ViSP: Detecting pattern in a the whole frame")

    # Predicted pose from tum_ardrone/drone_stateestimation, written to ardrone
    def callback_ardrone_prediction(self, pose):
        self.pred_pose = pose

    def run(self):
        twist = Twist()
        # Publish the estimated waypoint on object_tracker/pref_pos
        r = rospy.Rate(100)  # in Hz
        # 0=wait, 1=takeoff, 2=hover over marker,  3= search for marker, 4=
        # aproach marker, 5= land, 6= do nothing
        while not rospy.is_shutdown():
            if self.state == 0:
                # wait for start command
                pass
            if self.state == 1:  # takeoff
                pass
                self.ardrone_send_takeoff()
                if (
                    (self.ardrone_state == DroneStatus.Hovering)
                ):
                    self.state = 2
                    print("Hovering")
            if self.state == 2:  # hover over marker
                if self.marker_seen:
                    vision_x = self.vision_pose.transforms[0].transform.translation.x
                    vision_y = self.vision_pose.transforms[0].transform.translation.y
                    vision_z = self.vision_pose.transforms[0].transform.translation.z
                    q1 = self.vision_pose.transforms[0].transform.rotation.x
                    q2 = self.vision_pose.transforms[0].transform.rotation.y
                    q3 = self.vision_pose.transforms[0].transform.rotation.z
                    q0 = self.vision_pose.transforms[0].transform.rotation.w
                    yaw = math.atan2(2*(q0*q3 + q1*q2), 1 - 2*(q2**2 + q3**2))
                    x_goal = 0.0
                    y_goal = 0.0
                    z_goal = 0.9
                    yaw_goal = 0.0
                    x_err = x_goal - vision_x
                    y_err = y_goal - vision_y
                    z_err = z_goal - vision_z
                    yaw_err = yaw_goal - yaw
                    if z_err < 0:
                        x_vel = 0.05
                    else:
                        x_vel = -0.05
                    if x_err < 0:
                        y_vel = 0.05
                    else:
                        y_vel = -0.05
                    if y_err < 0:
                        z_vel = -0.1
                    else:
                        z_vel = 0.1
                    if yaw_err < 0:
                        yaw_vel = -0.1
                    else:
                        yaw_vel = 0.1
                    self.ardrone_set_xyzy(x_vel, y_vel, z_vel, yaw_vel)
                    print((x_err, y_err, z_err, yaw_err))
                else:
                    self.ardrone_set_xyzy(0, 0, 0, 0)

            if self.state == 3:  # search for marker
                pass
            if self.state == 5:  # land
                if self.ardrone_state == DroneStatus.Landed:
                    self.state = 0
                    print("Landed")
                else:
                    self.ardrone_send_land()
            if self.state == 6:
                pass  # do nothing
            self.ardrone_update_rpyz(None)
            # twist.linear.x = self.visp_pose.pose.position.x + \
            #     self.pred_pose.pose.position.x
            # twist.linear.y = self.visp_pose.pose.position.y + \
            #     self.pred_pose.pose.position.y
            # twist.linear.z = self.visp_pose.pose.position.z + \
            #     self.pred_pose.pose.position.z

            self.pub.publish(twist)
            r.sleep()
        print('\n\nRescuranger is terminating.\n')
        self.ardrone_send_emergency()
        # spin() simply keeps python from exiting until this node is stopped


if __name__ == '__main__':
    try:
        tracker = ObjectTracker()
        tracker.run()
    except rospy.ROSInterruptException:
        pass
