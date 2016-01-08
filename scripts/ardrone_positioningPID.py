#!/usr/bin/env python
import math
import rospy
import PID
import time
from geometry_msgs.msg import Twist
from tf2_msgs.msg import TFMessage
from std_msgs.msg import Empty       	 # for land/takeoff/emergency
from std_msgs.msg import Int8
from ardrone_autonomy.msg import Navdata  # for receiving navdata feedback
from keyboard_controller import KeyboardController
from visualization_msgs.msg import Marker # for Marker estimation from viewpoint_estimation

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
        
        self.pidx = PID.PID(Px, Ix, Dx)
        self.pidy = PID.PID(Py, Iy, Dy)
        self.pidz = PID.PID(Pz, Iz, Dz)
        self.pidyaw = PID.PID(Pyaw, Iyaw, Dyaw)
                
        self.smplTime = 1/60
        
        self.height_goal = 0.0
        self.horizontal_goal = 0.0
        self.distance_goal = 1.5
        self.yaw_goal = 0.0
        
        self.pidx.setReference(self.distance_goal)
        self.pidy.setReference(self.horizontal_goal)
        self.pidz.setReference(self.height_goal)
        self.pidyaw.setReference(self.yaw_goal)
        self.pidx.setSampleTime(smplTime)
        self.pidy.setSampleTime(smplTime)
        self.pidz.setSampleTime(smplTime)
        self.pidyaw.setSampleTime(smplTime)
        
        self.estimated_marker = None
        self.marker_seen = False
        self.marker_timeout_threshold = 0.2
        self.marker_last_seen = 0 - self.marker_timeout_threshold
        
        print('Rescueranger initilizing')
        
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
            
        # Marker pose subscriber
        rospy.Subscriber(
            '/Estimated_marker',
            Marker,
            self.callback_estimated_marker_pose)

        # Marker State
        self.estimated_marker_state_map = {
            'Waiting': 0,  # Not detecting any pattern, just recieving images
            'Detected_marker': 1,  # Pattern detected
        }

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
        if (
            self.ardrone_state == DroneStatus.Flying or 
            self.ardrone_state == DroneStatus.GotoHover or
            self.ardrone_state == DroneStatus.Hovering
        ):
            self.pubCommand.publish(self.command)

    # Predicted pose from tum_ardrone/drone_stateestimation, written to ardrone
    def callback_ardrone_prediction(self, pose):
        self.pred_pose = pose
    
    #TODO fix this callback: 
    def callback_estimated_marker_pose(self, Marker):
        self.marker_last_seen = time.time()
        self.estimated_marker = Marker
                
    #################################################################################
    #################################################################################
    # The main running loop
    # This is where all the intresting magic happens
    #################################################################################
    #################################################################################
    def run(self):
        twist = Twist()
        # Publish the estimated waypoint on object_tracker/pref_pos
        r = rospy.Rate(100)  # in Hz
        # 0=wait, 1=takeoff, 2=hover over marker,  3= search for marker, 4=
        # aproach marker, 5= land, 6= do nothing
        while not rospy.is_shutdown():
            dt = time.time() - self.marker_last_seen
            if dt < self.marker_timeout_threshold:
                self.marker_seen = True
            else:
                self.marker_seen = False
            if self.state == 0:
                # wait for start command
                pass
            if self.state == 1:  # takeoff
                self.ardrone_send_takeoff()
                if (
                    (self.ardrone_state == DroneStatus.Hovering)
                ):
                    self.state = 2
                    print("Hovering")
                #self.state = 2 #TODO Remove this, this ignores actual flight!
            if self.state == 2:  # hover over marker
                if self.marker_seen:
                    # Remap camera cooridnate frame to drone coordinate frame
                    # X: Positive forward drone frame
                    # Y: Positive left drone frame
                    # Z: Positive up drone frame
                    # Yaw: Positive clockwise drone frame
                    marker_z = self.estimated_marker.pose.position.x
                    marker_y = -self.estimated_marker.pose.position.y
                    marker_x = self.estimated_marker.pose.position.z
                    q0 = self.estimated_marker.pose.orientation.w
                    q1 = self.estimated_marker.pose.orientation.x
                    q2 = self.estimated_marker.pose.orientation.y
                    q3 = self.estimated_marker.pose.orientation.z
                    marker_roll = math.atan2(2*(q0*q3 + q1*q2), 1 - 2*(q2**2 + q3**2))
                    marker_pitch = math.atan2(2*(q0*q1 + q2*q3), 1 - 2*(q1**2 + q2**2))
                    marker_yaw = math.asin(2*(q0*q2 - q3*q1))
                    #print((round(marker_roll*180/math.pi),round(marker_pitch*180/math.pi),round(marker_yaw*180/math.pi)))                    
                    height_err = self.height_goal - marker_z
                    horizontal_err = self.horizontal_goal - marker_yaw
                    distance_err = self.distance_goal - (marker_z**2 + marker_y**2 + marker_x**2)**0.5
                    yaw_err = self.yaw_goal - marker_y
                    
                    # Quad distance from marker error
                    if distance_err < 0: # To far from marker
                        x_vel = 0.02 # Move forward
                    else:
                        x_vel = -0.02 # Move backward

                    # Quad horizontal error relative marker
                    if horizontal_err > 0: # To far right of the marker
                        y_vel = 0.02 # Move left
                    else:
                        y_vel = -0.02 # Move right

                    # Quad heigt error
                    if height_err > 0: # To low relative the marker
                        z_vel = 0.1 # Move up
                    else:
                        z_vel = -0.1 # Move down

                    # Quad yaw from horizontal drifting error
                    if yaw_err > 0: # Marker in the left of the image
                        yaw_vel = -0.12 # Turn anticlockwise
                    else:
                        yaw_vel = 0.12 # Turn clockwise
                    
                    x_vel = x_vel
                    y_vel = y_vel
                    z_vel = z_vel
                    yaw_vel = yaw_vel
                    print((x_vel, y_vel, z_vel, yaw_vel))
                    self.ardrone_set_xyzy(x_vel, y_vel, z_vel, yaw_vel)
                    #self.ardrone_set_xyzy(0, 0, 0, 0)
                    #print((x_err, y_err, z_err, yaw_err))
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
