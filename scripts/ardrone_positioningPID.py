#!/usr/bin/env python
import math
import rospy
import PID
import time
import sys
from geometry_msgs.msg import Twist
from tf2_msgs.msg import TFMessage
from std_msgs.msg import Empty       	 # for land/takeoff/emergency
from std_msgs.msg import Int8
from ardrone_autonomy.msg import Navdata  # for receiving navdata feedback
from keyboard_controller import KeyboardController
from visualization_msgs.msg import Marker # for Marker estimation from viewpoint_estimation
from rescueranger.msg import CoordStruct

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
        
        self.Px = 0.065
        self.Ix = 0
        self.Dx = 0.07
        self.saturation_x = 0.35
        
        self.Py = 0.1
        self.Iy = 0
        self.Dy = 0.035
        self.saturation_y = 0.3
        
        self.Pz = 1.1
        self.Iz = 0.01
        self.Dz = 0
        self.saturation_z = 0.4
        
        self.Pyaw = 0.75
        self.Iyaw = 0.065
        self.Dyaw = 0.2
        self.saturation_yaw = 0.6
        
        
        self.pidx = PID.PID(self.Px, self.Ix, self.Dx, self.saturation_x)
        self.pidy = PID.PID(self.Py, self.Iy, self.Dy, self.saturation_y)
        self.pidz = PID.PID(self.Pz, self.Iz, self.Dz, self.saturation_z)
        self.pidyaw = PID.PID(self.Pyaw, self.Iyaw, self.Dyaw, self.saturation_yaw)
        
        self.pidyaw.windup_guard = 1
                
        self.smplTime = 1/200
        
        self.height_goal = 0.0
        self.horizontal_goal = 0.0
        self.distance_goal = 4.5
        self.yaw_goal = 0.0
        
        self.pidx.setReference(self.distance_goal)
        self.pidy.setReference(self.horizontal_goal)
        self.pidz.setReference(self.height_goal)
        self.pidyaw.setReference(self.yaw_goal)
        self.pidx.setSampleTime(self.smplTime)
        self.pidy.setSampleTime(self.smplTime)
        self.pidz.setSampleTime(self.smplTime)
        self.pidyaw.setSampleTime(self.smplTime)
        
        self.estimated_marker = None
        self.marker_seen = False
        self.marker_timeout_threshold = 0.5
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
        self.pubLand = rospy.Publisher('/ardrone/land', Empty, queue_size=10)
        self.pubTakeoff = rospy.Publisher('/ardrone/takeoff', Empty, queue_size=10)
        self.pubReset = rospy.Publisher('/ardrone/reset', Empty, queue_size=10)

        # Allow the controller to publish to the /cmd_vel topic and thus control
        # the drone
        self.command = Twist()
        self.pubCommand = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        
        # Publish error relative the marker
        self.pub_info_error = rospy.Publisher(
            '/rescueranger/MarkerError',
            CoordStruct,
            queue_size=10
            )
        self.info_error = CoordStruct()
        
        # Publish error relative the marker
        self.pub_info_marker = rospy.Publisher(
            '/rescueranger/MarkerPos',
            CoordStruct,
            queue_size=10
            )
        self.info_marker = CoordStruct()

        # Publish error relative the marker
        self.pub_info_vel = rospy.Publisher(
            '/rescueranger/VelocityOutput',
            CoordStruct,
            queue_size=10
            )
        self.info_vel = CoordStruct()

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
    
    # Estimated marker position relative to the camera 
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
        r = rospy.Rate(300)  # in Hz
        # 0=wait, 1=takeoff, 2=hover over marker,  3= search for marker, 4=
        # aproach marker, 5= land, 6= do nothing
        while not rospy.is_shutdown():
            #sys.stderr.write("\x1b[2J\x1b[H")
            dt = time.time() - self.marker_last_seen
            if dt < self.marker_timeout_threshold:
                if self.marker_seen == False:
                    self.pidx.clear()
                    self.pidy.clear()
                    self.pidz.clear()
                    self.pidyaw.clear()
                self.marker_seen = True
                #print("Marker Seen")
            else:
                self.marker_seen = False
                #print("Marker Lost") 
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
                    
                    # Calculate error       
                    height_err = self.height_goal - marker_z
                    horizontal_err = self.horizontal_goal - marker_y
                    distance_err = self.distance_goal - (marker_z**2 + marker_y**2 + marker_x**2)**0.5
                    yaw_err = self.yaw_goal - marker_yaw
                    # Publish errors
                    self.info_error.x = distance_err
                    self.info_error.y = horizontal_err
                    self.info_error.z = height_err
                    self.info_error.yaw = yaw_err
                    self.info_error.time = time.time()
                    self.pub_info_error.publish(self.info_error)
                    
                    # Calculate current marker position
                    current_horizontal = 3.5*marker_yaw - 0.8*marker_y
                    real_distance = (marker_x**2 + marker_y**2 + marker_z**2)**0.5
                    current_distance = real_distance 
                    current_yaw = math.asin(-marker_y/real_distance) - 0.65*marker_yaw/real_distance
                    current_height =  marker_z + 0.04*real_distance
                    # Publish current
                    self.info_marker.x = current_distance
                    self.info_marker.y = current_horizontal
                    self.info_marker.z = current_height
                    self.info_marker.yaw = current_yaw
                    self.info_marker.time = time.time()
                    self.pub_info_marker.publish(self.info_marker)
                    
                    # Update the PIDs
                    self.pidx.updatePID(current_distance)
                    self.pidy.updatePID(current_horizontal)
                    self.pidz.updatePID(current_height)
                    self.pidyaw.updatePID(current_yaw)
                    
                    x_vel = -self.pidx.output
                    y_vel = self.pidy.output
                    z_vel = self.pidz.output
                    yaw_vel = self.pidyaw.output
                    
                    # Publish velocities
                    self.info_vel.x = x_vel
                    self.info_vel.y = y_vel
                    self.info_vel.z = z_vel
                    self.info_vel.yaw = yaw_vel
                    self.info_vel.time = time.time()                   
                    self.pub_info_vel.publish(self.info_vel)
                    #x_vel = 0
                    #y_vel = 0
                    #z_vel = 0
                    #yaw_vel = 0
                    
                    
                    #print((x_vel, y_vel, z_vel, yaw_vel, current_yaw))
                    #print((round(100*yaw_vel), round(100*self.pidyaw.ITerm)))
                    #print((self.pidx.last_error, self.pidy.last_error, self.pidz.last_error, self.pidyaw.last_error))
                    self.ardrone_set_xyzy(x_vel, y_vel, z_vel, yaw_vel)
                    #self.ardrone_set_xyzy(0, 0, 0, 0)
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
