#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import Empty       	 # for land/takeoff/emergency
from ardrone_autonomy.msg import Navdata # for receiving navdata feedback
from std_msgs.msg import Int8

class DroneStatus(object):
    Emergency = 0
    Inited    = 1
    Landed    = 2
    Flying    = 3
    Hovering  = 4
    Test      = 5
    TakingOff = 6
    GotoHover = 7
    Landing   = 8
    Looping   = 9


class ObjectTracker(object):

    def __init__(self):
        self.status = -1
        print 'Rescuranger initilizing'
        self.visp_pose = PoseStamped()
        self.pred_pose = PoseStamped()
        self.pub = rospy.Publisher(
            'object_tracker/pref_pos',
            Twist,
            queue_size=10)
        # Subscribe to the /ardrone/navdata topic, of message type navdata, and call self.ReceiveNavdata when a message is received
        rospy.Subscriber('/ardrone/navdata',Navdata,self.callback_ardrone_navdata) 
        
        # Allow the controller to publish to the /ardrone/takeoff, land and reset topics
        self.pubLand    = rospy.Publisher('/ardrone/land',Empty)
        self.pubTakeoff = rospy.Publisher('/ardrone/takeoff',Empty)
        self.pubReset   = rospy.Publisher('/ardrone/reset',Empty)
        
        # Allow the controller to publish to the /cmd_vel topic and thus control the drone
        self.pubCommand = rospy.Publisher('/cmd_vel',Twist)

        print '...'
        rospy.init_node('object_tracker', anonymous=True)
        rospy.Subscriber(
            "/visp_auto_tracker/object_position",
            PoseStamped,
            self.callback_visp_pose)
        
        # VISP STATE            
        self.visp_state = -1
        rospy.Subscriber('/visp_auto_tracker/status',Int8,self.callback_visp_state) 
        
        self.visp_state_map = {
            'Waiting': 0,  # Not detecting any pattern, just recieving images
            'Detect_flash': 1,  # Pattern detected
            'Detect_model': 2,  # Model successfully initialized (from wrl & xml files)
            'Track_model': 3,   # Tracking model
            'Redetect_flash':4, # Detecting pattern in a small region around where the pattern was last seen
            'Detect_flash2':5 # Detecting pattern in a the whole frame
        }

        print '...'
        rospy.Subscriber(
            "/ardrone/predictedPose",
            PoseStamped,
            self.callback_ardrone_prediction)
        print '...initilized\n'
        
    def callback_ardrone_navdata(self,navdata):
        # Although there is a lot of data in this packet, we're only interested in the state at the moment
        if self.status != navdata.state:
            print("Recieved droneState: %d" % navdata.state)
            self.status = navdata.state

    def ardrone_send_takeoff(self):
        # Send a takeoff message to the ardrone driver
        # Note we only send a takeoff message if the drone is landed - an unexpected takeoff is not good!
        if self.status == DroneStatus.Landed:
            self.pubTakeoff.publish(Empty())

    def ardrone_send_land(self):
        # Send a landing message to the ardrone driver
        # Note we send this in all states, landing can do no harm
        self.pubLand.publish(Empty())

    def ardrone_send_emergency(self):
        # Send an emergency (or reset) message to the ardrone driver
        self.pubReset.publish(Empty())
        
    def SetCommand(self,roll=0,pitch=0,yaw_velocity=0,z_velocity=0):
        # Called by the main program to set the current command
        self.command.linear.x  = pitch
        self.command.linear.y  = roll
        self.command.linear.z  = z_velocity
        self.command.angular.z = yaw_velocity
        
    def SendCommand(self,event):
        # The previously set command is then sent out periodically if the drone is flying
        if self.status == DroneStatus.Flying or self.status == DroneStatus.GotoHover or self.status == DroneStatus.Hovering:
            self.pubCommand.publish(self.command)

    def callback_visp_pose(self, pose):
        self.visp_pose = pose
    
    def callback_visp_state(self, data):
        if data.data != self.visp_state:
            self.visp_state = data.data

            if data.data == 0:
                print("ViSP: Not detecting any pattern, just recieving images")
            if data.data == 1:
                print("ViSP: Pattern detected")
            if data.data == 2:
                print("ViSP: Model successfully initialized (from wrl & xml files)")
            if data.data == 3:
                print("ViSP: Tracking model")
            if data.data == 4:
                print("ViSP: Detecting pattern in a small region around where the pattern was last seen")
            if data.data == 5:
                print("ViSP: Detecting pattern in a the whole frame")

    # Predicted pose from tum_ardrone/drone_stateestimation, written to ardrone
    def callback_ardrone_prediction(self, pose):
        self.pred_pose = pose

    def run(self):
        print 'Rescuranger Started\n Publishing to "/object_tracker/pref_pos"\n'
        twist = Twist()
        # Publish the estimated waypoint on object_tracker/pref_pos
        r = rospy.Rate(100)  # in Hz
        state = 0  # 0=wait, 1=takeoff, 2=hover over marker,  3= search for marker, 4= aproach marker, 5= land, 6= do nothing
        while not rospy.is_shutdown():       
            if state == 0:
                # wait for start command
                if True:
                    state = 1
                    print("Taking off")
            if state == 1:  # takeoff
                pass
                #self.ardrone_send_takeoff()
                #if (self.status == DroneStatus.Flying) or (self.status == DroneStatus.Hovering):
                #    state = 2
            if state == 2:  # hover over marker
                pass
            if state == 3:  # search for marker
                # SetCommand(self,roll=0,pitch=0,yaw_velocity=0,z_velocity=0) #publish go forward one meter
                if True:  # vispFoundMarker
                    state = 5  # change later
                    print("Landing")
            if state == 5:
                self.ardrone_send_land()
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
        print '\n\nRescuranger is terminating.\n'
        self.ardrone_send_emergency()
        # spin() simply keeps python from exiting until this node is stopped


if __name__ == '__main__':
    try:
        tracker = ObjectTracker()
        tracker.run()
    except rospy.ROSInterruptException:
        pass
