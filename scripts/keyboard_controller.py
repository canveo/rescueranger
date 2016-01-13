#!/usr/bin/env python
import rospy
from keyboard.msg import Key


class KeyboardController(object):

    def __init__(self):
        self.tracker = None
        self.key = None
        self.cmd_map = {
            'takeoff': 116,  # Takeoff
            'land': 108,  # Land
            'emergency': 101,  # Emergency
        }
        rospy.Subscriber('/keyboard_listener/keydown', Key, self.callback_keyboard)

    def callback_keyboard(self, data):
        self.key = data.code
        if self.tracker: # 0=wait, 1=taking off, 2=search for marker,  3= aproach marker, 4= land
            if data.code == self.cmd_map['takeoff']:
                if self.tracker.state == 0:
                    self.tracker.state = 1
                    print("Takeoff")
            elif data.code == self.cmd_map['land']:
                self.tracker.state = 4
                print("Landing")
            elif data.code == self.cmd_map['emergency']:
                self.tracker.state = 0
                self.tracker.ardrone_send_emergency()
                print("Emergency")
            else:
                print("usage: Takeoff: t, Land: l, Emergency: e")
