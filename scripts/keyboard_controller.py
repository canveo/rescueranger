import rospy
from keyboard.msg import Key


class KeyboardController(object):

    def __init__(self):
        self.last_key = None
        self.cmd_map = {
            'start': 115,  # Start
            'takeoff': 116,  # Takeoff
            'land': 108,  # Land
            'emergency': 101,  # Emergency
        }
        self.key_read = True
        rospy.Subscriber('/keyboard/keydown', Key, self.callback_keyboard)

    def callback_keyboard(self, data):
        if data.code != self.last_key:
            self.key_read = False
            self.last_key = data.code

    def get_key(self):
        if not self.key_read:
            return self.last_key
        else:
            return None
