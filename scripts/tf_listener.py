import rospy


class TFListener(object):

    def __init__(self):
        self.last_key = None
        self.cmd_map = {
            'start': 115,  # Start
            'takeoff': 116,  # Takeoff
            'land': 108,  # Land
            'emergency': 101,  # Emergency
        }
        self.key_read = True
        rospy.Subscriber('/tf', Key, self.callback_keyboard)

    def callback_keyboard(self, data):
        if data.code != self.last_key:
            self.key_read = False
            self.last_key = data.code


    def run(self):
        pass

if __name__ == "__main__":
    pass
