from BagDataType import BagDataType


class TorqueControllerDebug(BagDataType):

    TIME = 0
    DESIRED_SPEED = 1
    ERROR = 2
    COMMAND = 3
    BASE_THROTTLE = 4

    def new_message(self, topic, msg, t):
        self.data[topic].append([t.to_sec(
        ), msg.desired_speed, msg.error, msg.command, msg.base_throttle])
