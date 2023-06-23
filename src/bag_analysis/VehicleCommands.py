from BagDataType import BagDataType


class Ackermann(BagDataType):
    TIME = 0
    STEERING = 1
    STEERING_VEL = 2
    SPEED = 3
    ACCELERATION = 4
    JERK = 5

    def new_message(self, topic, msg, t):
        self.data[topic].append([t.to_sec(), msg.steering_angle,
                                 msg.steering_angle_velocity, msg.speed, msg.acceleration, msg.jerk])


class TwistCommand(BagDataType):
    TIME = 0
    V_X = 1
    V_Y = 2
    V_Z = 3
    OMEGA_X = 4
    OMEGA_Y = 5
    OMEGA_Z = 6

    def new_message(self, topic, msg, t):
        self.data[topic].append([t.to_sec(), msg.linear.x, msg.linear.y,
                                 msg.linear.z, msg.angular.x, msg.angular.y, msg.angular.z])
