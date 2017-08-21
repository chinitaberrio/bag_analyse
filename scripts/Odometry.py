import tf
import numpy as np

from BagDataType import BagDataType



class Odometry(BagDataType):

    TIME = 0
    X = 1
    Y = 2
    Z = 3
    ROLL = 4
    PITCH = 5
    YAW = 6
    SPEED = 7
    YAW_RATE = 8

    def new_message(self, topic, msg, t):
        euler = tf.transformations.euler_from_quaternion(
            [msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z,
             msg.pose.pose.orientation.w])

        self.data[topic].append([t.to_sec(),
                                 msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z,
                                 euler[0], euler[1], euler[2],
                                 msg.twist.twist.linear.x,
                                 msg.twist.twist.angular.z])

    def recalculate_odometry_2d(self, initial_heading=0.):
        # recompute the position from the odometry rate measurements (yaw and velocity)
        self.recalculated_data = dict()

        for key, source in self.data.iteritems():

            if len(source) == 0:
                continue

            self.recalculated_data[key] = []
            for new_sample in self.calculate_odometry(source, initial_theta=initial_heading):
                self.recalculated_data[key].append(new_sample)

            self.recalculated_data[key] = np.array(self.recalculated_data[key])

