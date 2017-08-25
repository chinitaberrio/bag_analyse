import tf
import math
import numpy as np

from BagDataType import BagDataType

# IMU has an additional field which is filled in with the latest speed measurement
#  This enables the recalculation of odometry information using yaw and speed
class IMU(BagDataType):

    TIME = 0
    QX = 1
    QY = 2
    QZ = 3
    QW = 4
    ROLL = 5
    PITCH = 6
    YAW = 7
    ROLL_RATE = 8
    PITCH_RATE = 9
    YAW_RATE = 10
    SPEED = 11

    def new_message(self, topic, msg, t, current_speed):
        euler = tf.transformations.euler_from_quaternion(
            [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w])

        self.data[topic].append([msg.header.stamp.to_sec(),
                                 msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w,
                                 euler[0], euler[1], euler[2],
                                 msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z,
                                 current_speed])


    def recalculate_odometry_2d(self, initial_heading=0.):
        # recompute the position from the imu measurements (yaw and attitude)
        self.gyro_path = dict()
        self.attitude_path = dict()

        for key, source in self.data.iteritems():

            if len(source) == 0:
                continue

            self.gyro_path[key] = []
            for new_sample in self.calculate_imu_odometry_gyro(source, initial_theta=initial_heading):
                self.gyro_path[key].append(new_sample)

            self.attitude_path[key] = []
            for new_sample in self.calculate_imu_odometry_attutide(source):
                self.attitude_path[key].append(new_sample)

            self.gyro_path[key] = np.array(self.gyro_path[key])
            self.attitude_path[key] = np.array(self.attitude_path[key])

    PATH_TIME = 0
    PATH_X = 1
    PATH_Y = 2
    PATH_YAW = 3
    PATH_YAW_RATE = 4
    PATH_SPEED = 5

    def calculate_imu_odometry_gyro(self, imu, initial_theta=0.):
        deltas = np.diff(imu[:,0])
        cumulative_x = 0.
        cumulative_y = 0.
        cumulative_theta = initial_theta

        for (t, ox, oy, oz, ow, roll, pitch, yaw, roll_rate, pitch_rate, yaw_rate, v), dt in zip(imu[1:,:], deltas):
            #dt = 0.01
            cumulative_theta += dt * yaw_rate
            cumulative_x += math.cos(cumulative_theta) * v * dt
            cumulative_y += math.sin(cumulative_theta) * v * dt
            yield ([t, cumulative_x, cumulative_y, cumulative_theta, yaw_rate, v])


    def calculate_imu_odometry_attutide(self, imu):
        deltas = np.diff(imu[:, 0])
        cumulative_x = 0.
        cumulative_y = 0.

        for (t, ox, oy, oz, ow, roll, pitch, yaw, roll_rate, pitch_rate, yaw_rate, v), dt in zip(imu[1:, :], deltas):
            #dt = 0.01
            cumulative_x += math.cos(yaw) * v * dt
            cumulative_y += math.sin(yaw) * v * dt
            yield ([t, cumulative_x, cumulative_y, yaw, yaw_rate, v])

