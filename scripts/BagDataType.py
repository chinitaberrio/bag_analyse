import tf
import utm
import math
import simplekml


import numpy as np



class BagDataType:
    def __init__(self, topic_list):
        self.topic_list = topic_list
        self.data = dict()
        for topic in topic_list:
            self.data[topic] = []

    def convert_numpy(self):
        for topic in self.topic_list:
            self.data[topic] = np.array(self.data[topic])

    def calculate_odometry(self, odometry, initial_x = 0., initial_y = 0., initial_theta = 0., stationary_constraint=False):
        deltas = np.diff(odometry[:, 0])
        cumulative_x = initial_x
        cumulative_y = initial_y
        cumulative_theta = initial_theta

        for (t, x, y, z, roll, pitch, yaw, v, yaw_rate), dt in zip(odometry[1:, :], deltas):
            if not stationary_constraint or v > 0.:
                #dt = 0.1
                cumulative_theta += dt * yaw_rate
                cumulative_x += math.cos(cumulative_theta) * v * dt
                cumulative_y += math.sin(cumulative_theta) * v * dt
            yield ([t, cumulative_x, cumulative_y, cumulative_theta, yaw_rate, v])

    def add_KML_path(self, kml, label, east_base, east_vector, north_base, north_vector, colour, zone, letter):
        ls = kml.newlinestring(name=str(label))
        ls.style.linestyle.width = 4

        path = []
        for east, north in zip(east_vector, north_vector):
            lat, lon = utm.to_latlon(east + east_base, north + north_base, zone, letter)
            path.append((lon, lat))

        ls.coords = path
        ls.style.linestyle.color = colour


# IMU has an additional field which is filled in with the latest speed measurement
#  This enables the recalculation of odometry information using yaw and speed
class IMU(BagDataType):
    def new_message(self, topic, msg, t, current_speed):
        euler = tf.transformations.euler_from_quaternion(
            [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w])

        self.data[topic].append([t.to_sec(),
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


class Odometry(BagDataType):
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



class GNSS(BagDataType):

    def __init__(self, topic_list):
        self.previous_east = dict()
        self.previous_north = dict()
        BagDataType.__init__(self, topic_list)

    def new_message(self, topic, msg, t):
        east, north, zone, letter = utm.from_latlon(msg.latitude, msg.longitude)

        estimated_heading = 0.
        if topic in self.previous_east:
            estimated_heading = math.atan2(north - self.previous_north[topic], east - self.previous_east[topic])
            #if (topic == '/gps/fix'):
            #    print (estimated_heading, east, north, self.previous_east[topic], self.previous_north, north - self.previous_north, east - self.previous_east)
        self.data[topic].append([t.to_sec(), 0., east, north, msg.altitude, estimated_heading, msg.latitude, msg.longitude])

        self.previous_east[topic] = east
        self.previous_north[topic] = north


class Steering(BagDataType):
    def new_message(self, topic, msg, t):
        self.data[topic].append([t.to_sec(), msg.position[2]])


class Velocity(BagDataType):
    def new_message(self, topic, msg, t):
        self.data[topic].append([t.to_sec(), msg.velocity[0], msg.velocity[1], msg.velocity[2], msg.velocity[3]])



