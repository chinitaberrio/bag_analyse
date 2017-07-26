import tf
import utm
import rosbag
from matplotlib import pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

import simplekml


import numpy as np
import pandas as pd

import math

bag_file = '/home/stew/data/EV/20170627-dual-imu/trip-from-mq_2017-06-27-12-30-30.bag'  # 4.1

bag_file = '/home/stew/data/EV/20170627-dual-imu/trip-from-mq_2017-06-27-12-30-30.bag'  # 4.1

bag_file = '/home/stew/data/hugh_aaron_imu_tests/1-3.bag'
bag_file = '/home/stew/data/tmp/ekf3.bag'

bag_file = '/home/stew/data/EV/vectornav/filtered_2017-07-25-13-49-29.bag'



print ("starting read of " + bag_file)
bag = rosbag.Bag(bag_file)


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
        print odometry.shape
        deltas = np.diff(odometry[:, 0])
        cumulative_x = initial_x
        cumulative_y = initial_y
        cumulative_theta = initial_theta

        for (t, x, y, z, roll, pitch, yaw, v, yaw_rate), dt in zip(odometry[1:, :], deltas):
            if not stationary_constraint or v > 0.:
                dt = 0.1
                cumulative_theta += dt * yaw_rate
                cumulative_x += math.cos(cumulative_theta) * v * dt
                cumulative_y += math.sin(cumulative_theta) * v * dt
            yield ([t, cumulative_x, cumulative_y, cumulative_theta, yaw_rate, v])


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

    def recalculate_odometry_2d(self):
        # recompute the position from the imu measurements (yaw and attitude)
        self.gyro_path = dict()
        self.attitude_path = dict()

        for key, source in self.data.iteritems():

            if len(source) == 0:
                continue

            self.gyro_path[key] = []
            for new_sample in self.calculate_imu_odometry_gyro(source):
                self.gyro_path[key].append(new_sample)

            self.attitude_path[key] = []
            for new_sample in self.calculate_imu_odometry_attutide(source):
                self.attitude_path[key].append(new_sample)

            self.gyro_path[key] = np.array(self.gyro_path[key])
            self.attitude_path[key] = np.array(self.attitude_path[key])


    def calculate_imu_odometry_gyro(self, imu):
        deltas = np.diff(imu[:,0])
        cumulative_x = 0.
        cumulative_y = 0.
        cumulative_theta = 0.

        for (t, ox, oy, oz, ow, roll, pitch, yaw, roll_rate, pitch_rate, yaw_rate, v), dt in zip(imu[1:,:], deltas):
            dt = 0.01
            cumulative_theta += dt * yaw_rate
            cumulative_x += math.cos(cumulative_theta) * v * dt
            cumulative_y += math.sin(cumulative_theta) * v * dt
            yield ([t, cumulative_x, cumulative_y, cumulative_theta, yaw_rate, v])


    def calculate_imu_odometry_attutide(self, imu):
        deltas = np.diff(imu[:, 0])
        cumulative_x = 0.
        cumulative_y = 0.
        cumulative_theta = 0.

        for (t, ox, oy, oz, ow, roll, pitch, yaw, roll_rate, pitch_rate, yaw_rate, v), dt in zip(imu[1:, :], deltas):
            dt = 0.01
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

    def recalculate_odometry_2d(self):
        # recompute the position from the odometry rate measurements (yaw and velocity)
        self.recalculated_data = dict()

        for key, source in self.data.iteritems():

            if len(source) == 0:
                continue

            self.recalculated_data[key] = []
            for new_sample in self.calculate_odometry(source):
                self.recalculated_data[key].append(new_sample)

            self.recalculated_data[key] = np.array(self.recalculated_data[key])



class GNSS(BagDataType):
    def new_message(self, topic, msg, t):
        east, north, zone, letter = utm.from_latlon(msg.latitude, msg.longitude)
        self.data[topic].append([t.to_sec(), msg.speed, east, north, msg.altitude])


class Steering(BagDataType):
    def new_message(self, topic, msg, t):
        self.data[topic].append([t.to_sec(), msg.position[2]])


class Velocity(BagDataType):
    def new_message(self, topic, msg, t):
        self.data[topic].append([t.to_sec(), msg.velocity[0], msg.velocity[1], msg.velocity[2], msg.velocity[3]])



imu = IMU(['/gy80/imu/raw', '/vn100/imu'])
odometry = Odometry(['/localisation/odometry/filtered', '/zio/odometry/front', '/zio/odometry/rear', '/gy80/imu/raw', '/gy80/odometry', '/imu'])
gnss = GNSS(['/gnss/extended_fix'])
steering = Steering(['/zio/joint_states'])
velocity = Velocity(['/zio/joint_states'])

current_speed = 0.
target_speed_source = '/zio/odometry/rear'

for topic, msg, t in bag.read_messages():

    # pick only the times where the vehicle was moving GOING OUT
    #if (t.to_sec() < 30+1.49678753e9 or t.to_sec() > 75+1.49678753e9):
    #    continue

    if topic == target_speed_source:
        current_speed = msg.twist.twist.linear.x

    if topic in gnss.topic_list:
        gnss.new_message(topic, msg, t)

    elif topic in imu.topic_list:
        imu.new_message(topic, msg, t, current_speed)

    elif topic in odometry.topic_list:
        odometry.new_message(topic, msg, t)

    # the steering wheel messages name starts with 'left_steering'
    elif topic in steering.topic_list and msg.name[0] == 'left_steering':
        steering.new_message(topic, msg, t)

    # the wheel velocity messages name starts with 'front_left_wheel'
    elif topic in steering.topic_list and msg.name[0] == 'front_left_wheel':
        velocity.new_message(topic, msg, t)

    """elif topic == '/zio/joint_states':
        if (msg.name[0] == 'left_steering'):
            steering['steering'].append([t.to_sec(), msg.position[2]])
        elif (msg.name[0] == 'front_left_wheel'):
            steering['velocity'].append([t.to_sec(), msg.velocity[0], msg.velocity[1], msg.velocity[2], msg.velocity[3]])
    """

bag.close()


imu.convert_numpy()
odometry.convert_numpy()
gnss.convert_numpy()

odometry.recalculate_odometry_2d()
imu.recalculate_odometry_2d()

plt.figure()
plt.title("yaw rate")
plt.plot(odometry.data['/localisation/odometry/filtered'][:, 0],
         odometry.data['/localisation/odometry/filtered'][:, 8], 'r')
plt.plot(odometry.data['/gy80/odometry'][:, 0],
         odometry.data['/gy80/odometry'][:, 8], 'g')

plt.figure()
plt.title("velocity")
plt.plot(odometry.data['/localisation/odometry/filtered'][:, 0],
         odometry.data['/localisation/odometry/filtered'][:, 7], 'r-')
plt.plot(odometry.data['/gy80/odometry'][:, 0],
         odometry.data['/gy80/odometry'][:, 7], 'g-')


plt.figure()
plt.title("IMU yaw")

plt.hold(True)

vector_nav_fixed_delta_time = 0.01
plt.plot(imu.data['/vn100/imu'][:,0], np.cumsum(imu.data['/vn100/imu'][:,10] * vector_nav_fixed_delta_time), 'b')
plt.plot(imu.data['/vn100/imu'][:,0], np.unwrap(imu.data['/vn100/imu'][:,7]), 'g')
plt.legend(['vectornav from gyro', 'vectornav from attitude'])



plt.figure()
plt.title("IMU yaw rate")

plt.hold(True)

vector_nav_fixed_delta_time = 0.01
plt.plot(imu.data['/vn100/imu'][:,0], imu.data['/vn100/imu'][:,10], 'b')
plt.plot(imu.data['/gy80/imu/raw'][:,0], imu.data['/gy80/imu/raw'][:,10], 'r')
plt.legend(['vn100', 'gy80'])


# plot the bridge data
fig = plt.figure()
fig.suptitle('ROSbag database sensor data plot against position')
ax = fig.add_subplot(111, projection='3d')
plt.hold(True)
plt.plot(odometry.data['/localisation/odometry/filtered'][:, 1],
         odometry.data['/localisation/odometry/filtered'][:, 2],
         odometry.data['/localisation/odometry/filtered'][:, 3])


# plot the bridge data
fig = plt.figure()
fig.suptitle('GNSS')
ax = fig.add_subplot(111, projection='3d')
plt.hold(True)
plt.plot(gnss.data['/gnss/extended_fix'][:, 2],
         gnss.data['/gnss/extended_fix'][:, 3],
         gnss.data['/gnss/extended_fix'][:, 4])



east_base, north_base, zone, letter = utm.from_latlon(-33.889565,151.1933352)

kml = simplekml.Kml()
ls = kml.newlinestring(name=str('path'))
ls.style.linestyle.width = 4

path = []
for east, north in zip (odometry.data['/localisation/odometry/filtered'][:, 2], odometry.data['/localisation/odometry/filtered'][:, 1]):
    lat, lon = utm.to_latlon(east + east_base, north + north_base, zone, letter)
    path.append((lon,lat))

ls.coords = path
ls.style.linestyle.color = simplekml.Color.green

kml.save('/home/stew/test.kml')







plt.figure()
plt.title("Position from various sources")

plt.subplot(321)
plt.hold(True)


plt.plot(imu.gyro_path['/gy80/imu/raw'][:,1],
         imu.gyro_path['/gy80/imu/raw'][:,2], 'g.')

plt.plot(imu.attitude_path['/gy80/imu/raw'][:,1],
         imu.attitude_path['/gy80/imu/raw'][:,2], 'r.')


#plt.plot(position['/zio/odometry/front'][:,1], position['/zio/odometry/front'][:,2], 'b')
#plt.plot(recalc_position['/zio/odometry/front'][:,1],
#         recalc_position['/zio/odometry/front'][:,2], 'g-.')
plt.legend(['gy80 gyro path', 'gy80 attitude path'])


plt.subplot(322)
plt.hold(True)


plt.plot(imu.gyro_path['/vn100/imu'][:,1],
         imu.gyro_path['/vn100/imu'][:,2], 'g.')

plt.plot(imu.attitude_path['/vn100/imu'][:,1],
         imu.attitude_path['/vn100/imu'][:,2], 'r.')

#plt.plot(position['/zio/odometry/rear'][:,1], position['/zio/odometry/rear'][:,2], 'b')
#plt.plot(recalc_position['/zio/odometry/rear'][:,1],
#         recalc_position['/zio/odometry/rear'][:,2], 'g-.')
#plt.legend(['rear', 'rear (recalc)'])
plt.legend(['vn100 gyro path', 'vn100 attitude path'])

plt.subplot(323)
plt.hold(True)

#plt.plot(gnss['/gnss/extended_fix'][:,3] - gnss['/gnss/extended_fix'][1,3], gnss['/gnss/extended_fix'][:,2] - gnss['/gnss/extended_fix'][1,2], 'r')
plt.plot(gnss.data['/gnss/extended_fix'][:,2], gnss.data['/gnss/extended_fix'][:,3], 'r')

plt.subplot(324)
plt.hold(True)

#plt.plot(position.data['/localisation/odometry/filtered'][:,1], position.data['/localisation/odometry/filtered'][:,2], 'b')
plt.plot(odometry.data['/localisation/odometry/filtered'][:, 1],
         odometry.data['/localisation/odometry/filtered'][:, 2], 'b')
plt.plot(odometry.recalculated_data['/localisation/odometry/filtered'][:,1],
         odometry.recalculated_data['/localisation/odometry/filtered'][:,2], 'g.')
#    plt.plot(recalc_position['/localisation/odometry/filtered'][:,1],
#         recalc_position['/localisation/odometry/filtered'][:,2], 'g-.')
plt.legend(['filtered', 'filtered (recalc)'])


plt.subplot(325)
plt.hold(True)
#plt.plot(odometry.data['/gy80/odometry'][:,1], odometry['/gy80/odometry'][:,2], 'b')
plt.plot(odometry.data['/gy80/odometry'][:, 1],
         odometry.data['/gy80/odometry'][:, 2], 'b')
plt.plot(odometry.recalculated_data['/gy80/odometry'][:,1],
         odometry.recalculated_data['/gy80/odometry'][:,2], 'g.')
#plt.plot(recalc_position_constrained['/gy80/odometry'][:,1],
#         recalc_position_constrained['/gy80/odometry'][:,2], 'r-.')
plt.legend(['gy80 odom', 'gy80 recalculated'])













plt.show()

