#/usr/bin/python2.4

import utm
import math
import psycopg2
import simplekml
import intervaltree

import numpy as np
from shapely.geometry import asLineString

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

from BagDB import BagDB


if __name__ == "__main__":

    analysis = BagDB()

    output_all_traces = True
    output_close_to_uni_analysis = False
    output_bridge_analysis = False

    query_data = (151.202, -33.870, 151.22, -33.833)

    odom_query = """select EXTRACT(EPOCH FROM time), twist_linear_x, twist_angular_z, pose_position_x, pose_position_y from bag_odometry
                               where bagid = %s and time > %s and time < %s
                               order by time;"""

    imu_query = """select EXTRACT(EPOCH FROM time), linear_acceleration_z, angular_velocity_z from bag_imu
                        where bagid = %s and time > %s and time < %s
                        order by time; """


    if output_all_traces:
        # output all of the data
        traces = analysis.ExtractSeparateTraces(analysis.position_query, ())
        #traces = ExtractSeparateTraces(restricted_position_query, (151.0, -33.96, 151.27, -33.8))
        analysis.OutputPathsToKML(traces, '/home/stew/420_paths_part_2.kml')


    if output_close_to_uni_analysis:
        uni_samples = analysis.LoadData(analysis.restricted_position_query, (151.1900, -33.8902, 151.1927, -33.8884), ((-33.8892, -33.8889, 151.1922, 151.19234), (-33.8891, -33.8883, 151.1912, 151.1914)), '/home/stew/uni-paths.kml')
        uni_imu_sensor_data = analysis.PerformAnalysis(uni_samples, imu_query, limit_samples=1e5, direction='north')
        uni_odom_sensor_data = analysis.PerformAnalysis(uni_samples, odom_query, limit_samples=1e5, direction='north')

        """
        # plot the bridge data
        fig = plt.figure()
        fig.suptitle('ROSbag database sensor data plot against position')
        ax = fig.add_subplot(211, projection='3d')
        plt.hold(True)
        """

        fig = plt.figure()
        plt.subplot(211)
        plt.xlabel('Distance travelled (m)')
        plt.ylabel('z-Acceleration (m/s^2)')
        plt.title('Sensor data from the IMU plot against position information')

        for bag_id, uni_sensor_data in uni_imu_sensor_data.iteritems():

            distance = list()
            distance.append(0.)
            previous = (0., 0.)
            for sample in uni_sensor_data:
                if previous != (0.,0.):
                    distance.append(distance[-1] + math.sqrt(math.pow(previous[0]-sample[5], 2) + math.pow(previous[1]-sample[6], 2)))
                previous = (sample[5], sample[6])

            plt.plot(distance, uni_sensor_data[:, 1])

            """
            plt.xlabel('east (m)')
            plt.ylabel('north (m)')
            plt.title('z-acceleration from imu')
            plt.plot(uni_sensor_data[:, 5] - uni_sensor_data[0, 5], uni_sensor_data[:, 6] - uni_sensor_data[0, 6], uni_sensor_data[:, 1])
            """

        # unwrap the data into distance travelled




        # plot the bridge data
        #fig = plt.figure()
        plt.subplot(212)
        plt.xlabel('Distance travelled (m)')
        plt.ylabel('Speed (m/s)')
        plt.title('Sensor data from the vehicle odometry plot against position information')

        """
        ax = fig.add_subplot(212, projection='3d')
        plt.hold(True)
        """


        for bag_id, uni_sensor_data in uni_odom_sensor_data.iteritems():

            distance = list()
            distance.append(0.)
            previous = (0., 0.)
            for sample in uni_sensor_data:
                if previous != (0.,0.):
                    distance.append(distance[-1] + math.sqrt(math.pow(previous[0]-sample[5], 2) + math.pow(previous[1]-sample[6], 2)))
                previous = (sample[5], sample[6])

            plt.plot(distance, uni_sensor_data[:, 1] * 3.6)

            """
            plt.xlabel('east (m)')
            plt.ylabel('north (m)')
            plt.title('velocity from odometery')
            plt.plot(uni_sensor_data[:, 5] - uni_sensor_data[0, 5], uni_sensor_data[:, 6] - uni_sensor_data[0, 6], uni_sensor_data[:, 1] * 3.6)
            """




        # plot data in 3D
        fig = plt.figure()
        fig.suptitle('ROSbag database sensor data plot against position')
        ax = fig.add_subplot(211, projection='3d')
        plt.hold(True)

        for bag_id, uni_sensor_data in uni_imu_sensor_data.iteritems():

            plt.xlabel('east (m)')
            plt.ylabel('north (m)')
            plt.title('z-acceleration from imu')
            plt.plot(uni_sensor_data[:, 5] - uni_sensor_data[0, 5], uni_sensor_data[:, 6] - uni_sensor_data[0, 6], uni_sensor_data[:, 1])

        ax = fig.add_subplot(212, projection='3d')

        for bag_id, uni_sensor_data in uni_odom_sensor_data.iteritems():

            plt.xlabel('east (m)')
            plt.ylabel('north (m)')
            plt.title('vehicle speed from imu')
            plt.plot(uni_sensor_data[:, 5] - uni_sensor_data[0, 5], uni_sensor_data[:, 6] - uni_sensor_data[0, 6], uni_sensor_data[:, 1] * 3.6)




    if output_bridge_analysis:
        # output the odometry data for driving over the harbour bridge
        bridge_samples = analysis.LoadData(analysis.restricted_position_query, query_data, ((-33.87, -33.867, 151.2, 151.205),), '/home/stew/bridge_positions.kml')
        north_sensor_data = analysis.PerformAnalysis(bridge_samples, odom_query, limit_samples = 1e5, direction="north")
        south_sensor_data = analysis.PerformAnalysis(bridge_samples, odom_query, limit_samples = 1e5, direction="south")

        """
        # plot the bridge data
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
        plt.hold(True)
    
        for bag_id, north_sensor_data in north_sensor_data.iteritems():
            plt.plot(north_sensor_data[:,3], north_sensor_data[:,4], north_sensor_data[:,1] * 3.6)
    
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
        plt.hold(True)
    
        for bag_id, south_sensor_data in south_sensor_data.iteritems():
            plt.plot(south_sensor_data[:,3], south_sensor_data[:,4], south_sensor_data[:,1] * 3.6)
    
        """
        fig = plt.figure()

        plt.subplot(211)
        plt.xlabel('Distance travelled (m)')
        plt.ylabel('Velocity (km/h)')
        plt.title('Southbound journeys')

        for bag_id, uni_sensor_data in north_sensor_data.iteritems():

            distance = list()
            distance.append(0.)
            previous = (0., 0.)
            for sample in uni_sensor_data:
                if previous != (0., 0.):
                    distance.append(distance[-1] + math.sqrt(
                        math.pow(previous[0] - sample[5], 2) + math.pow(previous[1] - sample[6], 2)))
                previous = (sample[5], sample[6])

            plt.plot(distance, uni_sensor_data[:, 1] * 3.6)



        # plot the bridge data
        # fig = plt.figure()
        plt.subplot(212)
        plt.xlabel('Distance travelled (m)')
        plt.ylabel('Velocity (km/h)')
        plt.title('Northbound journeys')

        for bag_id, uni_sensor_data in south_sensor_data.iteritems():

            distance = list()
            distance.append(0.)
            previous = (0., 0.)
            for sample in uni_sensor_data:
                if previous != (0., 0.):
                    distance.append(distance[-1] + math.sqrt(
                        math.pow(previous[0] - sample[5], 2) + math.pow(previous[1] - sample[6], 2)))
                previous = (sample[5], sample[6])

            plt.plot(distance, uni_sensor_data[:, 1] * 3.6)




    plt.show()
