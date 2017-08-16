#! /usr/bin/env python
import sys
import math
import rosbag

import numpy as np
import pandas as pd

from BagDataType import *
from DataContainer import DataContainer

from mpl_toolkits.mplot3d import Axes3D
from matplotlib import pyplot as plt
import argparse


if __name__=="__main__":

    parser = argparse.ArgumentParser()

    parser.add_argument('-yaw', '--show-yaw', help='Plot the yaw information from various sources', action='store_true')
    parser.add_argument('-yaw-rate', '--show-yaw-rate', help='Plot the yaw rate information from various sources', action='store_true')
    parser.add_argument('-speed', '--show-speed', help='Plot the speed information from various sources', action='store_true')
    parser.add_argument('-pos', '--show-position', help='Plot the position information from various sources', action='store_true')
    parser.add_argument('-pos-3d-gnss', '--show-3d-gnss', help='Plot the position information from GNSS sources in 3d', action='store_true')
    parser.add_argument('-pos-3d-odometry', '--show-3d-odometry', help='Plot the position information from odometry sources in 3d', action='store_true')

    parser.add_argument('-bag', '--input-bag', help='Name of the ROS bag file to analyse')
    parser.add_argument('-kml', '--output-kml-file', help='If given, the position information is output to this KML file to be used in google earth')
    args = parser.parse_args()

    if args.input_bag != "":

        if not (args.show_position or
                    args.show_speed or
                    args.show_yaw or
                    args.show_yaw_rate or
                    args.show_3d_gnss or
                    args.show_3d_odometry or
                    args.output_kml_file):
            print ('Nothing has been selected to plot or output - for more information use --help')
        else:

            container = DataContainer(rosbag.Bag(args.input_bag),
                                      steering=Steering([]),
                                      velocity=Velocity([]),
                                      imu=IMU(['/vn100/imu']),
                                      odometry=Odometry(['/localisation_3d/odometry/gps', '/localisation_3d/odometry/filtered', '/zio/odometry/rear']),
                                      gnss=GNSS(['/ublox_gps/fix', '/localisation_3d/gps/filtered']))

            # np.savetxt('/home/stew/gps.csv', gnss.data['/gps/fix'], delimiter=',')
            # np.savetxt('/home/stew/gpsf.csv', gnss.data['/gps/filtered'], delimiter=',')

            # outputs to KML only if this variable is not an empty string
            output_KML_file = ''
            if args.output_kml_file:
                output_KML_file = args.output_kml_file

            plot_velocity = True
            plot_yaw_rate = True
            plot_yaw = True
            plot_position = True


            FIXED_IMU_TIMING = 0.01

            # plot speed information
            if args.show_speed:
                plt.figure()
                plt.title("Speed from various sources")
                plt.xlabel("time (s)")
                plt.ylabel("speed (m/s)")

                legend = []

                for topic in container.odometry.topic_list:
                    if len(container.odometry.data[topic]) > 0:
                        plt.plot(container.odometry.data[topic][:, 0],
                                 container.odometry.data[topic][:, 7])
                        legend.append(topic)

                plt.legend(legend)

            # plot yaw rate information
            if args.show_yaw_rate:
                plt.figure()
                plt.title("Yaw rate from various sources")
                plt.xlabel("time (s)")
                plt.ylabel("angular velocity (rad/s)")

                legend = []

                for topic in container.odometry.topic_list:
                    if len(container.odometry.data[topic]) > 0:
                        plt.plot(container.odometry.data[topic][:, 0],
                                 container.odometry.data[topic][:, 8])
                        legend.append(topic)

                for topic in container.imu.topic_list:
                    if len(container.imu.data[topic]) > 0:
                        plt.plot(container.imu.data[topic][:, 0],
                                 container.imu.data[topic][:, 10])
                        legend.append(topic)

                plt.legend(legend)


            # plot yaw information
            if args.show_yaw:
                plt.figure()
                plt.suptitle("Yaw from various sources")

                plt.subplot(311)
                plt.hold(True)
                plt.xlabel("time (s)")
                plt.ylabel("roll angle (rad)")

                plt.subplot(312)
                plt.hold(True)
                plt.xlabel("time (s)")
                plt.ylabel("pitch angle (rad)")

                plt.subplot(313)
                plt.hold(True)
                plt.xlabel("time (s)")
                plt.ylabel("yaw angle (rad)")

                legend = []

                for topic in container.odometry.topic_list:
                    if len(container.odometry.data[topic]) > 0:
                        plt.subplot(311)
                        plt.plot(container.odometry.data[topic][:, 0],
                                 np.unwrap(container.odometry.data[topic][:, 4]))

                        plt.subplot(312)
                        plt.plot(container.odometry.data[topic][:, 0],
                                 np.unwrap(container.odometry.data[topic][:, 5]))

                        plt.subplot(313)
                        plt.plot(container.odometry.data[topic][:, 0],
                                 np.unwrap(container.odometry.data[topic][:, 6]))

                        legend.append(topic)

                for topic in container.imu.topic_list:
                    if len(container.imu.data[topic]) > 0:
                        plt.subplot(311)
                        plt.plot(container.imu.data[topic][:, 0],
                                 np.cumsum(container.imu.data[topic][:, 8] * FIXED_IMU_TIMING))
                        plt.plot(container.imu.data[topic][:, 0],
                                 np.unwrap(container.imu.data[topic][:, 5]), 'g')

                        plt.subplot(312)
                        plt.plot(container.imu.data[topic][:, 0],
                                 np.cumsum(container.imu.data[topic][:, 9] * FIXED_IMU_TIMING))
                        plt.plot(container.imu.data[topic][:, 0],
                                 np.unwrap(container.imu.data[topic][:, 6]), 'g')

                        plt.subplot(313)
                        plt.plot(container.imu.data[topic][:, 0],
                                 np.cumsum(container.imu.data[topic][:, 10] * FIXED_IMU_TIMING))
                        plt.plot(container.imu.data[topic][:, 0],
                                 np.unwrap(container.imu.data[topic][:, 7]), 'g')

                        legend.append(topic+"-gyro")
                        legend.append(topic+"-attitude")

                for topic in container.gnss.topic_list:
                    if len(container.gnss.data[topic]) > 0:
                        plt.subplot(313)
                        plt.plot(container.gnss.data[topic][1:, 0],
                                 np.unwrap(container.gnss.data[topic][1:, 5]))

                        legend.append(topic)

                plt.legend(legend)

            # plot position information
            if args.show_position:
                plt.figure()
                plt.suptitle("Position from various sources")

                plt.subplot(311)
                plt.hold(True)
                plt.xlabel("east")
                plt.ylabel("north")
                plt.axis('equal')

                plt.subplot(312)
                plt.hold(True)
                plt.xlabel("east")
                plt.ylabel("north")
                plt.axis('equal')

                plt.subplot(313)
                plt.hold(True)
                plt.xlabel("GNSS east")
                plt.ylabel("GNSS north")
                plt.axis('equal')

                legend = []
                for topic in container.odometry.topic_list:
                    if len(container.odometry.data[topic]) > 0:
                        plt.subplot(311)
                        plt.plot(container.odometry.data[topic][:, 2],
                                 container.odometry.data[topic][:, 1] * -1.)

                        legend.append(topic)

                if len(legend) > 0:
                    plt.legend(legend)

                legend = []
                for topic in container.imu.topic_list:
                    if len(container.imu.data[topic]) > 0:
                        plt.subplot(312)
                        plt.plot(container.imu.gyro_path[topic][:, 2],
                                 container.imu.gyro_path[topic][:, 1] * -1.)

                        plt.plot(container.imu.attitude_path[topic][:, 2],
                                 container.imu.attitude_path[topic][:, 1] * -1.)

                        legend.append((topic+'-gyro'))
                        legend.append((topic+'-attitude'))

                if len(legend) > 0:
                    plt.legend(legend)

                legend = []
                for topic in container.gnss.topic_list:
                    if len(container.gnss.data[topic]) > 0:
                        plt.subplot(313)
                        plt.plot(container.gnss.data[topic][:, 2],
                                 container.gnss.data[topic][:, 3])

                        legend.append(topic)

                if len(legend) > 0:
                    plt.legend(legend)


            if len(output_KML_file) > 0:
                container.output_KML_path(output_KML_file)

            if args.show_3d_odometry:
                for topic in container.odometry.topic_list:
                    if len(container.odometry.data[topic]) > 0:
                        fig = plt.figure()
                        fig.suptitle('3-Dimensional ' + topic + ' position')
                        ax = fig.add_subplot(111, projection='3d')
                        plt.hold(True)
                        plt.axis('equal')
                        plt.plot(container.odometry.data[topic][:, 1],
                                 container.odometry.data[topic][:, 2],
                                 container.odometry.data[topic][:, 3])

            if args.show_3d_gnss:
                for topic in container.gnss.topic_list:
                    if len(container.gnss.data[topic]) > 0:
                        fig = plt.figure()
                        fig.suptitle('3-Dimensional ' + topic + ' position')
                        ax = fig.add_subplot(111, projection='3d')
                        plt.hold(True)
                        plt.axis('equal')
                        plt.plot(container.gnss.data[topic][:, 2],
                                 container.gnss.data[topic][:, 3],
                                 container.gnss.data[topic][:, 4])

            plt.show()

