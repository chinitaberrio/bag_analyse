#! /usr/bin/env python
import sys
import math
import rospy
import rosbag
import glob
import os

import numpy as np
import pandas as pd

from IMU import IMU
from GNSS import GNSS, GNSSRates
from Odometry import Odometry
from PandasAnalysis import PandasAnalysis
from VehicleState import Velocity, Steering

from DataContainer import DataContainer

from mpl_toolkits.mplot3d import Axes3D
from matplotlib import pyplot as plt
import argparse


if __name__=="__main__":
    rospy.init_node('bag_analysis', anonymous=True)

    parser = argparse.ArgumentParser()

    parser.add_argument('-bag', '--input-bag', help='Name of the ROS bag file to analyse. If a folder is selected, the latest bag file in that folder is analysed', required=True)
    parser.add_argument('-kml', '--output-kml-file', help='If given, the position information is output to this KML file to be used in google earth')

    parser.add_argument('-yaw', '--show-yaw', help='Plot the yaw information from various sources', action='store_true')
    parser.add_argument('-pitch-roll', '--show-pitch-roll', help='Plot the Pitch and Roll information from various sources', action='store_true')
    parser.add_argument('-yaw-rate', '--show-yaw-rate', help='Plot the yaw rate information from various sources', action='store_true')
    parser.add_argument('-speed', '--show-speed', help='Plot the speed information from various sources', action='store_true')
    parser.add_argument('-pos', '--show-position', help='Plot the position information from various sources', action='store_true')
    parser.add_argument('-pos-3d-gnss', '--show-3d-gnss', help='Plot the position information from GNSS sources in 3d', action='store_true')
    parser.add_argument('-pos-3d-odometry', '--show-3d-odometry', help='Plot the position information from odometry sources in 3d', action='store_true')

    parser.add_argument('-pandas', '--run-pandas', help='[DEVEL] run pandas scripts', action='store_true')

    args = parser.parse_args()

    if args.input_bag != "":

        if not (args.show_position or
                    args.show_speed or
                    args.show_yaw or
                    args.show_yaw_rate or
                    args.show_3d_gnss or
                    args.show_3d_odometry or
                    args.run_pandas or
                    args.output_kml_file):
            rospy.logerr('Nothing has been selected to plot or output - for more information use --help')
        else:

            if os.path.isfile(args.input_bag):
                bag_file_name = args.input_bag
            elif os.path.isdir(args.input_bag):
                list_of_files = glob.glob(args.input_bag + '/*.bag')
                bag_file_name = max(list_of_files, key=os.path.getctime) # pick the latest file
                rospy.loginfo('latest file in folder ' + args.input_bag + ' is ' + bag_file_name)

            container = DataContainer(rosbag.Bag(bag_file_name),
                                      steering=Steering([]),
                                      velocity=Velocity([]),
                                      imu=IMU(['/vn100/imu', 'xsens/IMU']),
                                      odometry=Odometry(['/localisation_3d/odometry/gps', '/localisation_3d/odometry/filtered', '/localisation_debug/odometry/gps', '/localisation_debug/odometry/filtered', '/zio/odometry/rear', 'ibeo/odometry']),
                                      gnss=GNSS(['/ublox_gps/fix', '/localisation_3d/gps/filtered', '/localisation_debug/gps/filtered', 'ibeo/gnss']),
                                      gnss_rates=GNSSRates(['/ublox_gps/fix_velocity']))

            # np.savetxt('/home/stew/gps.csv', gnss.data['/gps/fix'], delimiter=',')
            # np.savetxt('/home/stew/gpsf.csv', gnss.data['/gps/filtered'], delimiter=',')

            print (container.odometry.data['ibeo/odometry'])
            print (container.gnss.data['ibeo/gnss'])
            print (container.imu.data['xsens/IMU'])

            # outputs to KML only if this variable is not an empty string
            output_KML_file = ''
            if args.output_kml_file:
                output_KML_file = args.output_kml_file


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
                        plt.plot(container.odometry.data[topic][:, container.odometry.TIME],
                                 container.odometry.data[topic][:, container.odometry.SPEED])
                        legend.append(topic)

                for topic in container.gnss_rates.topic_list:
                    if len(container.gnss_rates.data[topic]) > 0:
                        plt.plot(container.gnss_rates.data[topic][:, container.gnss_rates.TIME],
                                 container.gnss_rates.data[topic][:, container.gnss_rates.SPEED])
                        legend.append(topic)

                for topic in container.gnss.topic_list:
                    if len(container.gnss.data[topic]) > 0:
                        plt.plot(container.gnss.data[topic][:, container.gnss.TIME],
                                 container.gnss.data[topic][:, container.gnss.SPEED])
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
                        plt.plot(container.odometry.data[topic][:, container.odometry.TIME],
                                 container.odometry.data[topic][:, container.odometry.YAW_RATE])

                        legend.append(topic)

                for topic in container.imu.topic_list:
                    if len(container.imu.data[topic]) > 0:
                        plt.plot(container.imu.data[topic][:, container.imu.TIME],
                                 container.imu.data[topic][:, container.imu.YAW_RATE])
                        legend.append(topic)


                for topic in container.gnss.topic_list:
                    if len(container.gnss.data[topic]) > 0:
                        plt.plot(container.gnss.data[topic][:, container.gnss.TIME],
                                 container.gnss.data[topic][:, container.gnss.YAW_RATE])
                        legend.append(topic)

                plt.legend(legend)


            # plot show_pitch_roll information
            if args.show_pitch_roll:
                plt.figure()
                plt.suptitle("Pitch/Roll from various sources")

                plt.subplot(211)
                plt.hold(True)
                plt.xlabel("time (s)")
                plt.ylabel("roll angle (rad)")

                plt.subplot(212)
                plt.hold(True)
                plt.xlabel("time (s)")
                plt.ylabel("pitch angle (rad)")

                legend = []

                for topic in container.odometry.topic_list:
                    if len(container.odometry.data[topic]) > 0:
                        plt.subplot(211)
                        plt.plot(container.odometry.data[topic][:, container.odometry.TIME],
                                 np.unwrap(container.odometry.data[topic][:, container.odometry.ROLL]))

                        plt.subplot(212)
                        plt.plot(container.odometry.data[topic][:, container.odometry.TIME],
                                 np.unwrap(container.odometry.data[topic][:, container.odometry.PITCH]))

                        legend.append(topic)

                for topic in container.imu.topic_list:
                    if len(container.imu.data[topic]) > 0:
                        plt.subplot(211)
                        plt.plot(container.imu.data[topic][:, container.imu.TIME],
                                 np.cumsum(container.imu.data[topic][:, container.imu.ROLL_RATE] * FIXED_IMU_TIMING))
                        plt.plot(container.imu.data[topic][:, container.imu.TIME],
                                 np.unwrap(container.imu.data[topic][:, container.imu.ROLL]), 'g')

                        plt.subplot(212)
                        plt.plot(container.imu.data[topic][:, container.imu.TIME],
                                 np.cumsum(container.imu.data[topic][:, container.imu.PITCH_RATE] * FIXED_IMU_TIMING))
                        plt.plot(container.imu.data[topic][:, container.imu.TIME],
                                 np.unwrap(container.imu.data[topic][:, container.imu.PITCH]), 'g')

                        legend.append(topic+"-gyro")
                        legend.append(topic+"-attitude")

                plt.legend(legend)

            # plot show_yaw information
            if args.show_yaw:
                plt.figure()
                plt.title("Yaw from various sources")
                plt.hold(True)
                plt.xlabel("time (s)")
                plt.ylabel("yaw angle (rad)")

                legend = []

                for topic in container.odometry.topic_list:
                    if len(container.odometry.data[topic]) > 0:
                        plt.plot(container.odometry.data[topic][:, container.odometry.TIME],
                                 np.unwrap(container.odometry.data[topic][:, container.odometry.YAW]))

                        legend.append(topic)

                for topic in container.imu.topic_list:
                    if len(container.imu.data[topic]) > 0:
                        plt.plot(container.imu.data[topic][:, container.imu.TIME],
                                 np.cumsum(container.imu.data[topic][:, container.imu.YAW_RATE] * FIXED_IMU_TIMING))
                        plt.plot(container.imu.data[topic][:, container.imu.TIME],
                                 np.unwrap(container.imu.data[topic][:, container.imu.YAW]), 'g')

                        legend.append(topic + "-gyro")
                        legend.append(topic + "-attitude")

                for topic in container.gnss.topic_list:
                    if len(container.gnss.data[topic]) > 0:
                        plt.plot(container.gnss.data[topic][1:, container.gnss.TIME],
                                 np.unwrap(container.gnss.data[topic][1:, container.gnss.HEADING]))

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
                        plt.plot(container.odometry.data[topic][:, container.odometry.Y],
                                 container.odometry.data[topic][:, container.odometry.X] * -1., '.')

                        legend.append(topic)

                if len(legend) > 0:
                    plt.legend(legend)

                legend = []
                for topic in container.imu.topic_list:
                    if len(container.imu.data[topic]) > 0:
                        plt.subplot(312)
                        plt.plot(container.imu.gyro_path[topic][:, container.imu.PATH_Y],
                                 container.imu.gyro_path[topic][:, container.imu.PATH_X] * -1., '.')

                        plt.plot(container.imu.attitude_path[topic][:, container.imu.PATH_Y],
                                 container.imu.attitude_path[topic][:, container.imu.PATH_X] * -1., '.')

                        legend.append((topic+'-gyro'))
                        legend.append((topic+'-attitude'))

                if len(legend) > 0:
                    plt.legend(legend)

                legend = []
                for topic in container.gnss.topic_list:
                    if len(container.gnss.data[topic]) > 0:
                        plt.subplot(313)
                        plt.plot(container.gnss.data[topic][:, container.gnss.EASTING],
                                 container.gnss.data[topic][:, container.gnss.NORTHING], '.')

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
                        plt.plot(container.odometry.data[topic][:, container.odometry.X],
                                 container.odometry.data[topic][:, container.odometry.Y],
                                 container.odometry.data[topic][:, container.odometry.Z])

            if args.show_3d_gnss:
                for topic in container.gnss.topic_list:
                    if len(container.gnss.data[topic]) > 0:
                        fig = plt.figure()
                        fig.suptitle('3-Dimensional ' + topic + ' position')
                        ax = fig.add_subplot(111, projection='3d')
                        plt.hold(True)
                        plt.axis('equal')
                        plt.plot(container.gnss.data[topic][:, container.gnss.EASTING],
                                 container.gnss.data[topic][:, container.gnss.NORTHING],
                                 container.gnss.data[topic][:, container.gnss.ALTITUDE])

            if args.run_pandas:
                pa = PandasAnalysis()
                pa.run_analysis(container)


            plt.show()

