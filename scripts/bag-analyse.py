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

    parser.add_argument('-n', '--input-file', help='Name of the ROS bag file to analyse')
    parser.add_argument('-kml', '--output-kml-file', help='If given, the position information is output to this KML file to be used in google earth')
    args = parser.parse_args()

    if args.input_file != "":

        if not (args.show_position or args.show_speed or args.show_yaw or args.show_yaw_rate or args.output_kml_file):
            print ('Nothing has been selected to plot or output - for more information use --help')
        else:

            container = DataContainer(rosbag.Bag(args.input_file),
                                      steering=Steering([]),
                                      velocity=Velocity([]),
                                      imu=IMU(['/vn100/imu']),
                                      odometry=Odometry(['/localisation_3d/odometry/gps', '/localisation_3d/odometry/filtered', '/zio/odometry/rear']),
                                      gnss=GNSS(['/ublox_gps/fix', '/localisation_3d/gps/filtered']))

            #odometry = Odometry(['/localisation_test/odometry/gps', '/zio/odometry/front', '/zio/odometry/rear', '/localisation_test/odometry/gps'])
            #steering = Steering(['/zio/joint_states'])
            #velocity = Velocity(['/zio/joint_states'])

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



            if False:

                #np.savetxt('/home/stew/gps.csv', gnss.data['/gps/fix'], delimiter=',')
                #np.savetxt('/home/stew/gpsf.csv', gnss.data['/gps/filtered'], delimiter=',')


                fig = plt.figure()
                fig.suptitle('Plot filter odometry output in 3d')
                ax = fig.add_subplot(111, projection='3d')
                plt.hold(True)
                plt.plot(odometry.data['/odometry/gps'][:, 1],
                         odometry.data['/odometry/gps'][:, 2],
                         odometry.data['/odometry/gps'][:, 3])




                fig = plt.figure()
                fig.suptitle('TEST Plot filter odometry output in 3d')
                ax = fig.add_subplot(111, projection='3d')
                plt.hold(True)
                #plt.plot(odometry.data['/localisation_test/odometry/gps'][:, 1],
                #         odometry.data['/localisation_test/odometry/gps'][:, 2],
                #         odometry.data['/localisation_test/odometry/gps'][:, 3])


                if len(gnss.data['/gps/fix']) > 0:
                    fig = plt.figure()
                    fig.suptitle('Plot GNSS output in 3d')
                    ax = fig.add_subplot(111, projection='3d')
                    plt.hold(True)
                    plt.plot(gnss.data['/gps/fix'][:, 2],
                             gnss.data['/gps/fix'][:, 3],
                             gnss.data['/gps/fix'][:, 4])

                    plt.plot(gnss.data['/gps/filtered'][:, 2],
                             gnss.data['/gps/filtered'][:, 3],
                             gnss.data['/gps/filtered'][:, 4])







                plt.figure()
                plt.title("Position from various sources")

                plt.subplot(321)
                plt.hold(True)

                plt.plot(gnss.data['/gps/filtered'][:,2], gnss.data['/gps/filtered'][:,3], 'r')

                #plt.plot(position.data['/localisation_test/odometry/gps'][:,1], position.data['/localisation_test/odometry/gps'][:,2], 'b')
                #plt.plot(odometry.data['/localisation_test/odometry/gps'][:, 1],
                #         odometry.data['/localisation_test/odometry/gps'][:, 2], 'b')
                #plt.plot(odometry.recalculated_data['/localisation_test/odometry/gps'][:,1],
                #         odometry.recalculated_data['/localisation_test/odometry/gps'][:,2], 'g.')
                plt.legend(['gps filtered'])


                plt.subplot(322)
                plt.hold(True)


                plt.plot(imu.gyro_path['/imu/data'][:,1],
                         imu.gyro_path['/imu/data'][:,2], 'g.')

                plt.plot(imu.attitude_path['/imu/data'][:,1],
                         imu.attitude_path['/imu/data'][:,2], 'r.')

                #plt.plot(position['/zio/odometry/rear'][:,1], position['/zio/odometry/rear'][:,2], 'b')
                #plt.plot(recalc_position['/zio/odometry/rear'][:,1],
                #         recalc_position['/zio/odometry/rear'][:,2], 'g-.')
                #plt.legend(['rear', 'rear (recalc)'])
                plt.legend(['vn100 gyro path', 'vn100 attitude path'])

                plt.subplot(323)
                plt.hold(True)
                #plt.plot(gnss['/ublox_gps/fix'][:,3] - gnss['/ublox_gps/fix'][1,3], gnss['/ublox_gps/fix'][:,2] - gnss['/ublox_gps/fix'][1,2], 'r')
                plt.plot(gnss.data['/gps/fix'][:,2], gnss.data['/gps/fix'][:,3], 'r')
                plt.subplot(324)
                plt.hold(True)

                #plt.plot(position.data['/localisation_test/odometry/gps'][:,1], position.data['/localisation_test/odometry/gps'][:,2], 'b')
                plt.plot(odometry.data['/odometry/gps'][:, 1],
                         odometry.data['/odometry/gps'][:, 2], 'b')
                plt.plot(odometry.recalculated_data['/odometry/gps'][:,1],
                         odometry.recalculated_data['/odometry/gps'][:,2], 'g.')
                #    plt.plot(recalc_position['/localisation_test/odometry/gps'][:,1],
                #         recalc_position['/localisation_test/odometry/gps'][:,2], 'g-.')
                plt.legend(['filtered', 'filtered (recalc)'])


                plt.subplot(325)
                plt.hold(True)
                #plt.plot(odometry.data['/gy80/odometry'][:,1], odometry['/gy80/odometry'][:,2], 'b')
                #plt.plot(odometry.data['/gy80/odometry'][:, 1],
                #         odometry.data['/gy80/odometry'][:, 2], 'b')
                #plt.plot(odometry.recalculated_data['/gy80/odometry'][:,1],
                #         odometry.recalculated_data['/gy80/odometry'][:,2], 'g.')
                #plt.plot(recalc_position_constrained['/gy80/odometry'][:,1],
                #         recalc_position_constrained['/gy80/odometry'][:,2], 'r-.')
                plt.legend(['gy80 odom', 'gy80 recalculated'])


            plt.show()

