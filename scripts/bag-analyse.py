#! /usr/bin/env python
import sys
import math
import yaml
import copy
import rospy
import rosbag
import glob
import os

import numpy as np
import pandas as pd
from scipy.optimize import minimize

from bag_analysis.IMU import IMU
from bag_analysis.GNSS import GNSS, GNSSRates
from bag_analysis.Statistics import Statistics
from bag_analysis.Odometry import Odometry
from bag_analysis.PandasAnalysis import PandasAnalysis

from bag_analysis.DataContainer import DataContainer

from mpl_toolkits.mplot3d import Axes3D
from matplotlib import pyplot as plt
import argparse


if __name__=="__main__":
    rospy.init_node('bag_analysis', anonymous=True)

    parser = argparse.ArgumentParser()

    parser.add_argument('-bag', '--input-bag', help='Name of the ROS bag file to analyse. If a folder is selected, the latest bag file in that folder is analysed', required=True)
    parser.add_argument('-kml', '--output-kml-file', help='If given, the position information is output to this KML file to be used in google earth')
    parser.add_argument('-write-file', '--output-to-file', help='Output the graphs to the folder specified by this parameter', required=False)

    parser.add_argument('-yaw', '--show-yaw', help='Plot the yaw information from various sources', action='store_true')
    parser.add_argument('-pitch-roll', '--show-pitch-roll', help='Plot the Pitch and Roll information from various sources', action='store_true')
    parser.add_argument('-yaw-rate', '--show-yaw-rate', help='Plot the yaw rate information from various sources', action='store_true')
    parser.add_argument('-speed', '--show-speed', help='Plot the speed information from various sources', action='store_true')
    parser.add_argument('-acc', '--show-acc', help='Plot the acceleration information from IMU sources', action='store_true')
    parser.add_argument('-stats', '--show-stats', help='Plot the localisation statistical information', action='store_true')
    parser.add_argument('-pos', '--show-position', help='Plot the position information from various sources', action='store_true')
    parser.add_argument('-pos-3d-gnss', '--show-3d-gnss', help='Plot the position information from GNSS sources in 3d', action='store_true')
    parser.add_argument('-pos-3d-odometry', '--show-3d-odometry', help='Plot the position information from odometry sources in 3d', action='store_true')

    parser.add_argument('-pandas', '--run-pandas', help='[DEVEL] run pandas scripts', action='store_true')
    parser.add_argument('-time-sync', '--synchronise-data', help='[DEVEL] optimise the time difference between two data types. This is currently hardcoded to correct the odometry from the vehicle compared to the separate IMU', action='store_true')

    args = parser.parse_args()

    if args.input_bag != "":

        if not (args.show_position or
                    args.show_speed or
                    args.show_yaw or
                    args.synchronise_data or
                    args.show_acc or
                    args.show_stats or
                    args.show_yaw_rate or
                    args.show_pitch_roll or
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


            ros_bag = rosbag.Bag(bag_file_name)
            topics = ros_bag.get_type_and_topic_info()[1].keys()

            # get a dictionary for a list of topics belonging to each type
            types = dict()
            for key, value in ros_bag.get_type_and_topic_info()[1].iteritems():
                if value.msg_type not in types:
                    types[value.msg_type] = list()

                types[value.msg_type].append(key)

            container = DataContainer(ros_bag,
                                      imu=IMU(types.get('sensor_msgs/Imu', list())),
                                      odometry=Odometry(types.get('nav_msgs/Odometry', list())),
                                      gnss=GNSS(types.get('sensor_msgs/NavSatFix', list())),
                                      statistics=Statistics(types.get('dataset_tools/LocaliserStats', list())),
                                      gnss_rates=GNSSRates(types.get('geometry_msgs/TwistWithCovarianceStamped', list())))


            # outputs to KML only if this variable is not an empty string
            output_KML_file = ''
            if args.output_kml_file:
                output_KML_file = args.output_kml_file

            output_graph_folder = ''
            output_graph_type = 'eps'
            output_graph_prefix = ''
            if args.output_to_file:
                output_graph_folder = args.output_to_file
                output_graph_prefix = os.path.splitext(os.path.basename(bag_file_name))[0]

            FIXED_IMU_TIMING = 0.01

            if args.synchronise_data:

                # find the coarse time offset from the last timestamps of each dataset
                non_sync_topic = '/ibeo_interface_node/xsens/IMU'
                sync_topic = '/ibeo_interface_node/ibeo/odometry'
                re_sync_topic = [sync_topic, '/time_sync']

                coarse_offset = container.odometry.data[sync_topic][-1, container.odometry.TIME] - container.imu.data[non_sync_topic][-1, container.imu.TIME]
                print("coarse timing difference is " + str(coarse_offset))

                time_index_a = container.imu.data[non_sync_topic][:, container.imu.TIME]
                a = pd.Series(container.imu.data[non_sync_topic][:, container.imu.YAW_RATE], index=time_index_a)
                a = a[~a.index.duplicated()] 

                def cost_fun(delta_time):
                    time_index_b = container.odometry.data[sync_topic][:, container.odometry.TIME] - coarse_offset + delta_time
                    b = pd.Series(container.odometry.data[sync_topic][:, container.odometry.YAW_RATE], index=time_index_b)
                    b = b[~b.index.duplicated()]
                    b_frame = pd.DataFrame({'b':b, 'a':a})

                    d = b_frame.interpolate('index')
                    e = d.corr()

                    return -1. * e['a']['b']

                initial_value = 0.
                res = minimize(cost_fun, [initial_value], options={"maxiter":100, "disp":True, 'eps':0.01})

                print("graph " ,output_graph_folder , " prefix ", output_graph_prefix)

                if len(output_graph_folder) > 0:
                    output_dict = dict(res)
                    output_dict['x'] = float(res.x[0])
                    output_dict['coarse_time_offset'] = float(coarse_offset)
                    output_dict['total_time_offset'] = float(-coarse_offset + res.x[0])

                    with open(output_graph_folder + "/" + output_graph_prefix + ".time.yaml", 'w') as file:
                        yaml.dump(output_dict, file)

                print(res)

                container.odometry.data[re_sync_topic] = copy.deepcopy(container.odometry.data[sync_topic])
                container.odometry.data[re_sync_topic][:, container.odometry.TIME] = container.odometry.data[re_sync_topic][:, container.odometry.TIME] - coarse_offset + res.x[0]
                container.odometry.topic_list.append(re_sync_topic)


            # plot localiser statistical information
            if args.show_stats:

                for key in container.statistics.data.keys():
                    plt.figure()

                    fig, axs = plt.subplots(1, 3, sharey=True, tight_layout=True)

                    # We can set the number of bins with the `bins` kwarg
                    n_bins = 50
                    axs[0].hist(container.statistics.data[key][:, container.statistics.INNOV_X], bins=n_bins)
                    axs[1].hist(container.statistics.data[key][:, container.statistics.INNOV_Y], bins=n_bins)
                    axs[2].hist(container.statistics.data[key][:, container.statistics.INNOV_YAW], bins=n_bins)

                plt.figure()

                plt.subplot(311)
                plt.xlabel("time (s)")
                plt.ylabel("Error (m)")

                plt.subplot(312)
                plt.xlabel("time (s)")
                plt.ylabel("Error (m)")

                plt.subplot(313)
                plt.xlabel("time (s)")
                plt.ylabel("Error (deg)")

                plt.title("Localiser Statistics")

                legend = []

                for key in container.statistics.data.keys():
                    plt.subplot(311)
                    plt.plot(container.statistics.data[key][:, container.statistics.TIME],
                             np.abs(container.statistics.data[key][:, container.statistics.INNOV_X]), '*')
                    legend.append("Innov X")

                    plt.plot(container.statistics.data[key][:, container.statistics.TIME],
                             np.sqrt(container.statistics.data[key][:, container.statistics.CONFIDENCE_X]), '.-')
                    legend.append("Conf X")

                    plt.subplot(312)
                    plt.plot(container.statistics.data[key][:, container.statistics.TIME],
                             np.abs(container.statistics.data[key][:, container.statistics.INNOV_Y]), '*')
                    legend.append("Innov Y")

                    plt.plot(container.statistics.data[key][:, container.statistics.TIME],
                             np.sqrt(container.statistics.data[key][:, container.statistics.CONFIDENCE_Y]), '.-')
                    legend.append("Conf Y")

                    plt.subplot(313)
                    plt.plot(container.statistics.data[key][:, container.statistics.TIME],
                             np.abs(container.statistics.data[key][:, container.statistics.INNOV_YAW]) * 180. / 3.1415, '*')
                    legend.append("Innov Yaw")

                    plt.plot(container.statistics.data[key][:, container.statistics.TIME],
                             np.sqrt(container.statistics.data[key][:, container.statistics.CONFIDENCE_YAW]) * 180. / 3.1415, '.-')
                    legend.append("Conf Yaw")

                plt.legend(legend)
                if len(output_graph_folder) > 0:
                    plt.savefig(output_graph_folder + "/" + output_graph_prefix + ".statistics." + output_graph_type, format=output_graph_type, dpi=600)

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
                                 container.odometry.data[topic][:, container.odometry.SPEED], '.-')
                        legend.append(topic)

                for topic in container.gnss_rates.topic_list:
                    if len(container.gnss_rates.data[topic]) > 0:
                        plt.plot(container.gnss_rates.data[topic][:, container.gnss_rates.TIME],
                                 container.gnss_rates.data[topic][:, container.gnss_rates.SPEED], '.-')
                        legend.append(topic)

                for topic in container.gnss.topic_list:
                    if len(container.gnss.data[topic]) > 0:
                        plt.plot(container.gnss.data[topic][:, container.gnss.TIME],
                                 container.gnss.data[topic][:, container.gnss.SPEED], '.-')
                        legend.append(topic)

                plt.legend(legend)
                if len(output_graph_folder) > 0:
                    plt.savefig(output_graph_folder + "/" + output_graph_prefix + ".speed." + output_graph_type, format=output_graph_type, dpi=600)


            # plot acceleration rate information
            if args.show_acc:
                plt.figure()
                plt.suptitle("Acceleration from IMU")

                plt.subplot(311)
                plt.xlabel("time (s)")
                plt.ylabel("X acceleration (m/s^2)")

                plt.subplot(312)
                plt.xlabel("time (s)")
                plt.ylabel("Y acceleration (m/s^2)")

                plt.subplot(313)
                plt.xlabel("time (s)")
                plt.ylabel("Z acceleration (m/s^2)")

                legend = []

                for topic in container.imu.topic_list:
                    if len(container.imu.data[topic]) > 0:
                        plt.subplot(311)
                        plt.plot(container.imu.data[topic][:, container.imu.TIME],
                                 container.imu.data[topic][:, container.imu.ACC_X])
                        plt.subplot(312)
                        plt.plot(container.imu.data[topic][:, container.imu.TIME],
                                 container.imu.data[topic][:, container.imu.ACC_Y])
                        plt.subplot(313)
                        plt.plot(container.imu.data[topic][:, container.imu.TIME],
                                 container.imu.data[topic][:, container.imu.ACC_Z])
                        legend.append(topic)

                plt.legend(legend)
                if len(output_graph_folder) > 0:
                    plt.savefig(output_graph_folder + "/" + output_graph_prefix + ".accel." + output_graph_type, format=output_graph_type, dpi=600)


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
                if len(output_graph_folder) > 0:
                    plt.savefig(output_graph_folder + "/" + output_graph_prefix + ".yaw-rate." + output_graph_type, format=output_graph_type, dpi=600)



            # plot show_pitch_roll information
            if args.show_pitch_roll:
                plt.figure()
                plt.suptitle("Pitch/Roll from various sources")

                plt.subplot(211)
                #plt.hold(True)
                plt.xlabel("time (s)")
                plt.ylabel("roll angle (rad)")

                plt.subplot(212)
                #plt.hold(True)
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
                                 np.unwrap(container.imu.data[topic][:, container.imu.ROLL]))

                        plt.subplot(212)
                        plt.plot(container.imu.data[topic][:, container.imu.TIME],
                                 np.cumsum(container.imu.data[topic][:, container.imu.PITCH_RATE] * FIXED_IMU_TIMING))
                        plt.plot(container.imu.data[topic][:, container.imu.TIME],
                                 np.unwrap(container.imu.data[topic][:, container.imu.PITCH]))

                        legend.append(topic+"-gyro")
                        legend.append(topic+"-attitude")

                plt.legend(legend)
                if len(output_graph_folder) > 0:
                    plt.savefig(output_graph_folder + "/" + output_graph_prefix + ".pitch-roll." + output_graph_type, format=output_graph_type, dpi=600)

            # plot show_yaw information
            if args.show_yaw:
                plt.figure()
                plt.title("Yaw from various sources")
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
                                 np.unwrap(container.imu.data[topic][:, container.imu.YAW]))

                        legend.append(topic + "-gyro")
                        legend.append(topic + "-attitude")

                for topic in container.gnss.topic_list:
                    if len(container.gnss.data[topic]) > 0:
                        plt.plot(container.gnss.data[topic][1:, container.gnss.TIME],
                                 np.unwrap(container.gnss.data[topic][1:, container.gnss.HEADING]))

                        legend.append(topic)

                plt.legend(legend)
                if len(output_graph_folder) > 0:
                    plt.savefig(output_graph_folder + "/" + output_graph_prefix + ".yaw." + output_graph_type, format=output_graph_type, dpi=600)


            # plot position information
            if args.show_position:
                plt.figure()
                plt.suptitle("Position from various sources")

                plt.xlabel("x")
                plt.ylabel("y")
                plt.axis('equal')

                legend = []

                for topic in container.odometry.topic_list:
                    if len(container.odometry.data[topic]) > 0 and abs(container.odometry.data[topic][:, container.odometry.X][-1]) <= 100000:
                        plt.plot(container.odometry.data[topic][:, container.odometry.X],
                                 container.odometry.data[topic][:, container.odometry.Y], '.')

                        legend.append(topic)

                if len(legend) > 0:
                    plt.legend(legend)

                if len(output_graph_folder) > 0:
                    plt.savefig(output_graph_folder + "/" + output_graph_prefix + ".position.fix." + output_graph_type, format=output_graph_type, dpi=600)

                plt.figure()
                plt.suptitle("Position from various sources")

                plt.xlabel("east")
                plt.ylabel("north")
                plt.axis('equal')


                legend = []
                plt.axis('equal')
                for topic in container.imu.topic_list:
                    if len(container.imu.data[topic]) > 0:
                        plt.plot(container.imu.gyro_path[topic][:, container.imu.PATH_Y],
                                 container.imu.gyro_path[topic][:, container.imu.PATH_X] * -1., '.')

                        plt.plot(container.imu.attitude_path[topic][:, container.imu.PATH_Y],
                                 container.imu.attitude_path[topic][:, container.imu.PATH_X] * -1., '.')

                        legend.append((topic+'-gyro'))
                        legend.append((topic+'-attitude'))

                if len(legend) > 0:
                    plt.legend(legend)

                if len(output_graph_folder) > 0:
                    plt.savefig(output_graph_folder + "/" + output_graph_prefix + ".position.dr." + output_graph_type, format=output_graph_type, dpi=600)


                plt.figure()
                plt.suptitle("Position from various sources")
                plt.xlabel("GNSS east")
                plt.ylabel("GNSS north")
                plt.axis('equal')

                legend = []
                plt.axis('equal')

                for key in container.statistics.data.keys():
                    plt.plot(container.statistics.data[key][:, container.statistics.OBS_X],
                             container.statistics.data[key][:, container.statistics.OBS_Y], '*')
                    legend.append(key)

                for topic in container.gnss.topic_list:
                    if len(container.gnss.data[topic]) > 0:
                        plt.plot(container.gnss.data[topic][:, container.gnss.EASTING],
                                 container.gnss.data[topic][:, container.gnss.NORTHING], '.-')
                        legend.append(topic)

                for topic in container.odometry.topic_list:
                    if len(container.odometry.data[topic]) > 0 and ('gps' in topic or 'ukf/odometry' in topic or 'gnss' in topic):
                        plt.plot(container.odometry.data[topic][:, container.odometry.X],
                                 container.odometry.data[topic][:, container.odometry.Y], '.-')
                        legend.append(topic)

                if len(legend) > 0:
                    plt.legend(legend)

                if len(output_graph_folder) > 0:
                    plt.savefig(output_graph_folder + "/" + output_graph_prefix + ".position.abs_odom." + output_graph_type, format=output_graph_type, dpi=600)


                plt.figure()
                plt.suptitle("Position from various sources")
                plt.xlabel("east")
                plt.ylabel("north")
                plt.axis('equal')

                legend = []
                plt.axis('equal')

                for key in container.statistics.data.keys():
                    plt.plot(container.statistics.data[key][:, container.statistics.OBS_X],
                             container.statistics.data[key][:, container.statistics.OBS_Y], '*')
                    legend.append(key)

                for topic in container.odometry.topic_list:
                    if len(container.odometry.data[topic]) > 0 and abs(container.odometry.data[topic][:, container.odometry.X][-1]) > 100000:
                        plt.plot(container.odometry.data[topic][:, container.odometry.X],
                                 container.odometry.data[topic][:, container.odometry.Y], '.')

                        legend.append(topic)

                if len(legend) > 0:
                    plt.legend(legend)

                if len(output_graph_folder) > 0:
                    plt.savefig(output_graph_folder + "/" + output_graph_prefix + ".position.odom." + output_graph_type, format=output_graph_type, dpi=600)



            if len(output_KML_file) > 0:
                if len(output_graph_folder) > 0:
                    container.output_KML_path(output_graph_folder + "/" + output_graph_prefix + "." + output_KML_file)
                else:
                    container.output_KML_path(output_KML_file)

            if args.show_3d_odometry:
                for topic in container.odometry.topic_list:
                    if len(container.odometry.data[topic]) > 0:
                        fig = plt.figure()
                        fig.suptitle('3-Dimensional ' + topic + ' position')
                        ax = fig.add_subplot(111, projection='3d')
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
                        plt.axis('equal')
                        plt.plot(container.gnss.data[topic][:, container.gnss.EASTING],
                                 container.gnss.data[topic][:, container.gnss.NORTHING],
                                 container.gnss.data[topic][:, container.gnss.ALTITUDE])

            if args.run_pandas:
                pa = PandasAnalysis()
                pa.run_analysis(container)

            if len(output_graph_folder) == 0:
                plt.show()

