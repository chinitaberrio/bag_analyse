import rospy
import numpy as np
import pandas as pd
from matplotlib import pyplot as plt

from IMU import IMU
from GNSS import GNSS, GNSSRates
from Odometry import Odometry
from VehicleState import Velocity, Steering

from DataContainer import DataContainer


class PandasAnalysis:
    def __init__(self):
        pass

    def run_analysis(self, container):

        gnss_topic = 'ibeo/gnss'
        odo_topic = 'ibeo/odometry'
        imu_topic = 'xsens/IMU'

        gnss_speed = pd.Series(data=container.gnss.data[gnss_topic][:,container.gnss.SPEED],
                               index=container.gnss.data[gnss_topic][:,container.gnss.TIME])

        odo_speed = pd.Series(data=container.odometry.data[odo_topic][:, container.odometry.SPEED],
                               index=container.odometry.data[odo_topic][:, container.odometry.TIME])

        #rospy.loginfo(gnss_speed)
        #rospy.loginfo(odo_speed)

        plt.figure()
        plt.subplot(231)
        plt.title("pandas speed analysis")
        plt.xlabel("time (s)")
        plt.ylabel("speed m/s")

        gnss_speed.plot()
        odo_speed.plot()
        reindexed = odo_speed.reindex(gnss_speed.index, method='nearest')
        reindexed.plot()

        rospy.loginfo(reindexed)

        plt.subplot(234)
        plt.title("pandas speed deltas")
        plt.xlabel("time (s)")
        plt.ylabel("speed m/s")
        np.abs(gnss_speed - reindexed).plot(style='*')

        gnss_heading = pd.Series(data=container.gnss.data[gnss_topic][:,container.gnss.HEADING],
                               index=container.gnss.data[gnss_topic][:,container.gnss.TIME])

        imu_heading = pd.Series(data=container.imu.data[imu_topic][:, container.imu.YAW],
                               index=container.imu.data[imu_topic][:, container.imu.TIME])

        gnss_yaw_rate = pd.Series(data=container.gnss.data[gnss_topic][:,container.gnss.YAW_RATE],
                               index=container.gnss.data[gnss_topic][:,container.gnss.TIME])

        imu_yaw_rate = pd.Series(data=container.imu.data[imu_topic][:, container.imu.YAW_RATE],
                               index=container.imu.data[imu_topic][:, container.imu.TIME])


        plt.subplot(232)

        gnss_heading.plot()
        imu_heading.plot()
        reindexed_heading = imu_heading.reindex(gnss_heading.index, method='nearest')
        reindexed_heading.plot()

        plt.subplot(235)
        plt.title("pandas heading deltas")
        plt.xlabel("time (s)")
        plt.ylabel("heading rad")
        np.abs(gnss_heading - reindexed_heading).plot(style='*')


        plt.subplot(233)
        gnss_yaw_rate.plot()
        imu_yaw_rate.plot()
        reindexed_yaw_rate = imu_yaw_rate.reindex(gnss_yaw_rate.index, method='nearest')
        reindexed_yaw_rate.plot()

        plt.subplot(236)
        plt.title("pandas heading rate deltas")
        plt.xlabel("time (s)")
        plt.ylabel("heading rad")
        np.abs(gnss_yaw_rate - reindexed_yaw_rate).plot(style='*')



        #plt.plot(gnss_speed.index, gnss_speed.data)
        #plt.plot(odo_speed.index, odo_speed.data)