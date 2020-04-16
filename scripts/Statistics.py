import tf
import math
import numpy as np

from BagDataType import BagDataType

# IMU has an additional field which is filled in with the latest speed measurement
#  This enables the recalculation of odometry information using yaw and speed
class Statistics(BagDataType):

    TIME = 0
    INNOV_X = 1
    INNOV_Y = 2
    INNOV_YAW = 3
    CONFIDENCE_X = 4
    CONFIDENCE_Y = 5
    CONFIDENCE_YAW = 6
    COVAR_X_X = 7
    COVAR_X_Y = 8
    COVAR_X_YAW = 9
    COVAR_Y_X = 10
    COVAR_Y_Y = 11
    COVAR_Y_YAW = 12
    COVAR_YAW_X = 13
    COVAR_YAW_Y = 14
    COVAR_YAW_YAW = 15

    def new_message(self, topic, msg, t):

        self.data[topic].append([msg.header.stamp.to_sec(),
                                 msg.innovation.x, msg.innovation.y, msg.innovation.yaw,
                                 msg.confidence.x, msg.confidence.y, msg.confidence.yaw,
                                 msg.covariance[0],
                                 msg.covariance[1],
                                 msg.covariance[2],
                                 msg.covariance[3],
                                 msg.covariance[4],
                                 msg.covariance[5],
                                 msg.covariance[6],
                                 msg.covariance[7],
                                 msg.covariance[8]])

