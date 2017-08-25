import utm
import math

import numpy as np

from BagDataType import BagDataType

class GNSS(BagDataType):

    TIME = 0
    EASTING = 2
    NORTHING = 3
    ALTITUDE = 4
    HEADING = 5
    LATITUDE = 6
    LONGITUDE = 7
    SPEED = 8
    YAW_RATE = 9

    def __init__(self, topic_list):
        self.previous_east = dict()
        self.previous_north = dict()
        self.previous_time = dict()

        BagDataType.__init__(self, topic_list)

    def new_message(self, topic, msg, t):
        east, north, zone, letter = utm.from_latlon(msg.latitude, msg.longitude)

        estimated_heading = 0.
        estimated_speed = 0.
        if topic in self.previous_east:
            time_delta = (msg.header.stamp - self.previous_time[topic]).to_sec()
            if len(self.data[topic]) < 10:
                print time_delta
            if time_delta > 0.5:
                # print (time_delta)
                estimated_heading = math.atan2(north - self.previous_north[topic], east - self.previous_east[topic])
                estimated_speed = math.sqrt(math.pow(north - self.previous_north[topic], 2) + math.pow(east - self.previous_east[topic], 2)) / time_delta

                self.previous_east[topic] = east
                self.previous_north[topic] = north
                self.previous_time[topic] = msg.header.stamp

                self.data[topic].append([msg.header.stamp.to_sec(), 0., east, north, msg.altitude, estimated_heading, msg.latitude, msg.longitude, estimated_speed, 0.])


        else:
            self.previous_east[topic] = east
            self.previous_north[topic] = north
            self.previous_time[topic] = msg.header.stamp



    def estimate_yaw_rate(self):
        for topic in self.topic_list:
            if len(self.data[topic]) > 0:
                delta_heading = np.diff(np.unwrap(self.data[topic][:, self.HEADING]))
                delta_time = np.diff(self.data[topic][:, self.TIME])

                self.data[topic][1:, self.YAW_RATE] = delta_heading / delta_time;



class GNSSRates(BagDataType):

    TIME = 0
    V_X = 1
    V_Y = 2
    V_Z = 3
    SPEED = 4

    def new_message(self, topic, msg, t):
        self.data[topic].append([t.to_sec(),
                                 msg.twist.twist.linear.x,
                                 msg.twist.twist.linear.y,
                                 msg.twist.twist.linear.z,
                                 math.sqrt(math.pow(msg.twist.twist.linear.x, 2) +
                                           math.pow(msg.twist.twist.linear.y, 2) +
                                           math.pow(msg.twist.twist.linear.z, 2))])

