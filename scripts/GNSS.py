import tf
import utm
import math
import simplekml
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


class GNSSRates(BagDataType):

    TIME = 0
    V_X = 2
    V_Y = 3
    V_Z = 4
    SPEED = 5

    def new_message(self, topic, msg, t):
        self.data[topic].append([t.to_sec(),
                                 msg.twist.twist.linear.x,
                                 msg.twist.twist.linear.y,
                                 msg.twist.twist.linear.z,
                                 math.sqrt(math.pow(msg.twist.twist.linear.x, 2) +
                                           math.pow(msg.twist.twist.linear.y, 2) +
                                           math.pow(msg.twist.twist.linear.z, 2))])

