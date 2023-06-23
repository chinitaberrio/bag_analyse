import tf
import utm
import math
import simplekml


import numpy as np



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
        deltas = np.diff(odometry[:, 0])
        cumulative_x = initial_x
        cumulative_y = initial_y
        cumulative_theta = initial_theta

        for (t, x, y, z, roll, pitch, yaw, v, yaw_rate), dt in zip(odometry[1:, :], deltas):
            if not stationary_constraint or v > 0.:
                #dt = 0.1
                cumulative_theta += dt * yaw_rate
                cumulative_x += math.cos(cumulative_theta) * v * dt
                cumulative_y += math.sin(cumulative_theta) * v * dt
            yield ([t, cumulative_x, cumulative_y, cumulative_theta, yaw_rate, v])

    def add_KML_path(self, kml, label, east_base, east_vector, north_base, north_vector, colour, zone, letter):
        ls = kml.newlinestring(name=str(label))
        ls.style.linestyle.width = 4

        path = []
        for east, north in zip(east_vector, north_vector):
            lat, lon = utm.to_latlon(east - east_base, north - north_base, zone, letter)
            path.append((lon, lat))

        ls.coords = path
        ls.style.linestyle.color = colour



