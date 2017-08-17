import tf
import utm
import math
import simplekml
import numpy as np

from BagDataType import BagDataType


class Steering(BagDataType):
    def new_message(self, topic, msg, t):
        self.data[topic].append([t.to_sec(), msg.position[2]])


class Velocity(BagDataType):
    def new_message(self, topic, msg, t):
        self.data[topic].append([t.to_sec(), msg.velocity[0], msg.velocity[1], msg.velocity[2], msg.velocity[3]])


