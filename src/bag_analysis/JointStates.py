from BagDataType import BagDataType

import numpy as np


class JointStates(BagDataType):
    TIME = 0
    POSITION = 1
    VELOCITY = 2
    EFFORT = 3

    def __init__(self, topic_list):
        self.topic_list = topic_list
        self.joint_names = []
        self.data = dict()
        for topic in topic_list:
            self.data[topic] = dict()

    def new_message(self, topic, msg, t):
        for name, position, velocity, effort in zip(msg.name, msg.position, msg.velocity, msg.effort):
            if name not in self.joint_names:
                self.joint_names.append(name)
                self.data[topic][name] = []

            self.data[topic][name].append(
                [t.to_sec(), position, velocity, effort])

    def convert_numpy(self):
        for topic in self.topic_list:
            for joint_name in self.joint_names:
                self.data[topic][joint_name] = np.array(
                    self.data[topic][joint_name])
