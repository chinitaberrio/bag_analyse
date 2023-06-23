#! /usr/bin/env python

# This program takes the .time.yaml files that come from the bag-analyse.py tool (using the -time-sync option)
# this file must have the same filename prefix as the bag of data. 
# 
# This script will search for all time.yaml that fit the criteria (gate_name), check if the correction has been applied
# and if not, apply the time correction to the appropriate dataset.

import os
import sys
import math
import copy
import glob
import math
import yaml
import rospy
import rosbag

from datetime import datetime

# sync file names for glob
file_set = "gate*time.yaml"

# the name of the gate
gate_name = "gate_south-end_north-end_"

# first, find a list of the prefixes that have time correction data
sync_files = glob.glob(file_set)
#print(sync_files)

messages_to_resync_blindly = ['ibeo/objects', 'ibeo/odometry']

for file in sync_files:
    # extract the prefix of the file name
    file_prefix = file.replace(".time.yaml", "")
    #print(file_prefix)

    synchronised_bag_name = file_prefix + ".synced.bag"

    if os.path.isfile(synchronised_bag_name):
        print(file_prefix + " has already been processed")
        continue

    with open(file, "r") as stream:
        try:
            sync_data = yaml.load(stream)
            total_time_offset = sync_data['total_time_offset']
            dataset_datetime_str = file_prefix.replace(gate_name, '')

            dataset_datetime = datetime.strptime(dataset_datetime_str.split('.')[0], '%Y%m%d_%H%M%S')
            print(str(dataset_datetime) + "," + str(total_time_offset))
            # filename convention is for YYYYMMDD_HHMMSS

            if math.fabs(total_time_offset) > 100:
                # trying to open bag

                with rosbag.Bag(synchronised_bag_name, 'w', compression=rosbag.Compression.LZ4) as outbag:
                    info_dict = yaml.load(rosbag.Bag(file_prefix + ".bag", 'r')._get_yaml_info())
                    topic_list = []
                    for topic in info_dict['topics']:
                        topic_list.append(topic['topic'])

                    #print(topic_list)

                    offset_duration = rospy.Duration(total_time_offset)

                    bag = rosbag.Bag(file_prefix + ".bag")
                    #for topic, msg, t in bag.read_messages(topics=['chatter', 'numbers']):
                    for topic, msg, t in bag.read_messages():
                        
                        if topic == "tf" or topic == "/tf":
                            # assume only one transform per message (from the ibeo datasets)
                            if msg.transforms:
                                if msg.transforms[0].header.frame_id == 'odom' and msg.transforms[0].child_frame_id == 'base_link':
                                    msg.transforms[0].header.stamp = msg.transforms[0].header.stamp + offset_duration
                                    outbag.write(topic, msg, t + offset_duration)
                                else:
                                    outbag.write(topic, msg, t)
                        elif topic in messages_to_resync_blindly:
                            if msg._has_header:
                                msg.header.stamp = msg.header.stamp + offset_duration
                            outbag.write(topic, msg, t + offset_duration)
                        else:
                            outbag.write(topic, msg, t)
                        #print(topic)
                        continue
                    bag.close()            

        except yaml.YAMLError as exc:
            print(exc)

