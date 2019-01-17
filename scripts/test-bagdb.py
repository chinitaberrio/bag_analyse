from BagDB import BagDB

import sys
import json
import time
import math
import rospy
import rosbag

import datetime
import argparse

# some helper functions that should be wrapped in a class performing the bag data entry

# The message value could be a tuple or single value. This function replaces any nan, inf or unicode values
#  that cannot be stored in the database. These are set to -1 (need to think if this could be a problem)
def replace_invalid_data(value):
    try:
        # if isinstance(value, list) or isinstance(value, tuple):
        # tuples are immutable, so this has to be converted to a string first
        if isinstance(value, tuple):
            value = list(value)
            for count, item in enumerate(value):
                if math.isnan(item):
                    value[count] = -1.
                elif math.isinf(item):
                    value[count] = -1.
            value = tuple(value)

        elif math.isnan(value):
            value = -1.
        elif math.isinf(value):
            value = -1.
        elif "\\u" in value:
            print ("unicode here")
            value = ""
    except:
        pass

    return value


# test if the value can be converted to jsonable type
def is_jsonable(x):
    try:
        json.dumps(x)
        return True
    except:
        return False


# recursively fill the message
def recursive_msg(message_field, prefix, message_dict):
    for slot in message_field.__slots__:
        try:
            recursive_msg(getattr(message_field, slot), prefix + "." + slot, message_dict)
        except:
            if is_jsonable(getattr(message_field, slot)):
                final_prefix = (prefix + "." + slot)[1:]
                message_dict[final_prefix] = getattr(message_field, slot)



parser = argparse.ArgumentParser(description='Imports ROSbags to the SWRI bag-database.')
parser.add_argument('bag_files', metavar='abc.bag',
                    help='A ROSBag file.')

parser.add_argument('--config', dest='config_file', default='/root/.ros-bag-database/settings.yml', required=True,
                    help='A bag-database style config file.')

parser.add_argument('--bagid', dest='bag_id', required=True,
                    help='The ID of the bag to be imported against.')

args = parser.parse_args()

bagdb = BagDB(config_file=args.config_file)

bag = rosbag.Bag(args.bag_files)

## This code will be useful later - find the mapping between topic types and
## topic names for the entire bag
#topics = bags[0].get_type_and_topic_info()[1].keys()
#
## get a dictionary for a list of topics belonging to each type
#types = dict()
#for key, value in bags[0].get_type_and_topic_info()[1].iteritems():
#    if value.msg_type not in types:
#        types[value.msg_type] = list()
#
#    types[value.msg_type].append(key)
#odom_message_types = types.get('nav_msgs/Odometry', list())
#print (odom_message_types)

print("Processing start time ", datetime.datetime.now())

batch_query_count = 0
unique_message_counter = 0


for topic, msg, rosbag_time in bag.read_messages():

    #print(topic, msg._type)

    # set to not include tf messages - this condition can be changed to anything
    # for example:  topic in odom_message_types:
    if 'tf' not in topic:
        unique_message_counter+=1
        message_dict = dict()

        # recursively extract the message items that are json compatible
        recursive_msg(msg, "", message_dict)

        tree = {}

        # turn into a nested dictionary
        for item, value in zip(message_dict.keys(), message_dict.values()):
            t = tree
            string_parts = item.split('.')
            for i, part in enumerate(string_parts):
                if i < len(string_parts) - 1:
                    # this key contains a nested dictionary
                    t = t.setdefault(part, {})
                else:
                    # this key is the final value
                    t.setdefault(part, replace_invalid_data(value))

        message_json = json.dumps(tree)

        try:
            # see if the message has a header with timestamp
            message_time = datetime.datetime.fromtimestamp(float(msg.header.stamp.secs) + (float(msg.header.stamp.nsecs) / 1000000000))
        except:
            # if there is no timestamp in the header, use the message recorded time from the rosbag
            message_time = datetime.datetime.fromtimestamp(float(rosbag_time.secs) + (float(rosbag_time.nsecs) / 1000000000))

        # TODO: set a relationship to the most recent global position message to allow searching
        # TODO: when on a position message, generate a geometry position and add it to the query
        batch_query_count += 1
        bagdb.AddMessageData(message_time, args.bag_id, message_json, topic, position_message_id, position_geo )

        if batch_query_count > 20000:
            print (".")
            batch_query_count = 0
            bagdb.CommitMessagesSoFar()

bagdb.CommitMessagesSoFar()

bag.close()

print ("End time ", datetime.datetime.now())


