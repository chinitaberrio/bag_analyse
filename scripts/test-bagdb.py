from BagDB import BagDB

import sys
import json
import time
import rospy
import rosbag

import datetime
import argparse

parser = argparse.ArgumentParser(description='Imports ROSbags to the SWRI bag-database.')
parser.add_argument('bag_files', metavar='abc.bag', nargs='+',
                    help='one or more ROSBag files.')

parser.add_argument('--config', dest='config_file', default='/root/.ros-bag-database/settings.yml', required=True,
                    help='A bag-database style config file.')

args = parser.parse_args()

bagdb = BagDB(config_file=args.config_file)

bags = []
for file in args.bag_files:
  bags.append(rosbag.Bag(file))

topics = bags[0].get_type_and_topic_info()[1].keys()

# get a dictionary for a list of topics belonging to each type
types = dict()
for key, value in bags[0].get_type_and_topic_info()[1].iteritems():
    if value.msg_type not in types:
        types[value.msg_type] = list()

    types[value.msg_type].append(key)

bagdb.ClearMessageData()
bagdb.ClearBagMetadata()

#odom_message_types = types.get('nav_msgs/Odometry', list())
#print (odom_message_types)

print ("Start time ", datetime.datetime.now())

batch_query_count = 0
unique_message_counter = 0

# start counting the bags from 1
for bag_count, bag in enumerate(bags, 1):

    # insert metadata for each of the bags
    bagdb.InsertBagMetadata(bag, bag_count)

    for topic, msg, t in bag.read_messages():

        # set to not include tf messages - this condition can be changed to anything
        if 'tf' not in topic: #True: #topic in odom_message_types:
            unique_message_counter+=1

            message_dict = dict()

            # test if the value can be converted to jsonable type
            def is_jsonable(x):
                try:
                    json.dumps(x)
                    return True
                except:
                    return False

            def recursive_msg(message_field, prefix, message_dict):
                for slot in message_field.__slots__:
                    try:
                        recursive_msg(getattr(message_field, slot), prefix + "." + slot, message_dict)
                    except:
                        if is_jsonable(getattr(message_field, slot)):
                            final_prefix = (prefix + "." + slot)[1:]
                            message_dict[final_prefix] = getattr(message_field, slot)

            # recursively extract the message items that are json compatible
            recursive_msg(msg, "", message_dict)

            tree = {}

            # turn into a nested dictionary
            for item, value in zip(message_dict.keys(), message_dict.values()):
                t = tree
                string_parts = item.split('.')
                for i, part in enumerate(string_parts):
                    if i < len(string_parts) - 1:
                        t = t.setdefault(part, {})
                    else:
                        t.setdefault(part, value)

            message_json = json.dumps(tree)

            # TODO: what to do with messages with no timestamp ?
            # probably should use the most recent time
            try:
                # see if the message has a header
                message_time = datetime.datetime.fromtimestamp(float(msg.header.stamp.secs) + (float(msg.header.stamp.nsecs) / 1000000000))
            except:
                message_time = datetime.datetime.fromtimestamp(0)

            # TODO: set a relationship to the most recent global position message to allow searching
            batch_query_count += 1
            bagdb.AddMessageData(topic, unique_message_counter, bag_count, message_time, 0, 0, msg._type, message_json)

            if batch_query_count > 20000:
                print (".")
                batch_query_count = 0
                bagdb.CommitMessagesSoFar()

    bagdb.CommitMessagesSoFar()

print ("End time ", datetime.datetime.now())


