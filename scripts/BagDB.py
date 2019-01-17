import os
#import utm
import math
import rosbag
import random
#import hashlib
import psycopg2
import datetime
#import simplekml
#import intervaltree

import re
import yaml
import numpy as np
from shapely.geometry import asLineString

#import matplotlib.pyplot as plt
#from mpl_toolkits.mplot3d import Axes3D


class BagDB:
    def __init__(self, config_file):
        # Try to connect
    	if config_file:
            # Open the config file, get parameters
            with open(config_file, 'r') as ymlfile:
              # Skip the first line, as it contains a nonstandard yaml format that pyyaml doesn't like.
              for i in range(1):
                 _ = ymlfile.readline()
              cfg = yaml.safe_load(ymlfile)
            username = cfg['jdbcUsername']
            password = cfg['jdbcPassword']
            host, dbname = re.match('jdbc:postgresql://(.*?)/(.*)', cfg['jdbcUrl']).groups()

        try:
            print("connecting to the database")
            self.conn = psycopg2.connect(
                "dbname=" + dbname + " user=" + username + " password=" + password + " host=" + host + " port=5432")
        except:
            print("Unable to connect to the database.")


        self.collection_of_messages = ""
        self.tuple_of_message_data = []

        self.position_query = """select EXTRACT(EPOCH FROM bag_positions.positiontime), 
            	ST_X(bag_positions.position), 
            	ST_Y(bag_positions.position),
               	bag_positions.bagid,
        	    bag_positions.positiontime
            from bag_positions
            where bag_positions.bagid >= 353
            order by bag_positions.positiontime"""

        self.restricted_position_query = """select EXTRACT(EPOCH FROM bag_positions.positiontime), 
        	    ST_X(bag_positions.position), 
        	    ST_Y(bag_positions.position),
        	    bag_positions.bagid,
        	    bag_positions.positiontime
            from bag_positions
            where bag_positions.position && ST_MakeEnvelope(%s, %s, %s, %s)
            order by bag_positions.positiontime"""

    def RemoveOutlierPaths(self, sample_paths, required_bounding_boxes):

        for key, sample in list(sample_paths.items()):
            found_positions = dict()

            for bounding_box in required_bounding_boxes:
                found_positions[bounding_box] = False

            for row in sample:
                for bounding_box in required_bounding_boxes:
                    if row[2] > bounding_box[0] and row[2] < bounding_box[1] and row[1] > bounding_box[2] and row[1] < \
                            bounding_box[3]:
                        found_positions[bounding_box] = True

            for found_position in found_positions.itervalues():
                if not found_position:
                    del sample_paths[key]
                    break

    def OutputPathsToKML(self, samples, filename):

        kml = simplekml.Kml()

        for (bag_id, sample) in zip(samples.iterkeys(), samples.itervalues()):

            ls = kml.newlinestring(name=str('path:') + str(bag_id))
            np_sample = np.array(sample)

            ls.style.linestyle.width = 4

            path = np_sample[:, 1:3]
            ls.coords = path

            if path[0, 1] < path[-1, 1]:
                # colour for south->north
                ls.style.linestyle.color = simplekml.Color.red
            else:
                # colour for north->south
                ls.style.linestyle.color = simplekml.Color.green

        kml.save(filename)

    def ExtractSeparateTraces(self, position_query, query_data):
        cur = self.conn.cursor()

        try:
            cur.execute(position_query, query_data)
        except:
            print("Position query failed")

        db_rows = cur.fetchall()

        subsample = dict()
        samples = dict()

        print (len(db_rows))
        for row in db_rows:
            if row[3] not in samples:
                samples[row[3]] = list()
                subsample[row[3]] = 0

            subsample[row[3]] -= 1
            if subsample[row[3]] < 0:
                subsample[row[3]] = 10
                utm_sample = utm.from_latlon(row[2], row[1])

                samples[row[3]].append((row[0], row[1], row[2], utm_sample[0], utm_sample[1], row[4]))

        for key, sample in list(samples.items()):

            previous_position = (0., 0.)
            counter = 0
            start_position = 0
            for row in sample:
                if previous_position != (0., 0.):
                    # check distance between points
                    distance = math.sqrt(
                        math.pow(row[3] - previous_position[0], 2) + math.pow(row[4] - previous_position[1], 2))
                    if distance > 5000:
                        print ('[large jump detected] removing start of path ' + str(key) + ', ' + str(distance) + ', ' + str(counter))
                        start_position = counter
                previous_position = (row[3], row[4])
                counter += 1

            samples[key] = sample[start_position:-1]

            if len(sample) < 500:
                del samples[key]
                continue

        return samples

    def find_closest(self, A, target):
        # A must be sorted
        idx = A.searchsorted(target)
        idx = np.clip(idx, 1, len(A) - 1)
        left = A[idx - 1]
        right = A[idx]
        idx -= target - left < right - target
        return idx

    def LoadData(self, position_query, query_data, required_bounding_boxes, output_kml_path):

        samples = self.ExtractSeparateTraces(position_query, query_data)
        self.RemoveOutlierPaths(samples, required_bounding_boxes)
        self.OutputPathsToKML(samples, output_kml_path)

        return samples

    def PerformAnalysis(self, samples, sensor_query, limit_samples=1e10, direction='bidirectional'):

        sensor_data = dict()
        cur = self.conn.cursor()

        for (bag_id, sample) in zip(samples.iterkeys(), samples.itervalues()):
            np_sample = np.array(sample)

            if direction == 'north' and np_sample[0, 2] < np_sample[-1, 2]:
                continue
            elif direction == 'south' and np_sample[0, 2] > np_sample[-1, 2]:
                continue

            limit_samples -= 1

            if limit_samples < 0:
                break

            cur.execute(sensor_query, (bag_id, np_sample[0, 5], np_sample[-1, 5]))

            sensor = list()

            for row in cur.fetchall():
                # find the nearest position to sensor sample time
                closest_index = self.find_closest(np_sample[:, 0], row[0])
                sensor.append((row[0], row[1], row[2], np_sample[closest_index, 1], np_sample[closest_index, 2],
                               np_sample[closest_index, 3], np_sample[closest_index, 4]))

            sensor_data[bag_id] = np.array(sensor)

        return sensor_data


    # batch commit of messages
    def CommitMessagesSoFar(self):

        cur = self.conn.cursor()

        if self.collection_of_messages != "":
            self.collection_of_messages += ","

        args_str = ','.join(cur.mogrify("(%s,%s,%s,%s)", x) for x in self.tuple_of_message_data)
        cur.execute("INSERT INTO bag_message_data (positiontime, bagid, messagedata, messagetopic) VALUES " + args_str)

        self.conn.commit()

        self.tuple_of_message_data = []


    # Fill the metadata for each bag
    def InsertBagMetadata(self, bag, bag_number):
        cur = self.conn.cursor()
        print (int(bag.get_compression_info()[1]))
        cur.execute("INSERT INTO bags (id, createdon, filename, path, starttime, endtime, compressed, messagecount, size, indexed, md5sum, missing, version) VALUES (%s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s)",
                    (bag_number,
                     datetime.datetime.now(),
                     os.path.basename(bag.filename),
                     os.path.dirname(bag.filename),
                     datetime.datetime.fromtimestamp(bag.get_start_time()),
                     datetime.datetime.fromtimestamp(bag.get_end_time()),
                     bag.get_compression_info()[1] == bag.get_compression_info()[2], # is compressed size = uncompressed size
                     bag.get_message_count(),
                     int(bag.get_compression_info()[1]),  # assume size is uncompressed ?
                     True,
                     # the hash takes a long time and will be done in the java bag database, so the random number is temporary
                     random.randint(100000, 1e10), #hashlib.md5(open(bag.filename, 'rb').read()).hexdigest(),
                     False,
                     bag.version
                     ))

        self.conn.commit()



    def ClearBagMetadata(self):
        cur = self.conn.cursor()
        cur.execute("TRUNCATE bags CASCADE;")
        # Make the changes to the database persistent
        self.conn.commit()


    # add the message data to be commited later
    def AddMessageData(self, topic, id, bagid, message_time, lat, lon, message_type, message_dict):

        # TODO: what other invalid messages are possible ? (Infinity is not valid)
        if 'Infinity' in message_dict or '\\u' in message_dict:
            pass
        else:
            self.tuple_of_message_data.append((message_time, bagid, message_dict, topic))



    def ClearMessageData(self):

        cur = self.conn.cursor()
        cur.execute("TRUNCATE bag_message_data ")

        # Make the changes to the database persistent
        self.conn.commit()





