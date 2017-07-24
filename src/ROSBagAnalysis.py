import utm
import math
import psycopg2
import simplekml
import intervaltree

import numpy as np
from shapely.geometry import asLineString

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D


class ROSBagAnalysis:
    def __init__(self):
        # Try to connect
        try:
            print ("connecting to the database")
            self.conn = psycopg2.connect(
                "dbname='bag_database_acfr' user='bag_database' password='letmein' host='localhost' port=5432")
        except:
            print "Unable to connect to the database."

        self.position_query = """select EXTRACT(EPOCH FROM bag_positions.positiontime), 
            	ST_X(bag_positions.position), 
            	ST_Y(bag_positions.position),
               	bag_positions.bagid,
        	    bag_positions.positiontime
            from bag_positions
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
        cur = conn.cursor()

        try:
            cur.execute(position_query, query_data)
        except:
            print "Position query failed"

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
                        print (
                        '[large jump detected] removing start of path ' + str(key) + ', ' + str(distance) + ', ' + str(
                            counter))
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

        samples = ExtractSeparateTraces(position_query, query_data)
        RemoveOutlierPaths(samples, required_bounding_boxes)
        OutputPathsToKML(samples, output_kml_path)

        return samples

    def PerformAnalysis(self, samples, sensor_query, limit_samples=1e10, direction='bidirectional'):

        sensor_data = dict()
        cur = conn.cursor()

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
                closest_index = find_closest(np_sample[:, 0], row[0])
                sensor.append((row[0], row[1], row[2], np_sample[closest_index, 1], np_sample[closest_index, 2],
                               np_sample[closest_index, 3], np_sample[closest_index, 4]))

            sensor_data[bag_id] = np.array(sensor)

        return sensor_data





