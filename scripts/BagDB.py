import os
import math
import rosbag
import psycopg2
import datetime

import re
import yaml


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


    # add the message data to be commited later
    def AddMessageData(self, topic, id, bagid, message_time, lat, lon, message_type, message_dict):

        # TODO: what other invalid messages are possible ? (Infinity is not valid)
        if 'Infinity' in message_dict or '\\u' in message_dict:
            pass
        else:
            self.tuple_of_message_data.append((message_time, bagid, message_dict, topic))
