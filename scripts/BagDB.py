import os
import math
import rosbag
import psycopg2
import psycopg2.extras
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
        #TODO: implement this as execute_batch rather than single statements
        cur = self.conn.cursor()

        #if self.collection_of_messages != "":
        #    self.collection_of_messages += ","
        #args_str = ','.join(cur.mogrify("(%s,%s,%s,%s)", x) for x in self.tuple_of_message_data)
        #cur.execute("INSERT INTO bag_message_data (positiontime, bagid, messagedata, messagetopic) VALUES " + args_str)
        psycopg2.extras.execute_values(cur, """INSERT INTO bag_message_data (positiontime, bagid, typeid, messagedata, messagetopic, positionmessageid, position) VALUES %s""", self.tuple_of_message_data)

        self.conn.commit()

        self.tuple_of_message_data = []


    # add the message data to be commited later
    def AddMessageData(self, message_time, bag_id, type_id, message_json, topic, position_message_id, position_geo):

        # TODO: what other invalid messages are possible ? (Infinity is not valid)
        if 'Infinity' in message_json or '\\u' in message_json:
            pass
        else:
            self.tuple_of_message_data.append((message_time, bag_id, type_id, message_json, topic, position_message_id, position_geo))
