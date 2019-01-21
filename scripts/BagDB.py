import re
import yaml

import psycopg2
import psycopg2.extras


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
 

    # batch commit of messages
    def CommitMessagesSoFar(self):
        cur = self.conn.cursor()

        psycopg2.extras.execute_values(cur, """INSERT INTO bag_message_data (positiontime, bagid, typeid, messagedata, messagetopic, positionmessageid, position) VALUES %s""", self.tuple_of_message_data)
        self.conn.commit()

        # clear values after they are inserted
        self.tuple_of_message_data = []


    # add the message data to be commited later
    def AddMessageData(self, message_time, bag_id, type_id, message_json, topic, position_message_id, position_geo):

        # data should be cleaned before this point (remove nan, inf and unicode)
        self.tuple_of_message_data.append((message_time, bag_id, type_id, message_json, topic, position_message_id, position_geo))
