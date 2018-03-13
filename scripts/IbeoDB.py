import utm
import math
import psycopg2
from datetime import datetime
import simplekml
import intervaltree

import numpy as np
from shapely.geometry import asLineString

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D


class IbeoDB:
    def __init__(self):
        # Try to connect
        try:
            print ("connecting to the database")
            self.conn = psycopg2.connect(
                #"dbname='bag_database_acfr' user='bag_database' password='letmein' host='localhost' port=5432")
                "dbname='gis' user='zio' password='zio' host='localhost' port=5432")
            self.conn.autocommit = True
        except:
            print "Unable to connect to the database."

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

        self.create_table_query = """CREATE TABLE corners(
                time TIMESTAMP,
                height REAL,
                angle REAL);
                CREATE UNIQUE INDEX timestamp ON corners(time);
                SELECT AddGeometryColumn ('public','corners','position',32756,'POINT',2);        
        """

        self.insert_data = """INSERT INTO corners VALUES (%s, 3.4, 1.41, ST_SetSRID(ST_MakePoint(333026, 6248411), 32756)), 
                                                         (%s, 3.4, 1.41, ST_SetSRID(ST_MakePoint(333027, 6248421), 32756)), 
                                                         (%s, 3.4, 1.41, ST_SetSRID(ST_MakePoint(333029, 6248441), 32756)), 
                                                         (%s, 3.4, 1.41, ST_SetSRID(ST_MakePoint(333034, 6248421), 32756)), 
                                                         (%s, 3.4, 1.41, ST_SetSRID(ST_MakePoint(333046, 6248401), 32756)) ;"""

    def InsertTestData(self):

        t1 = datetime.fromtimestamp(1234567890.)
        t2 = datetime.fromtimestamp(1234567891.)
        t3 = datetime.fromtimestamp(1234567892.)
        t4 = datetime.fromtimestamp(1234567893.)
        t5 = datetime.fromtimestamp(1234567894.)

        cur = self.conn.cursor()
        query_data = [t1,t2,t3,t4,t5]

        try:
            print ("inserting data " + self.insert_data)
            cur.execute(self.insert_data, query_data)
        except Exception as err:
            print "Couldn't insert data to the table: " + err.message


    def CreateTable(self):
        cur = self.conn.cursor()

        test_query_data = []

        try:
            print ("creating table " + self.create_table_query)
            cur.execute(self.create_table_query)
        except Exception as err:
            print "Couldn't create the table: " + err.message


    def TestIfTableExist(self, table_name):
        test_query = """select '""" + table_name + """'::regclass;"""
        test_query_data = []
        cur = self.conn.cursor()

        try:
            cur.execute(test_query, test_query_data)
        except:
            return False

        return True


    def LoadPositionData(self, position_query, query_data):
        cur = self.conn.cursor()

        try:
            cur.execute(position_query, query_data)
        except:
            print "Position query failed"

        db_rows = cur.fetchall()



if __name__=="__main__":
    ibeo = IbeoDB()
    if not ibeo.TestIfTableExist("corners"):
        print ("Tables didn't exist, creating")
        ibeo.CreateTable()

    ibeo.InsertTestData()
