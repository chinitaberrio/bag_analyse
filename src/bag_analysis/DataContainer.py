from BagDataType import *

from IMU import IMU
from GNSS import GNSS, GNSSRates
from Odometry import Odometry
from Statistics import Statistics

from dataset_msgs.msg import LocaliserStats


class DataContainer:

    def __init__(self,
                 bag,
                 gnss=GNSS([]),
                 gnss_rates=GNSSRates([]),
                 imu=IMU([]),
                 odometry=Odometry([]),
                 statistics=Statistics([]),
                 target_speed_source=['ibeo/odometry', '/novatel/oem7/odom']):

        self.gnss = gnss
        self.gnss_rates = gnss_rates
        self.imu = imu
        self.odometry = odometry
  
        self.statistics = statistics

        current_speed = 0.

        self.datum = []

        for topic, msg, t in bag.read_messages():


            if topic in target_speed_source:
                current_speed = msg.twist.twist.linear.x

            if topic in self.gnss.topic_list:
                self.gnss.new_message(topic, msg, t)

                # take the first GNSS message and use as the datum for later plots
                if len(self.datum) == 0 \
                        and topic in self.gnss.data \
                        and len(self.gnss.data[topic]) > 0 \
                        and self.gnss.data[topic][-1][self.gnss.EASTING] > 300000 \
                        and self.gnss.data[topic][-1][self.gnss.EASTING] < 350000 \
                        and self.gnss.data[topic][-1][self.gnss.HEADING] != 0.0:
                    self.datum = np.array(self.gnss.data[topic][-1])

            elif topic in self.gnss_rates.topic_list:
                self.gnss_rates.new_message(topic, msg, t)

            elif topic in self.statistics.topic_list:
                self.statistics.new_message(topic, msg, t)

            elif topic in self.imu.topic_list:
                self.imu.new_message(topic, msg, t, current_speed)

            elif topic in self.odometry.topic_list:
                self.odometry.new_message(topic, msg, t)


        bag.close()

        self.imu.convert_numpy()
        self.odometry.convert_numpy()
        self.gnss.convert_numpy()
        self.statistics.convert_numpy()
        self.gnss_rates.convert_numpy()
   
    

        if len(self.datum) > 0:
            self.odometry.recalculate_odometry_2d(self.datum[5] + math.pi / 2.)
            self.imu.recalculate_odometry_2d(self.datum[5] + math.pi / 2.)
        else:
            self.odometry.recalculate_odometry_2d(0.)
            self.imu.recalculate_odometry_2d(0.)

        self.gnss.estimate_yaw_rate()


    def output_KML_path(self, file_name):
        # output the paths to KML
        east_base, north_base, zone, letter = utm.from_latlon(self.datum[6], self.datum[7])
        print ('east north base ' + str(east_base) + ", " + str(north_base))
        kml = simplekml.Kml()

        for topic in self.odometry.topic_list:
            if len(self.odometry.data[topic]) > 0:

                if abs(self.odometry.data[topic][:, self.odometry.X][-1]) >= 100000:
                    print('KML output topic with gps/gnss/ukf odometry ' + str(topic))
                    self.odometry.add_KML_path(kml,
                                            topic,
                                            0, self.odometry.data[topic][:, self.odometry.X],
                                            0, self.odometry.data[topic][:, self.odometry.Y],
                                            simplekml.Color.green, zone, letter)
                else:
                    self.odometry.add_KML_path(kml,
                                            topic,
                                            east_base, self.odometry.data[topic][:, self.odometry.Y],
                                            north_base, self.odometry.data[topic][:, self.odometry.X] * -1.,
                                            simplekml.Color.green, zone, letter)


        for topic in self.gnss.topic_list:
            if len(self.gnss.data[topic]) > 0:
                print('KML item on gnss topic list ' + str(topic))
                self.gnss.add_KML_path(kml,
                                        topic,
                                        0, self.gnss.data[topic][:, self.gnss.EASTING],
                                        0, self.gnss.data[topic][:, self.gnss.NORTHING],
                                        simplekml.Color.red, zone, letter)

        for topic in self.imu.topic_list:
            if len(self.imu.data[topic]) > 0:
                print('KML item on imu list ' + str(topic))
                self.imu.add_KML_path(kml, topic + '-gyro',
                                       east_base, self.imu.gyro_path[topic][:, self.imu.PATH_Y],
                                       north_base, self.imu.gyro_path[topic][:, self.imu.PATH_X] * -1.,
                                       simplekml.Color.yellow, zone, letter)

                self.imu.add_KML_path(kml, topic + '-attitude',
                                       east_base, self.imu.attitude_path[topic][:, self.imu.PATH_Y],
                                       north_base, self.imu.attitude_path[topic][:, self.imu.PATH_X] * -1.,
                                       simplekml.Color.blue, zone, letter)

        kml.save(file_name)

