from BagDataType import *



class DataContainer:

    def __init__(self,
                 bag,
                 gnss=GNSS([]),
                 imu=IMU([]),
                 odometry=Odometry([]),
                 steering=Steering([]),
                 velocity=Velocity([]),
                 target_speed_source='/zio/odometry/rear'):

        self.gnss = gnss
        self.imu = imu
        self.odometry = odometry
        self.steering = steering
        self.velocity = velocity

        current_speed = 0.

        self.datum = []

        for topic, msg, t in bag.read_messages():

            # pick only the times where the vehicle was moving GOING OUT
            # if (t.to_sec() < 30+1.49678753e9 or t.to_sec() > 75+1.49678753e9):
            #    continue

            if topic == target_speed_source:
                current_speed = msg.twist.twist.linear.x

            if topic in self.gnss.topic_list:
                self.gnss.new_message(topic, msg, t)
                if len(self.datum) == 0 \
                        and self.gnss.data[topic][-1][2] > 300000 \
                        and self.gnss.data[topic][-1][2] < 350000 \
                        and self.gnss.data[topic][-1][5] != 0.0:
                    self.datum = np.array(self.gnss.data[topic][-1])

            elif topic in self.imu.topic_list:
                self.imu.new_message(topic, msg, t, current_speed)

            elif topic in self.odometry.topic_list:
                self.odometry.new_message(topic, msg, t)

            # the steering wheel messages name starts with 'left_steering'
            elif topic in self.steering.topic_list and msg.name[0] == 'left_steering':
                self.steering.new_message(topic, msg, t)

            # the wheel velocity messages name starts with 'front_left_wheel'
            elif topic in self.steering.topic_list and msg.name[0] == 'front_left_wheel':
                self.velocity.new_message(topic, msg, t)

        bag.close()

        self.imu.convert_numpy()
        self.odometry.convert_numpy()
        self.gnss.convert_numpy()
        self.velocity.convert_numpy()
        self.steering.convert_numpy()

        print (self.datum)
        self.odometry.recalculate_odometry_2d(self.datum[5] + math.pi / 2.)
        self.imu.recalculate_odometry_2d(self.datum[5] + math.pi / 2.)


    def output_KML_path(self, file_name):
        # output the paths to KML
        # east_base, north_base, zone, letter = utm.from_latlon(-33.889565,151.1933352)
        #east_base, north_base, zone, letter = utm.from_latlon(-33.88977, 151.19324)
        # east_base, north_base, zone, letter = utm.from_latlon(-33.888518833, 151.194787333)
        east_base, north_base, zone, letter = utm.from_latlon(self.datum[6], self.datum[7])
        kml = simplekml.Kml()

        for topic in self.odometry.topic_list:
            if len(self.odometry.data[topic]) > 0:
                self.odometry.add_KML_path(kml,
                                            topic,
                                            east_base, self.odometry.data[topic][:, 2],
                                            north_base, self.odometry.data[topic][:, 1] * -1.,
                                            simplekml.Color.green, zone, letter)

        for topic in self.gnss.topic_list:
            if len(self.gnss.data[topic]) > 0:
                self.gnss.add_KML_path(kml,
                                        topic,
                                        0, self.gnss.data[topic][:, 2],
                                        0, self.gnss.data[topic][:, 3],
                                        simplekml.Color.red, zone, letter)

        for topic in self.imu.topic_list:
            if len(self.imu.data[topic]) > 0:
                self.imu.add_KML_path(kml, topic + '-gyro',
                                       east_base, self.imu.gyro_path[topic][:, 2],
                                       north_base, self.imu.gyro_path[topic][:, 1] * -1.,
                                       simplekml.Color.yellow, zone, letter)

                self.imu.add_KML_path(kml, topic + '-attitude',
                                       east_base, self.imu.attitude_path[topic][:, 2],
                                       north_base, self.imu.attitude_path[topic][:, 1] * -1.,
                                       simplekml.Color.blue, zone, letter)

        kml.save(file_name)
