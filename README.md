# ROS bag analysis scripts (TfNSW dataset)

Within this project, there are scripts designed to retrieve data from the rosbag specifically related to positioning. These scripts generate graphical representations of the vehicle's position and trajectory based on the collected sensor data. 

### rosbag-analysis.py

These scripts read the rosbag file and conducts detailed analysis on various sensors installed in the vehicle. It extracts relevant information from the sensor data and performs specific calculations or computations to gain insights into their performance or behavior. This analysis aims to provide a comprehensive understanding of the sensors' functionality and their impact on the overall vehicle localisation system.

### bag-analyse.py

#### rosrun bag_analysis bag-analyse.py --help

```
usage: bag-analyse.py [-h] -bag INPUT_BAG [-kml OUTPUT_KML_FILE] [-yaw]
                      [-yaw-rate] [-speed] [-pos] [-pos-3d-gnss]
                      [-pos-3d-odometry]

optional arguments:
  -h, --help            show this help message and exit
  -bag INPUT_BAG, --input-bag INPUT_BAG
                        Name of the ROS bag file to analyse. If a folder is
                        selected, the latest bag file in that folder is
                        analysed
  -kml OUTPUT_KML_FILE, --output-kml-file OUTPUT_KML_FILE
                        If given, the position information is output to this
                        KML file to be used in google earth
  -yaw, --show-yaw      Plot the yaw information from various sources
  -pitch-roll, --show-pitch-roll
                        Plot the Pitch and Roll information from various
                        sources
  -yaw-rate, --show-yaw-rate
                        Plot the yaw rate information from various sources
  -speed, --show-speed  Plot the speed information from various sources
  -pos, --show-position
                        Plot the position information from various sources
  -pos-3d-gnss, --show-3d-gnss
                        Plot the position information from GNSS sources in 3d
  -pos-3d-odometry, --show-3d-odometry
                        Plot the position information from odometry sources in
                        3d

```
