# ROS bag analysis scripts

## This project contains scripts which query the rosbag database and plot some results. These scripts were used to generate the graphs presented to Renault in the project report.

### rosbag-analysis.py

This script connects to the bag database and performs some specific anaysis that was used to generate the report for Renault - one for outputting all traces to KML, one for plotting z-acceleration and velocity for a small area near the university and one for plotting a velocity analysis of vehicles driving over the harbour bridge

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
