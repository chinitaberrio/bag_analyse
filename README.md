# ROS bag analysis scripts

## This project contains scripts which query the rosbag database and plot some results. These scripts were used to generate the graphs presented to Renault in the project report.

### ROSBagAnalysis.py

This script contains the class ROSBagAnalysis, which connects to the rosbag database and contains some functions to load data, output traces to KML, etc.

### analysis.py

This script contains several specific anaysis instances implementing the ROSBagAnalysis class - one for outputting all traces to KML, one for plotting z-acceleration and velocity for a small area near the university and one for plotting a velocity analysis of vehicles driving over the harbour bridge

