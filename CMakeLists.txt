cmake_minimum_required(VERSION 2.8.3)
project(bag_analysis)


find_package(catkin REQUIRED COMPONENTS
  rospy
  dataset_tools
)

catkin_python_setup()

###################################
## catkin specific configuration ##
###################################

catkin_package(
  CATKIN_DEPENDS rospy
)

###########
## Build ##
###########


include_directories(
  ${catkin_INCLUDE_DIRS}
)



#############
## Install ##
#############


catkin_install_python(PROGRAMS
	scripts/bag-analyse.py
		DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)



