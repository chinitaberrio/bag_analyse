cmake_minimum_required(VERSION 2.8.3)
project(bag_analysis)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  rospy
  dataset_tools
)

catkin_python_setup()

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  CATKIN_DEPENDS rospy
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)



#############
## Install ##
#############


catkin_install_python(PROGRAMS
  #	scripts/BagDB.py
  # scripts/BagDBAnalysis.py
  # scripts/BagDataType.py
  # scripts/DataContainer.py
	scripts/bag-analyse.py
	scripts/rosbag-analysis.py
  # scripts/GNSS.py
  # scripts/IMU.py
  # scripts/Odometry.py
  # scripts/VehicleState.py
  # scripts/PandasAnalysis.py
  # scripts/Statistics.py


		DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)





#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_vn100_driver.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
