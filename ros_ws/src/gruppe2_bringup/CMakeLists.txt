cmake_minimum_required(VERSION 2.8.3)
project(gruppe2_bringup)



## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  visualization_msgs
  std_msgs
  geometry_msgs
  tf 
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
find_package(Eigen3 REQUIRED)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES using_markers
#  CATKIN_DEPENDS roscpp visualization_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

add_executable(kalman_filter_node src/kalman_filter_node.cpp src/KF.cpp)
add_executable(extended_kalman_filter_node src/extended_kalman_filter_node.cpp src/EKF.cpp)
target_link_libraries(kalman_filter_node ${catkin_LIBRARIES})
target_link_libraries(extended_kalman_filter_node ${catkin_LIBRARIES})
