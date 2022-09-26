cmake_minimum_required(VERSION 2.8.3)
project(vod_dataset)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)
find_package(PCL 1.3 REQUIRED COMPONENTS common io)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES vod_dataset
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(velocity_profile src/velocity_profile.cpp)
target_link_libraries(velocity_profile ${catkin_LIBRARIES})

catkin_install_python(PROGRAMS scripts/read_radar.py scripts/radar_velocity_profile.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)