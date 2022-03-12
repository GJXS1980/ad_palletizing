cmake_minimum_required(VERSION 3.0.2)
project(ad_palletizing)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  serial
  std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES castlex_face_detection
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(Palletizing_AD_NODE src/palletizing.cpp src/cJSON.cpp)
target_link_libraries(Palletizing_AD_NODE ${catkin_LIBRARIES} -lpthread -lmosquitto )

install(TARGETS Palletizing_AD_NODE
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


catkin_install_python(PROGRAMS
   scripts/client_ros.py scripts/AN_Mission.py scripts/AN_Vision.py scripts/test.py
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )

install(
  DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)


