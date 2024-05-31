cmake_minimum_required(VERSION 2.8.3)
project(lightwarelidar)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  CATKIN_DEPENDS roscpp sensor_msgs
)

###########
## Build ##
###########

include_directories(
  include
  include/linux
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(lwNx src/lwNx.cpp)
add_library(platformLinux src/linux/platformLinux.cpp)
add_library(serialPortLinux src/linux/lwSerialPortLinux.cpp)

add_executable(sf40c src/sf40c.cpp)
add_executable(lightwarelidar_sf45b src/lightwarelidar_sf45b.cpp)

target_link_libraries(sf40c
  lwNx
  platformLinux
  serialPortLinux
  ${catkin_LIBRARIES}
)

target_link_libraries(lightwarelidar_sf45b
  lwNx
  platformLinux
  serialPortLinux
  ${catkin_LIBRARIES}
)


#############
## Install ##
#############

install(
        TARGETS
        lightwarelidar_sf45b
        lwNx
        platformLinux
        serialPortLinux
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
