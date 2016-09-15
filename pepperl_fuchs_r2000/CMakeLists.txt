cmake_minimum_required(VERSION 2.8.3)
project(pepperl_fuchs_r2000)

# Optionally set the default build type. This is bad practice.
IF(NOT CMAKE_BUILD_TYPE)
  #set(CMAKE_BUILD_TYPE Debug)
  set(CMAKE_BUILD_TYPE Release)
  #set(CMAKE_BUILD_TYPE RelWithDebInfo)
ENDIF(NOT CMAKE_BUILD_TYPE)

# This package requires c++0x
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -std=c++0x -Wfatal-errors")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
)
find_package(Boost REQUIRED COMPONENTS
  thread
)

catkin_package(
  INCLUDE_DIRS include ${Boost_INCLUDE_DIRS}
  LIBRARIES r2000_driver
  CATKIN_DEPENDS roscpp sensor_msgs std_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

add_library(r2000_driver
  src/driver/r2000_driver.cpp
  src/driver/http_command_interface.cpp
  src/driver/scan_data_receiver.cpp
)
target_link_libraries(r2000_driver
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)

add_executable(r2000_node
  src/rosnode/r2000_node.cpp
)
target_link_libraries(r2000_node
  r2000_driver
)

add_executable(driver_example
  src/example/driver_example.cpp
)
target_link_libraries(driver_example
  r2000_driver
)

install(TARGETS r2000_driver r2000_node driver_example
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(FILES launch/r2000.launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
install(FILES rviz/rviz.rviz
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/rviz
)
