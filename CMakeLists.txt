cmake_minimum_required(VERSION 2.8.3)
project(core_rover)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  nova_common
	sensor_msgs
	message_generation
	tf
  webots_ros
)

find_package (Threads REQUIRED)
find_package(SDL2 REQUIRED)

## Generate services in the 'srv' folder
 add_service_files(
    FILES
    set_bool.srv
    set_float.srv
    set_int.srv
  )
## Generate added messages and services with any dependencies listed here
  generate_messages(
    DEPENDENCIES
    std_msgs
    sensor_msgs
  )

catkin_package(
	CATKIN_DEPENDS roscpp rospy std_msgs message_runtime
)

include_directories(
  # include
  ${catkin_INCLUDE_DIRS}
  "${PROJECT_SOURCE_DIR}/include"
)



add_executable(driver src/driver.cpp)
add_dependencies(driver core_rover_generate_messages_cpp)
target_link_libraries(driver ${catkin_LIBRARIES})

add_executable(sim_init src/sim_init.cpp)
add_dependencies(sim_init core_rover_generate_messages_cpp)
target_link_libraries(sim_init ${catkin_LIBRARIES})

target_link_libraries(driver ${PROJECT_SOURCE_DIR}/lib/x86-64/libCTRE_Phoenix.a)
target_link_libraries(driver ${PROJECT_SOURCE_DIR}/lib/x86-64/libCTRE_PhoenixCCI.a)
target_link_libraries(driver ${PROJECT_SOURCE_DIR}/lib/x86-64/libCTRE_PhoenixPlatformLinuxSocketCan.a)
target_link_libraries(driver ${PROJECT_SOURCE_DIR}/lib/x86-64/libCTRE_PhoenixCanutils.a)

add_executable(testUART src/testUART.cpp)
add_dependencies(testUART core_rover_generate_messages_cpp)
target_link_libraries(testUART ${catkin_LIBRARIES})

target_link_libraries(driver Threads::Threads)


