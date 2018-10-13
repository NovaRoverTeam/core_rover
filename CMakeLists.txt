cmake_minimum_required(VERSION 2.8.3)
project(core_rover)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  nova_common
	sensor_msgs
	message_generation
	tf
)

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
)



add_executable(driver src/driver.cpp)

add_dependencies(driver core_rover_generate_messages_cpp)

target_link_libraries(driver ${catkin_LIBRARIES})

<<<<<<< HEAD



# ------ instructions for pioneer3at node -------
#add_executable(pioneer3at src/pioneer3at.cpp)
#add_dependencies(pioneer3at core_rover_generate_messages_cpp)
#target_link_libraries(pioneer3at
#	${catkin_LIBRARIES}
#)
=======
>>>>>>> 150e5811394a538e129e9c0a7878b58eb830d542
