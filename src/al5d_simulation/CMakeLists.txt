## Project settings
cmake_minimum_required(VERSION 2.8.3)
project(al5d_simulation)

## Building with C++ standard 11
add_definitions(-std=c++11 -Wall -Wextra)

## Finding packages
find_package(catkin REQUIRED COMPONENTS
		roscpp
		rospy
		std_msgs
		geometry_msgs
		message_generation)

## Importing catkin packages
catkin_package(
  CATKIN_DEPENDS     
	std_msgs
)

## Including directories and adding source files to be build
include_directories(include ${catkin_INCLUDE_DIRS})
file(GLOB_RECURSE SOURCE_FILES "${PROJECT_SOURCE_DIR}/src/*/*.cpp")

## Adding node executable
add_executable(${PROJECT_NAME}_node src/main.cpp ${SOURCE_FILES})
add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
#target_link_libraries(${PROJECT_NAME}_node virtual_robot_controller ${catkin_LIBRARIES})

