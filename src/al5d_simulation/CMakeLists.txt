## Project settings
cmake_minimum_required(VERSION 2.8.3)
project(al5d_simulation)

## Building with C++ standard 11
add_definitions(-std=c++14 -Wall -Wextra)

## Finding packages
find_package(catkin REQUIRED COMPONENTS
		roscpp
		rospy
		std_msgs
		geometry_msgs
		sensor_msgs
		message_generation
		tf
		urdf)

## Importing catkin packages
catkin_package(
		INCLUDE_DIRS include
		LIBRARIES ${PROJECT_NAME}
		CATKIN_DEPENDS roscpp rospy std_msgs message_runtime sensor_msgs urdf
		DEPENDS system_lib message_runtime
)

## Including directories and adding source files to be build
include_directories(include ${catkin_INCLUDE_DIRS})
file(GLOB_RECURSE SOURCE_FILES "${PROJECT_SOURCE_DIR}/src/*/*.cpp")

## Adding node executable
add_executable(${PROJECT_NAME}_node src/main.cpp ${SOURCE_FILES})

add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})



target_link_libraries(${PROJECT_NAME}_node
		${catkin_LIBRARIES}
		)