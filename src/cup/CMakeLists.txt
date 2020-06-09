cmake_minimum_required(VERSION 2.8.3)
project(cup)

## Compile as C++11, supported in ROS Kinetic and newer
 add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp std_msgs tf)

catkin_package(
        CATKIN_DEPENDS roscpp std_msgs tf
        INCLUDE_DIRS include
)

include_directories(
 include
 ${catkin_INCLUDE_DIRS}
)

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
 add_executable(${PROJECT_NAME}_node src/main.cpp src/cup.cpp src/world.cpp src/robot.cpp src/speed.cpp
         src/transformable.cpp src/velocity.cpp src/orientation.cpp src/sensor.cpp src/distance_sensor.cpp
         src/touch_sensor.cpp)

## Add cmake target dependencies of the executable
## same as for the library above
 add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
 target_link_libraries(${PROJECT_NAME}_node
   ${catkin_LIBRARIES}
 )