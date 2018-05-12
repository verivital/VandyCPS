cmake_minimum_required(VERSION 2.8.3)
project(cps_vision)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages

find_package(catkin REQUIRED COMPONENTS message_generation roscpp rospy std_msgs sensor_msgs geometry_msgs mavros_msgs cwru_opencv_common)
find_package(OpenCV REQUIRED)

include_directories(SYSTEM ${OpenCV_INCLUDE_DIRS})

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${catkin_INCLUDES}
)

catkin_package(CATKIN_DEPENDS message_runtime roscpp rospy std_msgs sensor_msgs geometry_msgs  mavros_msgs cwru_opencv_common)
# example boost usage
# find_package(Boost REQUIRED COMPONENTS system thread)

# C++0x support - not quite the same as final C++11!
# use carefully;  can interfere with point-cloud library
#SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -pg -Q" )
# SET(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -pg -Q")

catkin_package(
	DEPENDS ${EIGEN_DEP}
	LIBRARIES cps_vision_lib
	CATKIN_DEPENDS roscpp
	INCLUDE_DIRS include ${EIGEN3_INCLUDE_DIRS}
)
# Libraries: uncomment the following and edit arguments to create a new library
# add_library(my_lib src/my_lib.cpp)   

add_library(cps_vision_lib
          src/cps_vision.cpp
)
target_link_libraries(cps_vision_lib ${catkin_LIBRARIES})
add_executable(cps_vision src/cps_vision_main.cpp)
#the following is required, if desire to link a node in this package with a library created in this same package
# edit the arguments to reference the named node and named library within this package
# target_link_library(example my_lib)
target_link_libraries(cps_vision cps_vision_lib)