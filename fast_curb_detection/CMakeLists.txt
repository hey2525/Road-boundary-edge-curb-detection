cmake_minimum_required(VERSION 2.8.3)
project(fast_curb_detection)


find_package(catkin REQUIRED COMPONENTS
  pcl_conversions
  pcl_ros
  roscpp
  rospy
  std_msgs
  grid_map_ros
  grid_map_cv
  grid_map_msgs
  autoware_msgs
  #velodyne_pointcloud
)

catkin_package(CATKIN_DEPENDS)
find_package(OpenCV REQUIRED)
find_package(catkin REQUIRED)
include_directories( ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})

set(SOURCE_FILES main.cpp)

add_executable(main src/main.cpp src/algorithm.cpp src/algorithm.h src/obstacle.cpp src/obstacle.h)

add_definitions(-std=c++11 -O2  -g)

find_package(PCL REQUIRED)

find_package(catkin_simple REQUIRED)

add_dependencies(main ${catkin_EXPORTED_TARGETS})

include_directories(
	include 
	${catkin_INCLUDE_DIRS}${OpenCV_LIBRARIES}${PCL_INCLUDE_DIRS}
)
link_directories(${PCL_LIBRARY_DIRS})
link_directories(${PCL_INCLUDE_DIRS})
add_definitions(${PCL_DEFINITIONS})

target_link_libraries(main ${catkin_LIBRARIES} ${PCL_LIBRARIES})
target_link_libraries(main ${catkin_LIBRARIES} ${OpenCV_LIBS})
