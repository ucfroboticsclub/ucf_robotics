cmake_minimum_required(VERSION 2.8.3)
project(igvc_navigation)


#Load catkin dependencies
find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  tf
)

catkin_package(
	CATKIN_DEPENDS tf roscpp sensor_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(laser_footprint_filer src/laser_footprint_filter.cpp)
target_link_libraries(laser_footprint_filer ${catkin_LIBRARIES})

install(TARGETS laser_footprint_filer
	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY laser
	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
