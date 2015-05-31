cmake_minimum_required(VERSION 2.8)
project(opencvtesting)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
find_package(OpenCV REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${catkin_INCLUDE_DIRS})

set(SOURCE1_FILES pyramid.cpp)
add_executable(pyramid ${SOURCE1_FILES})
target_link_libraries(pyramid ${OpenCV_LIBRARIES})

set(GREEN_SOURCES green.cpp)
add_executable(green ${GREEN_SOURCES})
target_link_libraries(green ${OpenCV_LIBRARIES})

set(GET_HOMOGRAPHY_SOURCES get_homography_matrix.cpp)
add_executable(get_homography_matrix ${GET_HOMOGRAPHY_SOURCES})
target_link_libraries(get_homography_matrix ${OpenCV_LIBRARIES} ${catkin_LIBRARIES})

set(KNOWN_HOMOGRAPHY_SOURCES known_homography.cpp)
add_executable(known_homography ${KNOWN_HOMOGRAPHY_SOURCES})
target_link_libraries(known_homography ${OpenCV_LIBRARIES})

add_executable(dumb_config dumb_config_gen.cpp)
target_link_libraries(dumb_config ${OpenCV_LIBRARIES})

add_executable(vision_tuning_utility vision_tuning_utility.cpp)
target_link_libraries(vision_tuning_utility ${OpenCV_LIBRARIES})