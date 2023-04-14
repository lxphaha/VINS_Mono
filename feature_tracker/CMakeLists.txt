cmake_minimum_required(VERSION 2.8.3)
project(feature_tracker)

<<<<<<< HEAD
set(CMAKE_BUILD_TYPE "Release")
=======
set(CMAKE_BUILD_TYPE "Debug")
>>>>>>> edf
set(CMAKE_CXX_FLAGS "-std=c++14")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    sensor_msgs
    cv_bridge
    camera_model
    )

<<<<<<< HEAD
find_package(OpenCV  4 REQUIRED)
=======
find_package(OpenCV 4 REQUIRED)
>>>>>>> edf

catkin_package()

include_directories(
    ${catkin_INCLUDE_DIRS}
    )

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
find_package(Eigen3)
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

add_executable(feature_tracker
    src/feature_tracker_node.cpp
    src/parameters.cpp
    src/feature_tracker.cpp
    )

<<<<<<< HEAD
target_link_libraries(feature_tracker ${catkin_LIBRARIES} ${OpenCV_LIBS})
=======
target_link_libraries(feature_tracker ${catkin_LIBRARIES} ${OpenCV_LIBS})
>>>>>>> edf
