cmake_minimum_required(VERSION 3.15)
project(pagor)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")
set(CMAKE_CXX_STANDARD 17)

option(BUILD_TESTS "option for build tests" ON)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        rospy
        )
catkin_package(
        INCLUDE_DIRS include
        LIBRARIES ${PROJECT_NAME}
        CATKIN_DEPENDS roscpp rospy
)

set(ALL_TARGET_LIBRARIES "")
include(cmake/boost.cmake)
include(cmake/eigen.cmake)
include(cmake/glog.cmake)
include(cmake/openmp.cmake)
include(cmake/yaml.cmake)
include(cmake/pcl.cmake)
include(cmake/gtsam.cmake)

include_directories(
        ${PROJECT_SOURCE_DIR}/include
        ${catkin_INCLUDE_DIRS}
)

file(GLOB_RECURSE ALL_SRCS "*.cpp")
add_library(${PROJECT_NAME}
        ${ALL_SRCS}
        )

target_link_libraries(${PROJECT_NAME}
        PUBLIC
        ${ALL_TARGET_LIBRARIES} ${catkin_LIBRARIES}
        )