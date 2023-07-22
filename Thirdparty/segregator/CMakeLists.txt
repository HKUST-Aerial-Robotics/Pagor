cmake_minimum_required(VERSION 3.15)
project(segregator)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")
set(CMAKE_CXX_STANDARD 17)
# set(CMAKE_BUILD_TYPE "Debug")

set(CMAKE_INSTALL_RPATH "/usr/local/lib")
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release")
endif ()

find_package(catkin REQUIRED COMPONENTS
        roscpp
        rospy
        std_msgs
        geometry_msgs
        message_generation
        sensor_msgs
        nav_msgs
        pcl_ros
        eigen_conversions
        pagor
        )

find_package(OpenMP)
find_package(PCL 1.8 REQUIRED)
find_package(Boost 1.54 REQUIRED)
find_package(Eigen3 3.2 QUIET REQUIRED NO_MODULE)
find_package(yaml-cpp REQUIRED)
include(cmake/glog.cmake)

add_message_files(
        FILES
        cloud_info.msg
)

generate_messages(
        DEPENDENCIES
        geometry_msgs
        std_msgs
        nav_msgs
)

catkin_package(
        INCLUDE_DIRS include
        LIBRARIES ${PROJECT_NAME}
        CATKIN_DEPENDS
        roscpp
        rospy
        std_msgs
)

include_directories(${GLOG_INCLUDE_DIRS})
include_directories(include
        ${catkin_INCLUDE_DIRS}
        ${PCL_INCLUDE_DIRS}
        ${YAML_CPP_INCLUDEDIR}
        ${GLOG_INCLUDE_DIRS}
        )
message("catkin_INCLUDE_DIRS: ${catkin_INCLUDE_DIRS}")
add_definitions(${PCL_DEFINITIONS})
link_directories(${PCL_LIBRARY_DIRS})

if (OPENMP_FOUND)
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif ()

add_subdirectory(Thirdparty/pmc)

####### Executable ####### 
set(teaser_src
        src/registration.cc
        src/graph.cc
        src/teaser_utils/feature_matcher.cc
        src/teaser_utils/fpfh.cc
        )
file(GLOB_RECURSE lib_cpp "src/*.cpp")
file(GLOB_RECURSE lib_cc "src/*.cc")
list(APPEND ALL_SRCS ${lib_cpp} ${lib_cc})

add_library(${PROJECT_NAME}
        ${ALL_SRCS}
        )

target_link_libraries(${PROJECT_NAME}
        PUBLIC
        ${PCL_LIBRARIES}
        ${catkin_LIBRARIES}
        ${OpenMP_LIBS}
        stdc++fs
        pmc
        tbb
        ${YAML_CPP_LIBRARIES}
        ${GLOG_LIBRARIES}
        )

