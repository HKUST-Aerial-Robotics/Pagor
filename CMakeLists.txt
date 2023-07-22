cmake_minimum_required(VERSION 3.15)
project(pagor)

message("start make pagor")

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")
set(CMAKE_CXX_STANDARD 17)

add_subdirectory(pagor)
add_subdirectory(Thirdparty/segregator)

set(ALL_TARGET_LIBRARIES "")
include(cmake/catkin.cmake)
include(cmake/boost.cmake)
include(cmake/yaml.cmake)
include(cmake/glog.cmake)
include(cmake/pcl.cmake)
include_directories(
        ${PROJECT_SOURCE_DIR}/include
        ${catkin_INCLUDE_DIRS}
)
file(GLOB_RECURSE ALL_SRCS "src/*.cpp")

add_executable(kitti_bm examples/kitti_bm.cpp ${ALL_SRCS})
target_link_libraries(kitti_bm ${ALL_TARGET_LIBRARIES})


