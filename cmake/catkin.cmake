find_package(catkin REQUIRED COMPONENTS
        roscpp
        rospy
        roslib
        segregator
        pagor
        )
catkin_package()

include_directories(
        ${catkin_INCLUDE_DIRS}
)
list(APPEND ALL_TARGET_LIBRARIES ${catkin_LIBRARIES})
message(STATUS "ALL_TARGET_LIBRARIES: ${ALL_TARGET_LIBRARIES}")
