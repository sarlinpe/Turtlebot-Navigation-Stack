cmake_minimum_required(VERSION 2.8.3)
project(turtlebot_simple_navigation)

find_package(catkin REQUIRED COMPONENTS
    std_msgs
    geometry_msgs
    nav_msgs
    gazebo_msgs
    sensor_msgs
    message_filters
    rospy
    roscpp
)

include_directories(include
    ${catkin_INCLUDE_DIRS}
)

install(DIRECTORY launch
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
install(DIRECTORY worlds
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

