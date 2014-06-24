cmake_minimum_required(VERSION 2.4.6)
cmake_minimum_required(VERSION 2.8.3)
project(tom_smach)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roslib
  rospy
  smach
  smach_ros
)


catkin_package(
CATKIN_DEPENDS   
  roslib
  rospy
  smach
  smach_ros
)

install(DIRECTORY nodes
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

