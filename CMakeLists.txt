cmake_minimum_required(VERSION 2.8.3)
project(dmp)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS 
  eigen roscpp rospy std_msgs message_generation 
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependencies might have been
##     pulled in transitively but can be declared for certainty nonetheless:
##     * add a build_depend tag for "message_generation"
##     * add a run_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generatcatkinion" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
add_message_files(
   FILES DMPData.msg DMPPoint.msg DMPTraj.msg 
)

add_service_files(
   FILES GetDMPPlan.srv LearnDMPFromDemo.srv SetActiveDMP.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
   DEPENDENCIES std_msgs
)

catkin_package(
INCLUDE_DIRS include
   CATKIN_DEPENDS common_msgs image_common message_runtime
)

include_directories( include ${EIGEN_INCLUDE_DIRS} )

add_library(dmp src/dmp.cpp src/fourier_approx.cpp src/radial_approx.cpp src/linear_approx)
add_executable(dmp_server nodes/dmp_server.cpp)
target_link_libraries(dmp_server dmp ${catkin_LIBRARIES})
