cmake_minimum_required(VERSION 2.8.3)
project(dmp)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS eigen roscpp rospy std_msgs message_generation)

## Generate messages and services
add_message_files(FILES DMPData.msg DMPPoint.msg DMPTraj.msg)
add_service_files(FILES GetDMPPlan.srv LearnDMPFromDemo.srv SetActiveDMP.srv)
generate_messages(DEPENDENCIES std_msgs)

catkin_package(INCLUDE_DIRS include
               CATKIN_DEPENDS common_msgs message_runtime)

## Build library
include_directories(include ${EIGEN_INCLUDE_DIRS})
add_library(dmp src/dmp.cpp src/fourier_approx.cpp src/radial_approx.cpp src/linear_approx)

## Build package
add_executable(dmp_server nodes/dmp_server.cpp)
target_link_libraries(dmp_server dmp ${catkin_LIBRARIES})

## Tests
catkin_add_gtest(testing_function_approx tests/main.cpp tests/test_radial_approx.cpp src/radial_approx.cpp)
target_link_libraries(testing_function_approx ${catkin_LIBRARIES})
#catkin_add_gtest(testing_function_approx tests/main.cpp tests/test_radial_approx.cpp)
#target_link_libraries(testing_function_approx dmp_lib ${catkin_LIBRARIES})

catkin_add_gtest(testing_dmp tests/main.cpp tests/test_dmp_using_radial_approx.cpp 
                             src/dmp.cpp src/radial_approx.cpp src/fourier_approx.cpp src/linear_approx.cpp)
target_link_libraries(testing_dmp ${catkin_LIBRARIES})
#catkin_add_gtest(testing_dmp tests/main.cpp tests/test_dmp_using_radial_approx.cpp)
#target_link_libraries(testing_dmp dmp_lib ${catkin_LIBRARIES})


