cmake_minimum_required(VERSION 2.8.3)
project(motores)

find_package(catkin REQUIRED COMPONENTS dynamixel_sdk roscpp std_msgs)

catkin_package(CATKIN_DEPENDS dynamixel_sdk roscpp)

include_directories( ${dynamixel_sdk_INCLUDE_DIRS}
                     ${catkin_INCLUDE_DIRS}       )

add_executable(motionSeq src/motionSeq.cpp)
target_link_libraries(motionSeq ${catkin_LIBRARIES}
                           ${dynamixel_sdk_LIBRARIES})

