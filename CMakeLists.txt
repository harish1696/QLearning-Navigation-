cmake_minimum_required(VERSION 2.8.3)
project(QLearning-Navigation-)

## Use C++11 compiler
add_compile_options(-std=c++11)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS 
    roscpp 
    geometry_msgs 
    move_base_msgs
    sensor_msgs
)

## Declare ROS messages and services
#add_message_files(DIRECTORY msg FILES Num.msg)
#add_service_files(FILES change_string.srv)

## Generate added messages and services
#generate_messages(DEPENDENCIES std_msgs)

## Declare a catkin package
catkin_package()

# Build talker and listener
include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(Learner src/Learn.cpp src/Qlearn.cpp src/Qtable.cpp src/Turtlebot.cpp src/Environment.cpp)
target_link_libraries(Learner ${catkin_LIBRARIES})

add_executable(Tester src/Test.cpp)
target_link_libraries(Tester ${catkin_LIBRARIES})



