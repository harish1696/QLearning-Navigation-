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

add_executable(learner src/Learn.cpp src/Qlearn.cpp src/Qtable.cpp src/Turtlebot.cpp src/Environment.cpp)
target_link_libraries(learner ${catkin_LIBRARIES})

add_executable(testrun src/Test.cpp src/Qlearn.cpp src/Qtable.cpp src/Turtlebot.cpp src/Environment.cpp)
target_link_libraries(testrun ${catkin_LIBRARIES})

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)

  add_rostest_gtest(
    testQLearnNav
    test/testQLearnNav.launch
    test/main.cpp
    test/testQlearn.cpp
    test/testQtable.cpp
    test/testEnvironment.cpp
    test/testTurtlebot.cpp
    src/Turtlebot.cpp 
    src/Environment.cpp 
    src/Qlearn.cpp
    src/Qtable.cpp
  )

  target_link_libraries(
    testQLearnNav
    ${catkin_LIBRARIES}
  )

  add_dependencies(
    testQLearnNav
    learner
    ${catkin_EXPORTED_TARGETS}
  )
endif()
