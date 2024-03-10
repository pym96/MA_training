cmake_minimum_required(VERSION 3.0)

project(simple_bt)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(behaviortree_cpp)

add_executable(${PROJECT_NAME} bt_v3_demo.cc)
add_executable(${PROJECT_NAME}_grasp grasp_ball_robot_demo.cc)

target_link_libraries(${PROJECT_NAME} BT::behaviortree_cpp)
target_link_libraries(${PROJECT_NAME}_grasp BT::behaviortree_cpp)
