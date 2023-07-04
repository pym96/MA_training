cmake_minimum_required(VERSION 3.5)
project(training1_for_ma)

## Set c 99 standard here
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()


## Set c++ standard 17 here
if(NOT CMAKE_CXX_STANDARD)
  set(CXX_STANDARD 17)
  set(CXX_STANDARD_REQUIRED ON)
endif()


if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(OpenCV REQUIRED)


## Inlcude directory here
include_directories(include)
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${EIGEN3_INCLUDE_DIR})

add_executable(talker src/transform.cpp)
ament_target_dependencies(talker rclcpp std_msgs)

install(TARGETS
  talker
  DESTINATION lib/${PROJECT_NAME})

ament_package()