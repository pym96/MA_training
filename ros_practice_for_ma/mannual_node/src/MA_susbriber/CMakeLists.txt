cmake_minimum_required(VERSION 3.8)
project(MA_susbriber)

## Setting c 99 standard here
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

## Setting c++ standard 17 here
if(NOT CMAKE_CXX_STANDARD)
  set(CXX_STANDARD 17)
  set(CXX_STANDARD_REQUIRED ON)
endif()

## By adding this command below, we enfoce cleaner code
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

## Include directory herer
include_directories(include)
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${EIGEN3_INCLUDE_DIR})

add_executable(MA_subscriber src/MA_train_subsriber.cpp)
ament_target_dependencies(MA_subscriber rclcpp std_msgs)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(TARGETS
  MA_subscriber
  DESTINATION lib/${PROJECT_NAME}
  )

ament_package()
