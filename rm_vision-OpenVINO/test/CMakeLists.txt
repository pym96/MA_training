cmake_minimum_required(VERSION 3.5)

project(OPENVINO_DEMO)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
add_definitions(-Wall -Werror)

find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(${OpenCV_INCLUDE_DIRS})

add_executable(${PROJECT_NAME} main.cpp) 

target_link_libraries(${PROJECT_NAME}
    ${OpenCV_LIBS}

)