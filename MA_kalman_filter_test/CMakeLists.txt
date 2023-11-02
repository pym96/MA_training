cmake_minimum_required(VERSION 3.15)
project(MA)

include_directories(devices/camera)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

add_executable(${PROJECT_NAME} main.cpp)

add_subdirectory(devices)

target_link_libraries(${PROJECT_NAME} devices)

