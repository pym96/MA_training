cmake_minimum_required(VERSION 3.0)

project(CB)

add_executable(CB camera_caliberation.cpp)

if(NOT C_STANDARD)
    set(C_STANDARD 99)
    set(C_STANDARD_REQUIRED ON)
endif()

if(NOT CXX_STANDARD)
    set(CXX_STANDARD 11)
    set(CXX_STANDARD_REQUIRED ON)
endif()


## By adding the code below, we enfore cleaner code
# add_definitions(-Wall -Werror)

find_package(OpenCV REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS})
