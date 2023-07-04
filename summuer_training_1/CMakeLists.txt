cmake_minimum_required(VERSION 3.15)

project(MA)

## PROJECT_DIR这个量，就可以在代码中使用
add_compile_definitions(PROJECT_DIR="${PROJECT_SOURCE_DIR}")

## Set the basic info for project
set(SRC_LIST orb_cv.cpp)
set(2D_2D pose_estimation_2d_to_2d.cpp)
set(CMAKE_CXX_STANDARD 17)

## Export compile commands for clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

## By adding -Wall and -Werror, the complier does not ignore warnings
## enforcing cleaner code
add_definitions(-Wall -Werror)
add_compile_options(-Wall -Werror=return-type -Wno-unused-variable)

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_BINARY_DIR}/bin)
set(LIBRARY_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/lib)


message(STATUS "Current binary working dir is " ${PROJECT_BINARY_DIR})
message(STATUS "Current source working dir is " ${PROJECT_SOURCE_DIR})

add_executable(${PROJECT_NAME} ${SRC_LIST})


include_directories(modules)
add_subdirectory(modules)
target_link_libraries(MA modules)