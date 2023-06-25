cmake_minimum_required(VERSION 3.15)

project(MA)

# PROJECT_DIR这个量，就可以在代码中使用
add_compile_definitions(PROJECT_DIR="${PROJECT_SOURCE_DIR}")

set(SRC_LIST main.cpp)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)


set(EXECUTABLE_OUTPUT_PATH ${PROJECT_BINARY_DIR}/bin)
set(LIBRARY_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/lib)


message(STATUS "Current binary working dir is " ${PROJECT_BINARY_DIR})
message(STATUS "Current source working dir is " ${PROJECT_SOURCE_DIR})

add_executable(${PROJECT_NAME} ${SRC_LIST})


include_directories(modules)
add_subdirectory(modules)
target_link_libraries(MA modules)
