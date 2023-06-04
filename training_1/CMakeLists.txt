cmake_minimum_required(VERSION 3.15)

project(MA)

# PROJECT_DIR这个量，就可以在代码中使用
add_compile_definitions(PROJECT_DIR="${PROJECT_SOURCE_DIR}")

add_executable(${PROJECT_NAME} main.cpp)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

include_directories(modules)
add_subdirectory(modules)
target_link_libraries(MA modules)
