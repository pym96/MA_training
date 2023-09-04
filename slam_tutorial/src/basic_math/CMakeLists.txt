find_package(fmt REQUIRED)

add_executable(motion motion.cpp)

target_link_libraries(
    motion
    fmt::fmt
)