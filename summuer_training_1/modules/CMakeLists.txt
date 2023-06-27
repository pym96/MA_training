set(CMAKE_CXX_STANDARD 17)

find_package(OpenCV REQUIRED)
find_package(fmt REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS})

message("OpenCV version is ${OpenCV_VERSION}")
# message(STATUS "Current binary work dir is " ${MA_BINARY_DIR})

aux_source_directory(cv cv_src)
aux_source_directory(fmt_lib fmt_src)

add_library(modules SHARED ${cv_src} ${fmt_src})

target_link_libraries(modules
    ${OpenCV_LIBS}
    fmt::fmt
)