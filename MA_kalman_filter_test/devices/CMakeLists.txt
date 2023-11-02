find_package(OpenCV REQUIRED)
find_package(fmt REQUIRED)


include_directories(${OpenCV_INCLUDE_DIRS})

aux_source_directory(camera camera_include)

add_library(devices SHARED ${camera_include})


target_link_libraries(devices
    ${OpenCV_LIBS}
    fmt::fmt
    MVSDK
)