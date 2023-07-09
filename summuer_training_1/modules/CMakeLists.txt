set(CMAKE_CXX_STANDARD 11)


## Find package
find_package(OpenCV REQUIRED)
find_package(fmt REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Sophus REQUIRED)


## Include directories
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(${Sophus_INCLUDE_DIRS})


message("OpenCV version is ${OpenCV_VERSION}")
# message(STATUS "Current binary work dir is " ${MA_BINARY_DIR})
message("Eigen version is " ${Eigen3_VERSION} ", its dir is " ${EIGEN3_INCLUDE_DIRS})

aux_source_directory(cv cv_src)
aux_source_directory(fmt_lib fmt_src)
aux_source_directory(ekf ekf_src)

add_library(modules SHARED ${cv_src} ${fmt_src} ${ekf_src})

target_link_libraries(modules
    ${OpenCV_LIBS}
    fmt::fmt
)