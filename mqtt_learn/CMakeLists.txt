cmake_minimum_required(VERSION 3.5)

project(mqtt_example)

find_package(PkgConfig)
pkg_check_modules(MOSQUITTO libmosquitto)


include_directories(${MOSQUITTO_INCLUDE_DIRS})

add_executable(publisher mqtt_client.cpp)
target_link_libraries(publisher ${MOSQUITTO_LIBRARIES})

# 编译subscriber
add_executable(subscriber mqtt_subs.cpp)
target_link_libraries(subscriber ${MOSQUITTO_LIBRARIES})

