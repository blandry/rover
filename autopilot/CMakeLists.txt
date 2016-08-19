cmake_minimum_required(VERSION 2.8)

project(autopilot)

find_package(roscpp REQUIRED)

include_directories(${roscpp_INCLUDE_DIRS})

add_executable(autopilot autopilot.cpp arduino-serial-lib.cpp)

target_link_libraries(autopilot ${roscpp_LIBRARIES})