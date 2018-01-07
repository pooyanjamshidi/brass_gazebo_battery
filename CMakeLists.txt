cmake_minimum_required(VERSION 2.8.3 FATAL_ERROR)
project(brass_gazebo_battery)


# find gazebo
find_package(gazebo REQUIRED)

include_directories(${GAZEBO_INLCUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

add_library(gazebo_battery SHARED battery_discharge.cpp battery_charge.cpp battery_discharge.h)
target_link_libraries(gazebo_battery ${GAZEBO_LIBRARIES})

