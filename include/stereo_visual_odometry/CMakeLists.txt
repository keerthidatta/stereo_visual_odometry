cmake_minimum_required(VERSION 3.11.3)

# set(CMAKE_CXX_STANDARD 17)
project(OSM_A_star_search)
set(CMAKE_CXX_FLAGS ${CMAKE_CXX_FLAGS} "-std=c++17 -pthread")

find_package(OpenCV 4.4.0 REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS})
link_directories(${OpenCV_LIBRARY_DIRS})
add_definitions(${OpenCV_DEFINITIONS})

# Find all executables
file(GLOB project_SRCS src/main.cpp core/*.cpp core/*.h) #src/*.h

# Add project executable
add_executable(stereo_visual_odometry ${project_SRCS})
target_link_libraries(stereo_visual_odometry ${OpenCV_LIBRARIES})
