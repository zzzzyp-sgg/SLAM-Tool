cmake_minimum_required(VERSION 3.0)

project(test)

set(CMAKE_BUILD_TYPE "Debug")

find_package(OpenCV 4 REQUIRED)
message(STATUS "OpenCV version: " ${OpenCV_VERSION})

include_directories(
    ${OpenCV_INCLUDE_DIRS}
    "/usr/include/eigen3/"
)

add_executable(triangulation triangulation.cc Camera.cc)
target_link_libraries(triangulation ${OpenCV_LIBS})