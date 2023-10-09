cmake_minimum_required(VERSION 3.0)

project(test)

set(CMAKE_BUILD_TYPE "Debug")

find_package(OpenCV 4 REQUIRED)
message(STATUS "OpenCV version: " ${OpenCV_VERSION})

include_directories(
    ${OpenCV_INCLUDE_DIRS}
)

add_executable(test test.cpp 
                    qr_code.cpp qr_math.cpp 
                    ../initial/frame.cpp 
                    ../initial/Camera.cpp
                    ../initial/estimator.cpp)
target_link_libraries(test ${OpenCV_LIBS})