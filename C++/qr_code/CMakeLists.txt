cmake_minimum_required(VERSION 3.0)

project(test)

set(CMAKE_BUILD_TYPE "Debug")

find_package(OpenCV 4 REQUIRED)
message(STATUS "OpenCV version: " ${OpenCV_VERSION})

include_directories(
    ${OpenCV_INCLUDE_DIRS}
)


set(OUTPUT_PATH "${CMAKE_CURRENT_SOURCE_DIR}")
set(EXECUTABLE_OUTPUT_PATH "${OUTPUT_PATH}/bin/")

add_executable(test test.cpp 
                    qr_code.cpp qr_math.cpp 
                    ../initial/frame.cpp 
                    ../initial/Camera.cpp
                    ../initial/estimator.cpp)
target_link_libraries(test ${OpenCV_LIBS})