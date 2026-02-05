cmake_minimum_required(VERSION 3.14)
project(xsctt VERSION 1.0.0)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find packages
find_package(CCTag CONFIG REQUIRED)
find_package(OpenCV CONFIG REQUIRED)

# Create executable
add_executable(xsctt xsctt.cpp)

# Link libraries
target_link_libraries(xsctt PRIVATE
    CCTag::CCTag
    opencv_core
    opencv_imgcodecs
    opencv_imgproc
    opencv_highgui
)