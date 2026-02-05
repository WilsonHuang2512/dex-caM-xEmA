cmake_minimum_required(VERSION 3.14)
project(CCTagTest)

set(CMAKE_CXX_STANDARD 14)

# Find packages
find_package(CCTag CONFIG REQUIRED)
find_package(OpenCV CONFIG REQUIRED)

# Create test executable
add_executable(cctag_test main.cpp)

# Link libraries
target_link_libraries(cctag_test PRIVATE 
    CCTag::CCTag
    opencv_core
    opencv_imgcodecs
    opencv_imgproc
    opencv_highgui
)