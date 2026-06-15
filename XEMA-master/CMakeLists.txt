cmake_minimum_required(VERSION 3.20)

# 默认安装位置:build目录下的install文件夹
set(CMAKE_INSTALL_PREFIX ${CMAKE_BINARY_DIR}/install CACHE PATH "Install path")
if(NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
  set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Debug" "Release" "RelWithDebInfo" "MinSizeRel")
endif()

project(xema VERSION 0.1.0 LANGUAGES C CXX)

# 统一 C++ 标准（建议用 target_* 方式，但这里做兜底）
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
if(MSVC)
    set(CMAKE_MSVC_RUNTIME_LIBRARY "MultiThreadedDLL")  # /MD
    add_compile_options(/utf-8) # MSVC 默认使用本地编码，强制改为 UTF-8 以避免乱码
endif()
if(WIN32)
    message(STATUS "Compiling on Windows")
    add_definitions(-D_CRT_SECURE_NO_WARNINGS)
endif()

# 包含第三方库动态库，用于编译应用程序
set(OPENCV_LIB_DIR ${PROJECT_SOURCE_DIR}/3rdparty/opencv_4_10_0_shared_md/x64/vc16/lib)

set(OPENCV_3RDPARTY_LIBS
    ${OPENCV_LIB_DIR}/opencv_world4100.lib
)

set(OPENCV_INCLUDE_DIR ${PROJECT_SOURCE_DIR}/3rdparty/opencv_4_10_0_shared_md/include)

# 包含第三方静态库，用于编译 SDK
set(OPENCV_STATIC_LIB_DIR ${PROJECT_SOURCE_DIR}/3rdparty/opencv_4_10_0_static_md/x64/vc16/staticlib)

set(OPENCV_STATIC_3RDPARTY_LIBS
    ${OPENCV_STATIC_LIB_DIR}/opencv_gapi4100.lib
    ${OPENCV_STATIC_LIB_DIR}/ade.lib
    ${OPENCV_STATIC_LIB_DIR}/opencv_highgui4100.lib
    ${OPENCV_STATIC_LIB_DIR}/opencv_ml4100.lib
    ${OPENCV_STATIC_LIB_DIR}/opencv_objdetect4100.lib
    ${OPENCV_STATIC_LIB_DIR}/opencv_photo4100.lib
    ${OPENCV_STATIC_LIB_DIR}/opencv_stitching4100.lib
    ${OPENCV_STATIC_LIB_DIR}/opencv_video4100.lib
    ${OPENCV_STATIC_LIB_DIR}/opencv_calib3d4100.lib
    ${OPENCV_STATIC_LIB_DIR}/opencv_dnn4100.lib
    ${OPENCV_STATIC_LIB_DIR}/libprotobuf.lib
    ${OPENCV_STATIC_LIB_DIR}/opencv_features2d4100.lib
    ${OPENCV_STATIC_LIB_DIR}/opencv_flann4100.lib
    ${OPENCV_STATIC_LIB_DIR}/opencv_videoio4100.lib
    ${OPENCV_STATIC_LIB_DIR}/opencv_imgcodecs4100.lib
    ${OPENCV_STATIC_LIB_DIR}/libjpeg-turbo.lib
    ${OPENCV_STATIC_LIB_DIR}/libwebp.lib
    ${OPENCV_STATIC_LIB_DIR}/libpng.lib
    ${OPENCV_STATIC_LIB_DIR}/libtiff.lib
    ${OPENCV_STATIC_LIB_DIR}/libopenjp2.lib
    ${OPENCV_STATIC_LIB_DIR}/IlmImf.lib
    ${OPENCV_STATIC_LIB_DIR}/opencv_imgproc4100.lib
    ${OPENCV_STATIC_LIB_DIR}/opencv_core4100.lib
    ${OPENCV_STATIC_LIB_DIR}/zlib.lib
    ${OPENCV_STATIC_LIB_DIR}/ittnotify.lib
    ${OPENCV_STATIC_LIB_DIR}/ippiw.lib
    ${OPENCV_STATIC_LIB_DIR}/ippicvmt.lib
)

set(OPENCV_STATIC_INCLUDE_DIR ${PROJECT_SOURCE_DIR}/3rdparty/opencv_4_10_0_static_md/include)


# 选项集中管理
option(BUILD_C_EXAMPLE "Build c example." ON)
option(BUILD_CPP_EXAMPLE "Build cpp example." ON)
option(BUILD_CMD_TOOL "Build cmd tool." ON)
option(BUILD_FUNCTEST_TOOL "Build functest tool." ON)
option(BUILD_GUI "Build gui." ON)
option(BUILD_TEST_TOOL "Build test tool." ON)
option(BUILD_CALIBRATION_TOOL "Build calibration tool." ON)
option(BUILD_CPP_UNIT_TEST "Build cpp unit tests." ON)

add_subdirectory(cpp)
add_subdirectory(sdk)

if(BUILD_C_EXAMPLE)
    add_subdirectory(c_examples)
endif()

if(BUILD_CPP_EXAMPLE)
    add_subdirectory(cpp_examples)
endif()

if(BUILD_CMD_TOOL)
    add_subdirectory(cmd)
endif()

if(BUILD_FUNCTEST_TOOL)
    add_subdirectory(functest)
endif()

if(BUILD_GUI)
    add_subdirectory(gui)
endif()

if(BUILD_TEST_TOOL)
    add_subdirectory(test)
endif()

if(BUILD_CALIBRATION_TOOL)
    add_subdirectory(calibration)
endif()

if(BUILD_CPP_UNIT_TEST)
    add_subdirectory(cpp_unit_test)
endif()

if(WIN32)
    # Windows 平台需要安装 Qt5 的 DLL
    install(DIRECTORY "${PROJECT_SOURCE_DIR}/3rdparty/Qt5.9.3/bin/"
        DESTINATION bin)
    install(FILES
        "${PROJECT_SOURCE_DIR}/3rdparty/ConfiguringIP/enumerate.dll"
        DESTINATION bin)
    install(FILES
        "${PROJECT_SOURCE_DIR}/3rdparty/tbb_2021_12_shared_md/tbb12.dll"
        DESTINATION bin)

    if(BUILD_GUI)
    	install(FILES
        	"${PROJECT_SOURCE_DIR}/3rdparty/ConfiguringIP/configuring_ip_gui.exe"
        	DESTINATION bin)
        install(FILES
        	"${PROJECT_SOURCE_DIR}/3rdparty/ConfiguringIP/configuring_ip_info.cfg"
        	DESTINATION bin)
    endif()
    install(FILES
        "${PROJECT_SOURCE_DIR}/3rdparty/opencv_4_10_0_shared_md/x64/vc16/bin/opencv_world4100.dll"
        "${PROJECT_SOURCE_DIR}/3rdparty/opencv_4_10_0_shared_md/x64/vc16/bin/opencv_videoio_ffmpeg4100_64.dll"
        DESTINATION bin)
endif()
