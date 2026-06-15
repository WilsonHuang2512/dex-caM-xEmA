set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)

set(Qt5_DIR ${PROJECT_SOURCE_DIR}/3rdparty/Qt5.9.3/msvc2017_64/lib/cmake/Qt5)
find_package(Qt5 COMPONENTS Core Widgets Concurrent Xml REQUIRED)
find_package(Threads)

add_executable(open_cam3d_gui
    main.cpp
    camera_gui.cpp 
    camera_capture_gui.cpp
    calibration_param_gui.cpp 
    select_calibration_board_gui.cpp
    outlier_removal_gui.cpp
    update_firmware_gui.cpp
    update_opencam3d.cpp
    waiting_gui.cpp
    about_gui.cpp
    save_gui.cpp
    calibration_param_gui.cpp 
    settings_file_function.cpp 
    PrecisionTest.cpp
    file_io_function.cpp

    dexforce_camera_gui.qrc
    gui.rc
    myapp.rc

    ${PROJECT_SOURCE_DIR}/sdk/socket_tcp.cpp 
    ${PROJECT_SOURCE_DIR}/sdk/utils.cpp 
    ${PROJECT_SOURCE_DIR}/firmware/easylogging++.cc
    ${PROJECT_SOURCE_DIR}/firmware/system_config_settings.cpp
    ${PROJECT_SOURCE_DIR}/calibration/calibrate_function.cpp
)

# 仅 Windows：改为 GUI 子系统（避免弹控制台）
if(MSVC)
  target_link_options(open_cam3d_gui PRIVATE "/SUBSYSTEM:WINDOWS" "/ENTRY:mainCRTStartup")
elseif(WIN32)
  target_link_options(open_cam3d_gui PRIVATE "-mwindows")
endif()

target_compile_options(open_cam3d_gui PRIVATE
  $<$<CXX_COMPILER_ID:MSVC>:/utf-8>
)

target_compile_definitions(open_cam3d_gui PRIVATE
    TORRENT_LIBRARY
    NO_FREETYPE
    UNICODE
    _UNICODE
    HCPP_LEGACY_API
)

target_include_directories(open_cam3d_gui PRIVATE
    ${OPENCV_INCLUDE_DIR}
    ${PROJECT_SOURCE_DIR}/test
    ${PROJECT_SOURCE_DIR}/sdk 
    ${PROJECT_SOURCE_DIR}/3rdparty/ConfiguringIP/
)

target_link_directories(open_cam3d_gui PRIVATE
    ${PROJECT_SOURCE_DIR}/3rdparty/ConfiguringIP/
)

target_link_libraries(open_cam3d_gui PRIVATE
    ${OPENCV_3RDPARTY_LIBS}
    Threads::Threads
    Qt5::Core
    Qt5::Gui
    Qt5::Widgets
    Qt5::Xml
    open_cam3d_sdk
    enumerate
)

install(TARGETS open_cam3d_gui
    RUNTIME DESTINATION bin
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
)
