# 查找线程库
find_package(Threads REQUIRED)

add_library(xemacpp SHARED
    xema_camera.cpp
    ../test/triangulation.cpp
    ../sdk/socket_tcp.cpp
    ../firmware/easylogging++.cc
    ../firmware/basic_function.cpp
    calibration.cpp
    calibrate_function.cpp
    DetectCCTag.cpp
    PrecisionTest.cpp
    encode.cpp
    LookupTableFunction.cpp
    support_function.cpp
    ../firmware/system_config_settings.cpp
)


# 添加头文件路径
target_include_directories(xemacpp PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}                     
    ${CMAKE_CURRENT_SOURCE_DIR}/../SDK              
    ${CMAKE_CURRENT_SOURCE_DIR}/../firmware   
    ${CMAKE_CURRENT_SOURCE_DIR}/../test          
    ${CMAKE_CURRENT_SOURCE_DIR}/../3rdparty/boost_1_76_0_static_md/include/boost-1_76
    ${OPENCV_STATIC_INCLUDE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/../3rdparty/cctag_1_0_4_static_md/include
    ${CMAKE_CURRENT_SOURCE_DIR}/../3rdparty/Eigen3
)

# 添加库目录
link_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/../3rdparty/boost_1_76_0_static_md/lib
    ${CMAKE_CURRENT_SOURCE_DIR}/../3rdparty/cctag_1_0_4_static_md/lib
    ${CMAKE_CURRENT_SOURCE_DIR}/../3rdparty/tbb_2021_12_shared_md
)
# 添加第三方库
set(CCTAG_LIB_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../3rdparty/cctag_1_0_4_static_md/lib)
set(TBB_LIB_DIR  ${CMAKE_CURRENT_SOURCE_DIR}/../3rdparty/tbb_2021_12_shared_md)
set(BOOST_LIB_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../3rdparty/boost_1_76_0_static_md/lib/)

set(THIRD_PARTY_LIBS
    comctl32.lib
    ole32.lib
    setupapi.lib
    ws2_32.lib
    user32.lib
    gdi32.lib
    wsock32.lib

    ${CCTAG_LIB_DIR}/CCTag.lib

    ${TBB_LIB_DIR}/tbb12.lib
         
    ${BOOST_LIB_DIR}/libboost_wserialization-vc142-mt-x64-1_76.lib
    ${BOOST_LIB_DIR}/libboost_wave-vc142-mt-x64-1_76.lib
    ${BOOST_LIB_DIR}/libboost_unit_test_framework-vc142-mt-x64-1_76.lib
    ${BOOST_LIB_DIR}/libboost_type_erasure-vc142-mt-x64-1_76.lib
    ${BOOST_LIB_DIR}/libboost_timer-vc142-mt-x64-1_76.lib
    ${BOOST_LIB_DIR}/libboost_thread-vc142-mt-x64-1_76.lib
    ${BOOST_LIB_DIR}/libboost_test_exec_monitor-vc142-mt-x64-1_76.lib
    ${BOOST_LIB_DIR}/libboost_system-vc142-mt-x64-1_76.lib
    ${BOOST_LIB_DIR}/libboost_stacktrace_windbg_cached-vc142-mt-x64-1_76.lib
    ${BOOST_LIB_DIR}/libboost_stacktrace_windbg-vc142-mt-x64-1_76.lib
    ${BOOST_LIB_DIR}/libboost_serialization-vc142-mt-x64-1_76.lib
    ${BOOST_LIB_DIR}/libboost_stacktrace_noop-vc142-mt-x64-1_76.lib
    ${BOOST_LIB_DIR}/libboost_regex-vc142-mt-x64-1_76.lib
    ${BOOST_LIB_DIR}/libboost_random-vc142-mt-x64-1_76.lib
    ${BOOST_LIB_DIR}/libboost_program_options-vc142-mt-x64-1_76.lib
    ${BOOST_LIB_DIR}/libboost_prg_exec_monitor-vc142-mt-x64-1_76.lib
    ${BOOST_LIB_DIR}/libboost_nowide-vc142-mt-x64-1_76.lib
    ${BOOST_LIB_DIR}/libboost_math_tr1l-vc142-mt-x64-1_76.lib
    ${BOOST_LIB_DIR}/libboost_math_tr1f-vc142-mt-x64-1_76.lib
    ${BOOST_LIB_DIR}/libboost_math_tr1-vc142-mt-x64-1_76.lib
    ${BOOST_LIB_DIR}/libboost_math_c99l-vc142-mt-x64-1_76.lib
    ${BOOST_LIB_DIR}/libboost_math_c99f-vc142-mt-x64-1_76.lib
    ${BOOST_LIB_DIR}/libboost_math_c99-vc142-mt-x64-1_76.lib
    ${BOOST_LIB_DIR}/libboost_log_setup-vc142-mt-x64-1_76.lib
    ${BOOST_LIB_DIR}/libboost_log-vc142-mt-x64-1_76.lib
    ${BOOST_LIB_DIR}/libboost_locale-vc142-mt-x64-1_76.lib
    ${BOOST_LIB_DIR}/libboost_json-vc142-mt-x64-1_76.lib
    ${BOOST_LIB_DIR}/libboost_iostreams-vc142-mt-x64-1_76.lib
    ${BOOST_LIB_DIR}/libboost_graph-vc142-mt-x64-1_76.lib
    ${BOOST_LIB_DIR}/libboost_filesystem-vc142-mt-x64-1_76.lib
    ${BOOST_LIB_DIR}/libboost_fiber-vc142-mt-x64-1_76.lib
    ${BOOST_LIB_DIR}/libboost_exception-vc142-mt-x64-1_76.lib
    ${BOOST_LIB_DIR}/libboost_coroutine-vc142-mt-x64-1_76.lib
    ${BOOST_LIB_DIR}/libboost_date_time-vc142-mt-x64-1_76.lib
    ${BOOST_LIB_DIR}/libboost_contract-vc142-mt-x64-1_76.lib
    ${BOOST_LIB_DIR}/libboost_context-vc142-mt-x64-1_76.lib
    ${BOOST_LIB_DIR}/libboost_container-vc142-mt-x64-1_76.lib
    ${BOOST_LIB_DIR}/libboost_chrono-vc142-mt-x64-1_76.lib
    ${BOOST_LIB_DIR}/libboost_atomic-vc142-mt-x64-1_76.lib

)

# 链接库
target_link_libraries(xemacpp PRIVATE
    Threads::Threads
    ${THIRD_PARTY_LIBS}
    ${OPENCV_STATIC_3RDPARTY_LIBS}
)

# Windows DLL 导出宏
if(WIN32)
    target_compile_definitions(xemacpp PRIVATE USE_OPENCV  XEMA_SDK_EXPORTS)
endif()

install(TARGETS xemacpp
    RUNTIME DESTINATION bin
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
)


##############不含第三方依赖##################################
set(NOOPENCV_SRC
    xema_camera.cpp
    ../test/triangulation.cpp
    ../SDK/socket_tcp.cpp
    ../firmware/easylogging++.cc
    ../firmware/system_config_settings.cpp
    ../firmware/basic_function.cpp
)

add_library(xema_core SHARED ${NOOPENCV_SRC})

target_include_directories(xema_core PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/../SDK
    ${CMAKE_CURRENT_SOURCE_DIR}/../firmware
    ${CMAKE_CURRENT_SOURCE_DIR}/../test
)

target_link_libraries(xema_core PRIVATE
    Threads::Threads
)

if(WIN32)
    target_compile_definitions(xema_core PRIVATE SPARROW_SDK_EXPORTS)
endif()

install(TARGETS xema_core
    RUNTIME DESTINATION bin
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib	
)

##############中性模式##################################
set(NOOPENCV_SRC
    xema_camera.cpp
    ../test/triangulation.cpp
    ../SDK/socket_tcp.cpp
    ../firmware/easylogging++.cc
    ../firmware/system_config_settings.cpp
    ../firmware/basic_function.cpp
)

add_library(camera SHARED ${NOOPENCV_SRC})

target_include_directories(camera PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/../SDK
    ${CMAKE_CURRENT_SOURCE_DIR}/../firmware
    ${CMAKE_CURRENT_SOURCE_DIR}/../test
)

target_link_libraries(camera PRIVATE
    Threads::Threads
)

if(WIN32)
    target_compile_definitions(camera PRIVATE NEUTRAL_MODE CAMERA_SDK_EXPORTS)
endif()

install(TARGETS camera
    RUNTIME DESTINATION bin
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib	
)

