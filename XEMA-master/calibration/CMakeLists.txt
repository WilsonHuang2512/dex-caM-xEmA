# CCTag 1.0.4 静态链接库路径和库列表
set(CCTAG_LIB_DIR ${PROJECT_SOURCE_DIR}/3rdparty/cctag_1_0_4_static_md/lib)
set(BOOST_LIB_DIR ${PROJECT_SOURCE_DIR}/3rdparty/boost_1_76_0_static_md/lib/)
set(TBB_LIB_DIR  ${PROJECT_SOURCE_DIR}/3rdparty/tbb_2021_12_shared_md)

set(CCTAG_3RDPARTY_LIBS
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

set(CCTAG_INCLUDE_DIR
    ${PROJECT_SOURCE_DIR}/3rdparty/cctag_1_0_4_static_md/include
    ${PROJECT_SOURCE_DIR}/3rdparty/Eigen3
    ${PROJECT_SOURCE_DIR}/3rdparty/boost_1_76_0_static_md/include/boost-1_76
)

find_package(Threads)

add_executable(calibration
    calibration.cpp
    calibrate_function.cpp
    DetectCCTag.cpp
    ${PROJECT_SOURCE_DIR}/cmd/getopt.c
    ${PROJECT_SOURCE_DIR}/test/encode.cpp
    ${PROJECT_SOURCE_DIR}/test/support_function.cpp
    ${PROJECT_SOURCE_DIR}/firmware/easylogging++.cc
)

target_include_directories(calibration PRIVATE
    ${OPENCV_INCLUDE_DIR}
    ${CCTAG_INCLUDE_DIR}
    ${PROJECT_SOURCE_DIR}/test
    ${PROJECT_SOURCE_DIR}/sdk
    ${PROJECT_SOURCE_DIR}/gui
)

target_link_libraries(calibration PRIVATE
    ${OPENCV_3RDPARTY_LIBS}
    ${CCTAG_3RDPARTY_LIBS}
    Threads::Threads
    open_cam3d_sdk
)

install(TARGETS calibration
    RUNTIME DESTINATION bin
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
)
