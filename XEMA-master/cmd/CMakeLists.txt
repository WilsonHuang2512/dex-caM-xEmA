find_package(Threads)

add_executable(open_cam3d
    cmd.cpp
    getopt.c
    projector_sharpness.cpp
    detect_chessboard.cpp
    ${PROJECT_SOURCE_DIR}/firmware/easylogging++.cc
    ${PROJECT_SOURCE_DIR}/gui/PrecisionTest.cpp
    ${PROJECT_SOURCE_DIR}/calibration/calibrate_function.cpp
    ${PROJECT_SOURCE_DIR}/test/reconstruct.cpp
    ${PROJECT_SOURCE_DIR}/test/AnalyseError.cpp
    ${PROJECT_SOURCE_DIR}/test/encode.cpp
    ${PROJECT_SOURCE_DIR}/test/solution.cpp
    ${PROJECT_SOURCE_DIR}/test/LookupTableFunction.cpp
    ${PROJECT_SOURCE_DIR}/test/FilterModule.cpp
    ${PROJECT_SOURCE_DIR}/test/support_function.cpp
    ${PROJECT_SOURCE_DIR}/test/triangulation.cpp
) 

target_link_libraries(open_cam3d PRIVATE
    ${OPENCV_3RDPARTY_LIBS}
    Threads::Threads
    open_cam3d_sdk
)

target_include_directories(open_cam3d PUBLIC
    ${PROJECT_SOURCE_DIR}/cmd
    ${PROJECT_SOURCE_DIR}/test
    ${PROJECT_SOURCE_DIR}/sdk
    ${OPENCV_INCLUDE_DIR}
)

target_compile_definitions(open_cam3d PUBLIC
    NDEBUG
    _CONSOLE
    _CRT_SECURE_NO_WARNINGS
)

install(TARGETS open_cam3d
    RUNTIME DESTINATION bin
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
)
