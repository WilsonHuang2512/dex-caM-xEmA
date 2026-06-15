find_package(Threads)

add_executable(open_cam3d_test
    test.cpp
    reconstruct.cpp
    AnalyseError.cpp
    encode.cpp
    solution.cpp
    LookupTableFunction.cpp
    FilterModule.cpp
    support_function.cpp
    triangulation.cpp
    ${PROJECT_SOURCE_DIR}/cmd/getopt.c
    ${PROJECT_SOURCE_DIR}/firmware/easylogging++.cc
    ${PROJECT_SOURCE_DIR}/gui/PrecisionTest.cpp
    ${PROJECT_SOURCE_DIR}/calibration/calibrate_function.cpp
)

target_include_directories(open_cam3d_test PRIVATE
    ${OPENCV_INCLUDE_DIR}
    ${PROJECT_SOURCE_DIR}/cmd/
    ${PROJECT_SOURCE_DIR}/sdk/
    ${PROJECT_SOURCE_DIR}/firmware/
    ${PROJECT_SOURCE_DIR}/gui/
    ${PROJECT_SOURCE_DIR}/calibration/
)

target_link_libraries(open_cam3d_test PRIVATE
    Threads::Threads
    ${OPENCV_3RDPARTY_LIBS}
    open_cam3d_sdk
)

install(TARGETS open_cam3d_test
    RUNTIME DESTINATION bin
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
)
