add_executable(functest
    case_GetFrame01.cpp
    case_GetPointCloud.cpp
    evaluate.cpp
    functest.cpp
    help.cpp
    status.cpp
    ${PROJECT_SOURCE_DIR}/src/encode.cpp
    ${PROJECT_SOURCE_DIR}/cmd/getopt.c
    ${PROJECT_SOURCE_DIR}/src/reconstruct.cpp
    ${PROJECT_SOURCE_DIR}/src/solution.cpp
    ${PROJECT_SOURCE_DIR}/src/triangulation.cpp
)

target_include_directories(functest PRIVATE
    ${OPENCV_INCLUDE_DIR}
    ${PROJECT_SOURCE_DIR}/src/
    ${PROJECT_SOURCE_DIR}/cmd/
)

target_link_directories(functest PRIVATE
    ${PROJECT_SOURCE_DIR}/3rdparty/ConfiguringIP/
)

target_link_libraries(functest PRIVATE
    open_cam3d_sdk
    ${OPENCV_3RDPARTY_LIBS}
)

install(TARGETS functest
    RUNTIME DESTINATION bin
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
)
