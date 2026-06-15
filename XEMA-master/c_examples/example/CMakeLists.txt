add_executable(example
    example.cpp
)

target_include_directories(example PRIVATE
    ${PROJECT_SOURCE_DIR}/sdk/
    ${PROJECT_SOURCE_DIR}/firmware/
    ${PROJECT_SOURCE_DIR}/3rdparty/ConfiguringIP/
)

target_link_directories(example PRIVATE
    ${PROJECT_SOURCE_DIR}/3rdparty/ConfiguringIP/
)

target_link_libraries(example PRIVATE
    open_cam3d_sdk
    enumerate
)

install(TARGETS example
    RUNTIME DESTINATION bin
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
)
