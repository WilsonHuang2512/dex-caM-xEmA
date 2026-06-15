add_executable(param_json
    param_json.cpp
)

target_include_directories(param_json PRIVATE
    ${PROJECT_SOURCE_DIR}/sdk/
    ${PROJECT_SOURCE_DIR}/firmware/
    ${PROJECT_SOURCE_DIR}/3rdparty/ConfiguringIP/
)

target_link_libraries(param_json PRIVATE
    open_cam3d_sdk
)

install(TARGETS param_json
    RUNTIME DESTINATION bin
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
)
