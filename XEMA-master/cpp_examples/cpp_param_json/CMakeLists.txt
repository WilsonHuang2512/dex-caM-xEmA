add_executable(CPP_param_json
    CPP_param_json.cpp
)

target_include_directories(CPP_param_json PRIVATE
    ${PROJECT_SOURCE_DIR}/cpp/
    ${PROJECT_SOURCE_DIR}/firmware/
    ${PROJECT_SOURCE_DIR}/3rdparty/ConfiguringIP/
)

target_link_libraries(CPP_param_json PRIVATE
    xemacpp
)

install(TARGETS CPP_param_json
    RUNTIME DESTINATION bin
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
)
