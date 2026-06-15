add_executable(cpp_example
    cpp_example.cpp
)

target_include_directories(cpp_example PRIVATE
    ${PROJECT_SOURCE_DIR}/cpp/
    ${PROJECT_SOURCE_DIR}/firmware/
    ${PROJECT_SOURCE_DIR}/3rdparty/ConfiguringIP/
)

target_link_directories(cpp_example PRIVATE
    ${PROJECT_SOURCE_DIR}/3rdparty/ConfiguringIP/
)

target_link_libraries(cpp_example PRIVATE
    xemacpp
    enumerate
)

install(TARGETS cpp_example
    RUNTIME DESTINATION bin
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
)
