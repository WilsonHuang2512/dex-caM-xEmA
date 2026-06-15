set(GTEST_ROOT_DIR ${PROJECT_SOURCE_DIR}/3rdparty/googletest_1_14_0_static_md)
set(GTEST_INCLUDE_DIR ${GTEST_ROOT_DIR}/include)
set(GTEST_LIB_DIR ${GTEST_ROOT_DIR}/lib)

if(NOT WIN32)
    message(STATUS "Skip utest_xema: non-Windows platform")
    return()
endif()

enable_testing()

add_executable(utest_xema
    utest_xema.cpp
    ${PROJECT_SOURCE_DIR}/firmware/easylogging++.cc
)

target_compile_definitions(utest_xema PRIVATE _CRT_SECURE_NO_WARNINGS)

target_include_directories(utest_xema PRIVATE
    ${PROJECT_SOURCE_DIR}/cpp/
    ${PROJECT_SOURCE_DIR}/firmware/
    ${PROJECT_SOURCE_DIR}/3rdparty/ConfiguringIP/
    ${GTEST_INCLUDE_DIR}
)

target_link_directories(utest_xema PRIVATE
    ${PROJECT_SOURCE_DIR}/3rdparty/ConfiguringIP/
)

target_link_libraries(utest_xema PRIVATE
    xemacpp
    enumerate
    ${GTEST_LIB_DIR}/gtest.lib
)

add_test(NAME utest_xema COMMAND utest_xema)
set_tests_properties(utest_xema PROPERTIES
    ENVIRONMENT "PATH=$<TARGET_FILE_DIR:utest_xema>\;$<TARGET_FILE_DIR:xemacpp>\;${PROJECT_SOURCE_DIR}/3rdparty/ConfiguringIP\;${PROJECT_SOURCE_DIR}/3rdparty/tbb_2021_12_shared_md\;$ENV{PATH}"
)

install(TARGETS utest_xema
    RUNTIME DESTINATION bin
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
)

install(FILES utest_xema_config.json
    DESTINATION config
)