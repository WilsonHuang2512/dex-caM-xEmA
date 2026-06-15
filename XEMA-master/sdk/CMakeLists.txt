include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../3rdparty/ConfiguringIP/)

find_package(Threads)

add_library(open_cam3d_sdk SHARED 
    open_cam3d.cpp
    ../test/triangulation.cpp
    socket_tcp.cpp
    utils.cpp
    ../firmware/easylogging++.cc
    ../firmware/basic_function.cpp)

target_link_libraries(open_cam3d_sdk PRIVATE
    Threads::Threads
)

install(TARGETS open_cam3d_sdk
    RUNTIME DESTINATION bin
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
)
