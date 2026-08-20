cmake_minimum_required(VERSION 3.16)
project(xema_camera_gui LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_AUTOMOC ON)

find_package(Qt5 COMPONENTS Widgets REQUIRED)
find_package(OpenCV REQUIRED)

# Build the real XEMA SDK (XEMA-master/sdk -> CMake target "open_cam3d_sdk") as part of this
# project. Assumes this folder sits directly under XEMA-master, as a sibling of sdk/.
set(XEMA_SDK_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../sdk)
add_subdirectory(${XEMA_SDK_DIR} ${CMAKE_CURRENT_BINARY_DIR}/xema_sdk_build)

# open_cam3d.cpp includes <configuring_network.h> from the vendor 3rdparty/ConfiguringIP
# package, which isn't present in this checkout (it's a separate prebuilt-binary bundle, not
# part of the git source). sdk/CMakeLists.txt doesn't link "enumerate" (that's only needed by
# device-discovery example targets we don't use -- we connect by known IP), so this empty
# stub is enough to satisfy the #include; confirmed working in the previous build.
target_include_directories(open_cam3d_sdk PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/stub_3rdparty)

add_executable(xema_camera_gui
	main.cpp
	XemaCameraWindow.cpp
	XemaCameraWindow.h
)

if(WIN32)
	set_target_properties(xema_camera_gui PROPERTIES WIN32_EXECUTABLE TRUE)
endif()

target_link_libraries(xema_camera_gui PRIVATE
	Qt5::Widgets
	${OpenCV_LIBS}
	open_cam3d_sdk
)

target_include_directories(xema_camera_gui PRIVATE
	${OpenCV_INCLUDE_DIRS}
	${XEMA_SDK_DIR}
)

# open_cam3d_sdk is a DLL, built into its own output dir -- copy it next to our exe or
# DfConnect/DfCaptureBrightnessData etc. fail to resolve at load time (STATUS_DLL_NOT_FOUND).
add_custom_command(TARGET xema_camera_gui POST_BUILD
	COMMAND ${CMAKE_COMMAND} -E copy_if_different
		$<TARGET_FILE:open_cam3d_sdk>
		$<TARGET_FILE_DIR:xema_camera_gui>
)

# MSVC misreads UTF-8 source as GBK/936 on a Chinese Windows install unless told otherwise --
# breaks both our own Chinese u8"..." labels and the SDK's own Chinese comments.
if(MSVC)
	target_compile_options(xema_camera_gui PRIVATE /utf-8)
	target_compile_options(open_cam3d_sdk PRIVATE /utf-8)
endif()
