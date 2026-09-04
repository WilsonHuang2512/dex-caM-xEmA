cmake_minimum_required(VERSION 3.16)
project(xema_camera_gui LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_AUTOMOC ON)

# The real XEMA-master build ships its own OpenCV 4.10.0 at 3rdparty/opencv_4_10_0_shared_md
# (standard official-OpenCV-Windows-package layout: OpenCVConfig.cmake sits next to the .lib
# files under x64/vc16/lib) -- point find_package(OpenCV) at that instead of relying on a
# system-wide OpenCV install, so this matches the exact version calibration.exe/open_cam3d_sdk
# are already built against. Override with -DOpenCV_DIR=... at configure time if this path is
# wrong for your checkout (e.g. a different OpenCV version folder name).
if(NOT DEFINED OpenCV_DIR)
	set(OpenCV_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../3rdparty/opencv_4_10_0_shared_md/x64/vc16/lib CACHE PATH "")
endif()

# Test component added alongside Widgets -- only the test targets below link Qt5::Test; this
# find_package call doesn't change anything about the real xema_camera_gui target.
find_package(Qt5 COMPONENTS Widgets Test REQUIRED)
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

# opencv_world4100.dll is what every OpenCV-linked target needs at RUNTIME (the .lib found via
# OpenCV_DIR above is only for link time) -- same STATUS_DLL_NOT_FOUND trap as open_cam3d_sdk.
# Exact path confirmed from XEMA-master/CMakeLists.txt's own install() step for the real GUI.
set(OPENCV_RUNTIME_DLLS
	${CMAKE_CURRENT_SOURCE_DIR}/../3rdparty/opencv_4_10_0_shared_md/x64/vc16/bin/opencv_world4100.dll
	${CMAKE_CURRENT_SOURCE_DIR}/../3rdparty/opencv_4_10_0_shared_md/x64/vc16/bin/opencv_videoio_ffmpeg4100_64.dll
)

# Copies open_cam3d_sdk.dll + opencv_world4100.dll next to the given target's exe. Call this
# for every target that links open_cam3d_sdk/OpenCV -- xema_camera_gui and every test target
# below all need it, tst_smoke doesn't (it links neither).
function(xema_copy_runtime_dlls target_name)
	add_custom_command(TARGET ${target_name} POST_BUILD
		COMMAND ${CMAKE_COMMAND} -E copy_if_different
			$<TARGET_FILE:open_cam3d_sdk>
			$<TARGET_FILE_DIR:${target_name}>
	)
	foreach(dll ${OPENCV_RUNTIME_DLLS})
		add_custom_command(TARGET ${target_name} POST_BUILD
			COMMAND ${CMAKE_COMMAND} -E copy_if_different
				${dll}
				$<TARGET_FILE_DIR:${target_name}>
		)
	endforeach()
endfunction()

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

xema_copy_runtime_dlls(xema_camera_gui)

# MSVC misreads UTF-8 source as GBK/936 on a Chinese Windows install unless told otherwise --
# breaks both our own Chinese u8"..." labels and the SDK's own Chinese comments.
if(MSVC)
	target_compile_options(xema_camera_gui PRIVATE /utf-8)
	target_compile_options(open_cam3d_sdk PRIVATE /utf-8)
endif()

# ============================================================================
# Bisection smoke test -- does QtTest itself work in this build, with no XemaCameraWindow
# construction at all? See tst_smoke.cpp's own comment for what this isolates. Links neither
# OpenCV nor the SDK, so it doesn't need xema_copy_runtime_dlls().
# ============================================================================

add_executable(tst_smoke tst_smoke.cpp)

if(WIN32)
	set_target_properties(tst_smoke PROPERTIES WIN32_EXECUTABLE FALSE)
endif()

target_link_libraries(tst_smoke PRIVATE
	Qt5::Widgets
	Qt5::Test
)

if(MSVC)
	target_compile_options(tst_smoke PRIVATE /utf-8)
endif()

# ============================================================================
# Bisection test -- does XemaCameraWindow construct successfully with nothing else involved?
# See tst_window_construct.cpp's own comment. This is what caught the FreeConsole() bug.
# ============================================================================

add_executable(tst_window_construct
	tst_window_construct.cpp
	XemaCameraWindow.cpp
	XemaCameraWindow.h
)

if(WIN32)
	set_target_properties(tst_window_construct PROPERTIES WIN32_EXECUTABLE FALSE)
endif()

target_compile_definitions(tst_window_construct PRIVATE XEMA_TEST_HOOKS)

target_link_libraries(tst_window_construct PRIVATE
	Qt5::Widgets
	Qt5::Test
	${OpenCV_LIBS}
	open_cam3d_sdk
)

target_include_directories(tst_window_construct PRIVATE
	${OpenCV_INCLUDE_DIRS}
	${XEMA_SDK_DIR}
)

xema_copy_runtime_dlls(tst_window_construct)

if(MSVC)
	target_compile_options(tst_window_construct PRIVATE /utf-8)
endif()

# ============================================================================
# Tests -- tst_xema_calib_logic (see xema_calib_tool_test_plan.md, Tier 1).
#
# Pure-logic tests: buildCalibrateArgs/buildCorrectArgs/calibModeFromConfig/
# boardSpacingFromConfig are static and callable with no instance; identityFolderName/
# currentIdentityFolder go through XEMA_TEST_HOOKS wrappers since they read UI widgets. No
# camera, no exe, no background thread -- runs in well under a second, unlike
# tst_xema_click_races (which has to wait out real TEST-NET timeouts).
# ============================================================================

add_executable(tst_xema_calib_logic
	tst_xema_calib_logic.cpp
	XemaCameraWindow.cpp
	XemaCameraWindow.h
)

if(WIN32)
	set_target_properties(tst_xema_calib_logic PROPERTIES WIN32_EXECUTABLE FALSE)
endif()

target_compile_definitions(tst_xema_calib_logic PRIVATE XEMA_TEST_HOOKS)

target_link_libraries(tst_xema_calib_logic PRIVATE
	Qt5::Widgets
	Qt5::Test
	${OpenCV_LIBS}
	open_cam3d_sdk
)

target_include_directories(tst_xema_calib_logic PRIVATE
	${OpenCV_INCLUDE_DIRS}
	${XEMA_SDK_DIR}
)

xema_copy_runtime_dlls(tst_xema_calib_logic)

if(MSVC)
	target_compile_options(tst_xema_calib_logic PRIVATE /utf-8)
endif()

enable_testing()
add_test(NAME tst_xema_calib_logic COMMAND tst_xema_calib_logic)

# ============================================================================
# Tests -- tst_xema_sequencing (see xema_calib_tool_test_plan.md, Tier 2B).
#
# fake_open_cam3d_sdk.cpp #includes the REAL open_cam3d.h and only ever DEFINES functions
# declared there, so signature mismatches are compile errors, not guessed ABI bugs. Built as
# its own DLL (fake_open_cam3d_sdk) -- a drop-in replacement for the real open_cam3d_sdk that
# XemaCameraWindow.cpp/.h are compiled against completely unmodified, just linked against a
# different DLL. tst_xema_sequencing links THIS instead of open_cam3d_sdk, so DfConnect/
# DfDisconnect/etc. succeed instantly and log every call -- no real sockets, no TEST-NET
# timeouts, and (unlike tst_xema_click_races) able to verify what a SUCCESSFUL
# disconnect/reconnect sequence actually does, not just that overlapping clicks are guarded.
# ============================================================================

add_library(fake_open_cam3d_sdk SHARED
	fake_open_cam3d_sdk.cpp
)

target_include_directories(fake_open_cam3d_sdk PRIVATE ${XEMA_SDK_DIR})

if(MSVC)
	target_compile_options(fake_open_cam3d_sdk PRIVATE /utf-8)
endif()

add_executable(tst_xema_sequencing
	tst_xema_sequencing.cpp
	XemaCameraWindow.cpp
	XemaCameraWindow.h
)

if(WIN32)
	set_target_properties(tst_xema_sequencing PROPERTIES WIN32_EXECUTABLE FALSE)
endif()

target_compile_definitions(tst_xema_sequencing PRIVATE XEMA_TEST_HOOKS)

target_link_libraries(tst_xema_sequencing PRIVATE
	Qt5::Widgets
	Qt5::Test
	${OpenCV_LIBS}
	fake_open_cam3d_sdk
)

target_include_directories(tst_xema_sequencing PRIVATE
	${OpenCV_INCLUDE_DIRS}
	${XEMA_SDK_DIR}
	${CMAKE_CURRENT_SOURCE_DIR}
)

# Deliberately does NOT call xema_copy_runtime_dlls() -- that copies the REAL open_cam3d_sdk,
# which this target must never link or load. Copies fake_open_cam3d_sdk + OpenCV only.
add_custom_command(TARGET tst_xema_sequencing POST_BUILD
	COMMAND ${CMAKE_COMMAND} -E copy_if_different
		$<TARGET_FILE:fake_open_cam3d_sdk>
		$<TARGET_FILE_DIR:tst_xema_sequencing>
)
foreach(dll ${OPENCV_RUNTIME_DLLS})
	add_custom_command(TARGET tst_xema_sequencing POST_BUILD
		COMMAND ${CMAKE_COMMAND} -E copy_if_different
			${dll}
			$<TARGET_FILE_DIR:tst_xema_sequencing>
	)
endforeach()

if(MSVC)
	target_compile_options(tst_xema_sequencing PRIVATE /utf-8)
endif()

enable_testing()
add_test(NAME tst_xema_sequencing COMMAND tst_xema_sequencing)

# ============================================================================
# Tests -- tst_xema_click_races (see xema_calib_tool_test_plan.md, Tier 2A).
#
# This target compiles a SECOND copy of XemaCameraWindow.cpp/.h (same source files as the
# real app, not a fork) linked against tst_xema_click_races.cpp instead of main.cpp. Set
# BUILD_XEMA_TESTS to OFF (-DBUILD_XEMA_TESTS=OFF at configure time) if you just want the app.
# ============================================================================

option(BUILD_XEMA_TESTS "Build tst_xema_click_races" ON)

if(BUILD_XEMA_TESTS)
	add_executable(tst_xema_click_races
		tst_xema_click_races.cpp
		XemaCameraWindow.cpp
		XemaCameraWindow.h
	)

	if(WIN32)
		# Deliberately a console app, unlike xema_camera_gui -- QTest prints pass/fail output
		# to stdout, which needs a console to actually be visible when run from a terminal.
		set_target_properties(tst_xema_click_races PROPERTIES WIN32_EXECUTABLE FALSE)
	endif()

	target_compile_definitions(tst_xema_click_races PRIVATE XEMA_TEST_HOOKS)

	target_link_libraries(tst_xema_click_races PRIVATE
		Qt5::Widgets
		Qt5::Test
		${OpenCV_LIBS}
		open_cam3d_sdk
	)

	target_include_directories(tst_xema_click_races PRIVATE
		${OpenCV_INCLUDE_DIRS}
		${XEMA_SDK_DIR}
	)

	xema_copy_runtime_dlls(tst_xema_click_races)

	if(MSVC)
		target_compile_options(tst_xema_click_races PRIVATE /utf-8)
	endif()

	# Registers with `ctest` so `ctest --test-dir <build-dir> -V` runs it too, if you use that.
	enable_testing()
	add_test(NAME tst_xema_click_races COMMAND tst_xema_click_races)
endif()
