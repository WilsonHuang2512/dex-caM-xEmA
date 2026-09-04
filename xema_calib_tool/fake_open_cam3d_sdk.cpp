// fake_open_cam3d_sdk.cpp
//
// A fake implementation of the 14 Df* functions XemaCameraWindow.cpp actually calls (confirmed
// via `grep -oE "\bDf[A-Za-z0-9_]*\(" XemaCameraWindow.cpp | sort -u`), built as a drop-in
// replacement for the real open_cam3d_sdk DLL in test builds. Unlike fake_exe_stub.cpp (a
// separate process, controlled via env vars), this is a DLL loaded into the SAME process as
// the test executable -- so control and inspection happen through ordinary C++ function calls
// (FakeSdk_*), not env vars or files.
//
// SIGNATURE SAFETY: this file #includes the REAL open_cam3d.h and only ever DEFINES functions
// declared there -- it never retypes a signature by hand. If a signature here doesn't match
// the real header, this simply fails to COMPILE (wrong parameter types/const-ness), rather
// than silently linking a subtly-wrong ABI. That's deliberate and is the whole point of doing
// it this way instead of guessing from documentation/memory.
//
// Build as a SHARED library (see CMakeLists.txt's fake_open_cam3d_sdk target) that mimics the
// real open_cam3d_sdk target's DLL/import-lib pair closely enough that a test executable can
// link against THIS instead, with zero changes to XemaCameraWindow.cpp/.h.

#include "open_cam3d.h"
#include <string>
#include <vector>
#include <mutex>
#include <cstring>
#include <thread>
#include <chrono>

namespace
{
	std::mutex g_state_mutex;

	// ---- configurable results (defaults are all "everything succeeds instantly") ----
	int g_connect_result = DF_SUCCESS;
	int g_connect_delay_ms = 0;
	int g_disconnect_result = DF_SUCCESS;
	int g_capture_engine_result = DF_SUCCESS;
	int g_camera_resolution_width = 1920;
	int g_camera_resolution_height = 1200;
	int g_projector_version = 3010;
	int g_pixel_type = 0; // XemaPixelType::Mono
	std::string g_firmware_version = "1.0.0-fake";
	int g_capture_data_result = DF_SUCCESS;
	int g_brightness_data_result = DF_SUCCESS;
	int g_led_current = 0;
	float g_camera_exposure = 10000.0f;
	float g_camera_gain = 1.0f;

	bool g_connected = false;

	// ---- call log: every call, in order, as "FunctionName(arg summary)" ----
	std::vector<std::string> g_call_log;

	void LogCall(const std::string& entry)
	{
		std::lock_guard<std::mutex> lock(g_state_mutex);
		g_call_log.push_back(entry);
	}
}

// ==================== test control API (not part of the real SDK) ====================
// Exported so a test executable linking this DLL can configure/inspect it directly. Prefixed
// FakeSdk_ to make it unmistakable these aren't real SDK entry points.

extern "C"
{
	__declspec(dllexport) void FakeSdk_Reset()
	{
		std::lock_guard<std::mutex> lock(g_state_mutex);
		g_connect_result = DF_SUCCESS;
		g_connect_delay_ms = 0;
		g_disconnect_result = DF_SUCCESS;
		g_capture_engine_result = DF_SUCCESS;
		g_camera_resolution_width = 1920;
		g_camera_resolution_height = 1200;
		g_projector_version = 3010;
		g_pixel_type = 0;
		g_firmware_version = "1.0.0-fake";
		g_capture_data_result = DF_SUCCESS;
		g_brightness_data_result = DF_SUCCESS;
		g_led_current = 0;
		g_camera_exposure = 10000.0f;
		g_camera_gain = 1.0f;
		g_connected = false;
		g_call_log.clear();
	}

	__declspec(dllexport) void FakeSdk_SetConnectResult(int result) { std::lock_guard<std::mutex> lock(g_state_mutex); g_connect_result = result; }
	__declspec(dllexport) void FakeSdk_SetConnectDelayMs(int ms) { std::lock_guard<std::mutex> lock(g_state_mutex); g_connect_delay_ms = ms; }
	__declspec(dllexport) void FakeSdk_SetDisconnectResult(int result) { std::lock_guard<std::mutex> lock(g_state_mutex); g_disconnect_result = result; }
	__declspec(dllexport) void FakeSdk_SetProjectorVersion(int version) { std::lock_guard<std::mutex> lock(g_state_mutex); g_projector_version = version; }
	__declspec(dllexport) void FakeSdk_SetPixelType(int type) { std::lock_guard<std::mutex> lock(g_state_mutex); g_pixel_type = type; }
	__declspec(dllexport) void FakeSdk_SetFirmwareVersion(const char* v) { std::lock_guard<std::mutex> lock(g_state_mutex); g_firmware_version = v; }

	__declspec(dllexport) bool FakeSdk_IsConnected()
	{
		std::lock_guard<std::mutex> lock(g_state_mutex);
		return g_connected;
	}

	// Call-log accessors use a C-safe interface (index + fixed buffer), not std::vector/
	// std::string return values, to avoid any doubt about STL-across-DLL-boundary safety --
	// deliberately more conservative here than fake_exe_stub.cpp needs to be, since this is
	// linked in-process rather than spawned as a separate exe.
	__declspec(dllexport) int FakeSdk_GetCallCount()
	{
		std::lock_guard<std::mutex> lock(g_state_mutex);
		return (int)g_call_log.size();
	}

	__declspec(dllexport) void FakeSdk_GetCallAt(int index, char* buf, int buf_size)
	{
		std::lock_guard<std::mutex> lock(g_state_mutex);
		if (index < 0 || index >= (int)g_call_log.size() || buf_size <= 0)
		{
			if (buf_size > 0) buf[0] = '\0';
			return;
		}
		std::strncpy(buf, g_call_log[index].c_str(), buf_size - 1);
		buf[buf_size - 1] = '\0';
	}
}

// ==================== the 14 real Df* functions XemaCameraWindow.cpp calls ====================

DF_SDK_API int DfConnect(const char* camera_id)
{
	LogCall(std::string("DfConnect(") + (camera_id ? camera_id : "null") + ")");

	int delay_ms;
	int result;
	{
		std::lock_guard<std::mutex> lock(g_state_mutex);
		delay_ms = g_connect_delay_ms;
		result = g_connect_result;
	}

	if (delay_ms > 0)
	{
		std::this_thread::sleep_for(std::chrono::milliseconds(delay_ms));
	}

	if (result == DF_SUCCESS)
	{
		std::lock_guard<std::mutex> lock(g_state_mutex);
		g_connected = true;
	}
	return result;
}

DF_SDK_API int DfDisconnect(const char* camera_id)
{
	LogCall(std::string("DfDisconnect(") + (camera_id ? camera_id : "null") + ")");

	int result;
	{
		std::lock_guard<std::mutex> lock(g_state_mutex);
		result = g_disconnect_result;
	}

	if (result == DF_SUCCESS)
	{
		std::lock_guard<std::mutex> lock(g_state_mutex);
		g_connected = false;
	}
	return result;
}

DF_SDK_API int DfSetCaptureEngine(XemaEngine engine)
{
	LogCall("DfSetCaptureEngine(" + std::to_string((int)engine) + ")");
	std::lock_guard<std::mutex> lock(g_state_mutex);
	return g_capture_engine_result;
}

DF_SDK_API int DfGetCameraResolution(int* width, int* height)
{
	LogCall("DfGetCameraResolution");
	std::lock_guard<std::mutex> lock(g_state_mutex);
	if (width) *width = g_camera_resolution_width;
	if (height) *height = g_camera_resolution_height;
	return DF_SUCCESS;
}

DF_SDK_API int DfCaptureData(int exposure_num, char* timestamp)
{
	LogCall("DfCaptureData(" + std::to_string(exposure_num) + ")");
	if (timestamp) timestamp[0] = '\0';
	std::lock_guard<std::mutex> lock(g_state_mutex);
	return g_capture_data_result;
}

DF_SDK_API int DfGetBrightnessData(unsigned char* brightness)
{
	LogCall("DfGetBrightnessData");
	int width, height, result;
	{
		std::lock_guard<std::mutex> lock(g_state_mutex);
		width = g_camera_resolution_width;
		height = g_camera_resolution_height;
		result = g_brightness_data_result;
	}
	if (brightness && result == DF_SUCCESS)
	{
		std::memset(brightness, 128, (size_t)width * (size_t)height); // flat mid-grey fake frame
	}
	return result;
}

DF_SDK_API int DfGetCameraPixelType(int& type)
{
	LogCall("DfGetCameraPixelType");
	std::lock_guard<std::mutex> lock(g_state_mutex);
	type = g_pixel_type;
	return DF_SUCCESS;
}

DF_SDK_API int DfGetFirmwareVersion(char* version)
{
	LogCall("DfGetFirmwareVersion");
	if (!version) return DF_FAILED;
	std::lock_guard<std::mutex> lock(g_state_mutex);
	std::strncpy(version, g_firmware_version.c_str(), 63);
	version[63] = '\0';
	return DF_SUCCESS;
}

DF_SDK_API int DfGetProjectorVersion(int& version)
{
	LogCall("DfGetProjectorVersion");
	std::lock_guard<std::mutex> lock(g_state_mutex);
	version = g_projector_version;
	return DF_SUCCESS;
}

DF_SDK_API int DfSetParamLedCurrent(int led)
{
	LogCall("DfSetParamLedCurrent(" + std::to_string(led) + ")");
	std::lock_guard<std::mutex> lock(g_state_mutex);
	g_led_current = led;
	return DF_SUCCESS;
}

DF_SDK_API int DfGetParamLedCurrent(int& led)
{
	LogCall("DfGetParamLedCurrent");
	std::lock_guard<std::mutex> lock(g_state_mutex);
	led = g_led_current;
	return DF_SUCCESS;
}

DF_SDK_API int DfSetParamCameraExposure(float exposure)
{
	LogCall("DfSetParamCameraExposure(" + std::to_string(exposure) + ")");
	std::lock_guard<std::mutex> lock(g_state_mutex);
	g_camera_exposure = exposure;
	return DF_SUCCESS;
}

DF_SDK_API int DfGetParamCameraExposure(float& exposure)
{
	LogCall("DfGetParamCameraExposure");
	std::lock_guard<std::mutex> lock(g_state_mutex);
	exposure = g_camera_exposure;
	return DF_SUCCESS;
}

DF_SDK_API int DfSetParamCameraGain(float gain)
{
	LogCall("DfSetParamCameraGain(" + std::to_string(gain) + ")");
	std::lock_guard<std::mutex> lock(g_state_mutex);
	g_camera_gain = gain;
	return DF_SUCCESS;
}

DF_SDK_API int DfGetParamCameraGain(float& gain)
{
	LogCall("DfGetParamCameraGain");
	std::lock_guard<std::mutex> lock(g_state_mutex);
	gain = g_camera_gain;
	return DF_SUCCESS;
}
