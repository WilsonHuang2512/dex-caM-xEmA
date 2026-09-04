#pragma once
// Declarations for fake_open_cam3d_sdk.cpp's test-control API. Included by test files that
// link against fake_open_cam3d_sdk instead of the real open_cam3d_sdk. Not part of the real
// SDK -- never included by XemaCameraWindow.cpp/.h itself.

#ifdef _WIN32
#define FAKE_SDK_API __declspec(dllimport)
#else
#define FAKE_SDK_API
#endif

extern "C"
{
	FAKE_SDK_API void FakeSdk_Reset();
	FAKE_SDK_API void FakeSdk_SetConnectResult(int result);
	FAKE_SDK_API void FakeSdk_SetConnectDelayMs(int ms);
	FAKE_SDK_API void FakeSdk_SetDisconnectResult(int result);
	FAKE_SDK_API void FakeSdk_SetProjectorVersion(int version);
	FAKE_SDK_API void FakeSdk_SetPixelType(int type);
	FAKE_SDK_API void FakeSdk_SetFirmwareVersion(const char* v);
	FAKE_SDK_API bool FakeSdk_IsConnected();
	FAKE_SDK_API int FakeSdk_GetCallCount();
	FAKE_SDK_API void FakeSdk_GetCallAt(int index, char* buf, int buf_size);
}
