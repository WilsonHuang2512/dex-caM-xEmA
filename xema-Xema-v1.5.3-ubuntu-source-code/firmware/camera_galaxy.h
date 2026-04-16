#pragma once
#include "GxIAPI.h" 
#include "camera.h"
#include <chrono>         // std::chrono::milliseconds
#include <thread>         // std::thread
#include <mutex>          // std::timed_mutex

class CameraGalaxy: public Camera
{
public:
	CameraGalaxy();
	~CameraGalaxy();

	bool openCamera(); 
	bool closeCamera(); 

	bool switchToInternalTriggerMode(); 
	bool switchToExternalTriggerMode();

	bool getExposure(double &val); 
	bool setExposure(double val); 

	bool getGain(double &value);
	bool setGain(double value);  

	bool streamOn(); 
	bool streamOff();
 
    bool trigger_software();
    bool grap(unsigned char* buf);

private:
	void streamOffThread();

private:
   
 
    GX_DEV_HANDLE hDevice_;
    
    PGX_FRAME_BUFFER pFrameBuffer_; 
 
	std::mutex operate_mutex_;
  
};