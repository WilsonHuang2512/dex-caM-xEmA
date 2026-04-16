#pragma once
#include<iostream>
#include "../sdk/xema_enums.h"


class Camera
{
public:
	Camera();
	~Camera();

	virtual bool openCamera();

	virtual bool closeCamera(); 
	
	virtual bool switchToInternalTriggerMode();

	virtual bool switchToExternalTriggerMode();

	virtual bool getExposure(double &val){}; 
	virtual bool setExposure(double val){}; 
    
	virtual bool getGain(double &val){};
	virtual bool setGain(double val){};
	  
	virtual bool streamOn(){}; 
	virtual bool streamOff(){};

    virtual bool trigger_software(){};

    virtual bool grap(unsigned char* buf){}; 

    virtual bool grap(unsigned short* buf){};

	virtual bool setPixelFormat(int val){};

	virtual bool getPixelFormat(int &val){};


	bool getImageSize(int &width,int &height);
	
	bool getMinExposure(float &val);

	bool getPixelType(XemaPixelType &type);

protected:
 
	bool camera_opened_state_; 
 

	long int image_width_;
	long int image_height_;

	float min_camera_exposure_; 
	float max_camera_exposure_; 
	
	bool stream_off_flag_;
	bool trigger_on_flag_;

	XemaPixelType pixel_type_;
};

