#include "camera.h"


Camera::Camera()
{
    camera_opened_state_ = false;  
    trigger_on_flag_ = true;
  
    image_width_= 0;
    image_height_= 0;
    min_camera_exposure_ = 0;
    max_camera_exposure_ = 990000; 
    
    pixel_type_ = XemaPixelType::Mono;
}

Camera::~Camera()
{

}


bool Camera::getPixelType(XemaPixelType &type)
{
    type = pixel_type_;
    return true;
}

bool Camera::getMinExposure(float &val)
{
    val = min_camera_exposure_;
}

bool Camera::getImageSize(int &width,int &height)
{
    width = image_width_;
    height = image_height_;
}

bool Camera::openCamera()
{
    return false;
} 

bool Camera::closeCamera()
{

    return false;
}
  
bool Camera::switchToInternalTriggerMode()
{

    return false;
}

bool Camera::switchToExternalTriggerMode()
{

    return false;
}

 