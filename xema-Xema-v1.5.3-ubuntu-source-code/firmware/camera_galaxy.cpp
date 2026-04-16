#include "camera_galaxy.h"
#include "easylogging++.h"

CameraGalaxy::CameraGalaxy()
{
}
CameraGalaxy::~CameraGalaxy()
{
}


bool CameraGalaxy::trigger_software()
{
    GX_STATUS status = GX_STATUS_SUCCESS;
    // 发送软触发命令
	status = GXSendCommand(hDevice_,GX_COMMAND_TRIGGER_SOFTWARE);
	if(status != GX_STATUS_SUCCESS)
	{
        LOG(ERROR) << "GX_COMMAND_TRIGGER_SOFTWARE ERROR!";
		return false;
	} 

    return true;
}

bool CameraGalaxy::grap(unsigned char *buf)
{
    std::lock_guard<std::mutex> my_guard(operate_mutex_);
 

    GX_STATUS status = GX_STATUS_SUCCESS;

    LOG(INFO) << "capture:";
    status = GXDQBuf(hDevice_, &pFrameBuffer_, 1000);
    LOG(INFO) << "status=" << status;
    if (status != GX_STATUS_SUCCESS)
    {
        status = GXQBuf(hDevice_, pFrameBuffer_); 
        return false;
    }

    if (pFrameBuffer_->nStatus == GX_FRAME_STATUS_SUCCESS)
    {
        int img_rows = pFrameBuffer_->nHeight;
        int img_cols = pFrameBuffer_->nWidth;
        int img_size = img_rows * img_cols;

        memcpy(buf, pFrameBuffer_->pImgBuf, img_size);
    }

    status = GXQBuf(hDevice_, pFrameBuffer_);
    if (status != GX_STATUS_SUCCESS)
    { 
        return false;
    }

    return true;
}

bool CameraGalaxy::streamOn()
{

    std::lock_guard<std::mutex> my_guard(operate_mutex_);
 

 

    GX_STATUS status = GX_STATUS_SUCCESS;

    LOG(INFO) << "GXStreamOn";
    status = GXStreamOn(hDevice_);
    if (status != GX_STATUS_SUCCESS)
    {

        LOG(INFO) << "Stream On Error: " << status;
        return false;
    }

    return true;
}

void CameraGalaxy::streamOffThread()
{
    std::lock_guard<std::mutex> my_guard(operate_mutex_); 
  
    stream_off_flag_ = true;
    GXStreamOff(hDevice_);
    LOG(INFO) << "Thread GXStreamOff"; 
}

bool CameraGalaxy::streamOff()
{
    stream_off_flag_ = false;

    std::thread stop_thread(&CameraGalaxy::streamOffThread, this);
    stop_thread.detach();

    while (!stream_off_flag_)
    { 
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    return true;
}

bool CameraGalaxy::openCamera()
{
    std::lock_guard<std::mutex> my_guard(operate_mutex_);
 

    GX_STATUS status = GX_STATUS_SUCCESS;
    uint32_t nDeviceNum = 0;

    status = GXInitLib();
    if (status != GX_STATUS_SUCCESS)
    {
        return false;
    }

    status = GXUpdateDeviceList(&nDeviceNum, 1000);
    if ((status != GX_STATUS_SUCCESS) || (nDeviceNum <= 0))
    {
        return false;
    }

    char cam_idx[8] = "0";
    if (status == GX_STATUS_SUCCESS && nDeviceNum > 0)
    {
        GX_DEVICE_BASE_INFO *pBaseinfo = new GX_DEVICE_BASE_INFO[nDeviceNum];
        size_t nSize = nDeviceNum * sizeof(GX_DEVICE_BASE_INFO);
        // Gets the basic information of all devices.
        status = GXGetAllDeviceBaseInfo(pBaseinfo, &nSize);
        for (int i = 0; i < nDeviceNum; i++)
        {
            if (GX_DEVICE_CLASS_U3V == pBaseinfo[i].deviceClass)
            {
                // camera index starts from 1
                snprintf(cam_idx, 8, "%d", i + 1);
            }
        }

        delete[] pBaseinfo;
    }

    GX_OPEN_PARAM stOpenParam;
    stOpenParam.accessMode = GX_ACCESS_EXCLUSIVE;
    stOpenParam.openMode = GX_OPEN_INDEX;
    stOpenParam.pszContent = cam_idx;
    status = GXOpenDevice(&stOpenParam, &hDevice_);

    if (status != GX_STATUS_SUCCESS)
    {

        LOG(INFO) << "Open Camera Error!";
        return false;
    }

    if (status == GX_STATUS_SUCCESS)
    {

        /***********************************************************************************************/

        status = GXSetFloat(hDevice_, GX_FLOAT_EXPOSURE_TIME, 12000.0);

        status = GXSetEnum(hDevice_, GX_ENUM_EXPOSURE_AUTO, GX_EXPOSURE_AUTO_OFF);


        status = GXSetEnum(hDevice_, GX_ENUM_GAIN_SELECTOR, GX_GAIN_SELECTOR_ALL);

        status = GXSetFloat(hDevice_, GX_FLOAT_GAIN, 0.0);

        status = GXSetEnum(hDevice_, GX_ENUM_LINE_SELECTOR, GX_ENUM_LINE_SELECTOR_LINE2);

        status = GXSetEnum(hDevice_, GX_ENUM_LINE_MODE, GX_ENUM_LINE_MODE_INPUT);


        // emStatus = GXSetEnum(hDevice_, GX_ENUM_LINE_SOURCE, GX_ENUM_LINE_SOURCE_STROBE);
        status = GXSetEnum(hDevice_, GX_ENUM_TRIGGER_ACTIVATION, GX_TRIGGER_ACTIVATION_RISINGEDGE);
        status = GXSetEnum(hDevice_, GX_ENUM_TRIGGER_SOURCE, GX_TRIGGER_SOURCE_LINE2);

        status = GXSetAcqusitionBufferNumber(hDevice_, 72);
        camera_opened_state_ = true;

        status = GXGetInt(hDevice_, GX_INT_WIDTH, &image_width_);
        status = GXGetInt(hDevice_, GX_INT_HEIGHT, &image_height_);

        double max_frame = 0;
        status = GXGetFloat(hDevice_, GX_FLOAT_ACQUISITION_FRAME_RATE, &max_frame);

        LOG(INFO) << "max frame: " << max_frame;
        min_camera_exposure_ = 1000000/((int)max_frame);
        LOG(INFO) << "min_camera_exposure_: " << min_camera_exposure_;

        trigger_on_flag_ = true;
    }

    return true;
}
bool CameraGalaxy::closeCamera()
{
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    std::lock_guard<std::mutex> my_guard(operate_mutex_);
 

    if (!camera_opened_state_)
    {
        return false;
    }

    GX_STATUS status = GX_STATUS_SUCCESS;
    status = GXCloseDevice(hDevice_);
    status = GXCloseLib();

    camera_opened_state_ = false;

    return true;
}
bool CameraGalaxy::switchToInternalTriggerMode()
{
    std::lock_guard<std::mutex> my_guard(operate_mutex_);
 

    GX_STATUS status;

 
    status = GXSetEnum(hDevice_, GX_ENUM_TRIGGER_MODE, GX_TRIGGER_MODE_ON);  
    if (GX_STATUS_SUCCESS != status)
    {
        LOG(INFO) << "GX_ENUM_TRIGGER_MODE Error: " << status;
        return false;
    }

    status =  GXSetEnum(hDevice_,GX_ENUM_TRIGGER_SOURCE, GX_TRIGGER_SOURCE_SOFTWARE);
    if (GX_STATUS_SUCCESS != status)
    {
        LOG(INFO) << "GX_TRIGGER_SOURCE_SOFTWARE Error: " << status;
        return false;
    }

 
    // 曝光完成事件使能
	status = GXSetEnum(hDevice_, GX_ENUM_EVENT_NOTIFICATION, GX_ENUM_EVENT_NOTIFICATION_ON);
    if (GX_STATUS_SUCCESS != status)
    {
        LOG(INFO) << "GX_ENUM_EVENT_NOTIFICATION_ON Error: " << status;
        return false;
    }

    trigger_on_flag_ = false;
 
    return true;
}
bool CameraGalaxy::switchToExternalTriggerMode()
{
    
    std::lock_guard<std::mutex> my_guard(operate_mutex_);
 

    GX_STATUS status;
    status = GXSetEnum(hDevice_, GX_ENUM_TRIGGER_SOURCE, GX_TRIGGER_SOURCE_LINE2);

    if (GX_STATUS_SUCCESS != status)
    {
        LOG(INFO) << "GX_TRIGGER_SOURCE_LINE2 Error: " << status;
        return false;
    }

    status = GXSetEnum(hDevice_, GX_ENUM_TRIGGER_MODE, GX_TRIGGER_MODE_ON);
    if (status != GX_STATUS_SUCCESS)
    {

        LOG(INFO) << "GX_TRIGGER_MODE_ON Error: " << status;
        return false;
    }

    trigger_on_flag_ = true;

    return true;
}

bool CameraGalaxy::getExposure(double &val)
{
 

    GX_STATUS status = GX_STATUS_SUCCESS;
    status = GXGetFloat(hDevice_, GX_FLOAT_EXPOSURE_TIME, &val);

    if (status != GX_STATUS_SUCCESS)
    {

        LOG(INFO) << "Error Status: " << status;
        return false;
    }

    return true;
}
bool CameraGalaxy::setExposure(double val)
{

    if (trigger_on_flag_)
    {
        if (val < min_camera_exposure_)
        {
            val = min_camera_exposure_;
        }
    }

    if (val > max_camera_exposure_)
    {
        val = max_camera_exposure_;
    }

    GX_STATUS status = GX_STATUS_SUCCESS;
    status = GXSetFloat(hDevice_, GX_FLOAT_EXPOSURE_TIME, val);

    if (status != GX_STATUS_SUCCESS)
    {
        LOG(INFO) << "Error Status: " << status;
        return false;
    }

    return true;
}

bool CameraGalaxy::getGain(double &val)
{
    std::lock_guard<std::mutex> my_guard(operate_mutex_);
 

    GX_STATUS status = GX_STATUS_SUCCESS;
    status = GXGetFloat(hDevice_, GX_FLOAT_GAIN, &val);

    if (status != GX_STATUS_SUCCESS)
    {

        LOG(INFO) << "Error Status: " << status;
        return false;
    }

    return true;
}
bool CameraGalaxy::setGain(double val)
{
    std::lock_guard<std::mutex> my_guard(operate_mutex_);
 

    GX_STATUS status = GX_STATUS_SUCCESS;
    status = GXSetFloat(hDevice_, GX_FLOAT_GAIN, val);

    if (status != GX_STATUS_SUCCESS)
    {

        LOG(INFO) << "Error Status: " << status;
        return false;
    }

    return true;
}
