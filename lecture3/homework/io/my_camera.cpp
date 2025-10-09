#include "my_camera.hpp"
#include <unordered_map>
#include <opencv2/opencv.hpp>
#include <iostream>

myCamera::myCamera()
 {
    _handle_ = nullptr;
    int ret;
    ret = MV_CC_EnumDevices(MV_USB_DEVICE, &_device_list_);


    ret = MV_CC_CreateHandle(&_handle_, _device_list_.pDeviceInfo[0]);


    ret = MV_CC_OpenDevice(_handle_);


    MV_CC_SetEnumValue(_handle_, "BalanceWhiteAuto", MV_BALANCEWHITE_AUTO_CONTINUOUS);
    MV_CC_SetEnumValue(_handle_, "ExposureAuto", MV_EXPOSURE_AUTO_MODE_OFF);
    MV_CC_SetEnumValue(_handle_, "GainAuto", MV_GAIN_MODE_OFF);
    MV_CC_SetFloatValue(_handle_, "ExposureTime", 2000);
    MV_CC_SetFloatValue(_handle_, "Gain", 20);
    MV_CC_SetFrameRate(_handle_, 60);

    // 读取一帧图像
    ret = MV_CC_StartGrabbing(_handle_);
   
 }

myCamera::~myCamera() 
{
    if (_handle_)
     {
        MV_CC_StopGrabbing(_handle_);
        MV_CC_CloseDevice(_handle_);
        MV_CC_DestroyHandle(_handle_);
        _handle_ = nullptr;
    }
}



cv::Mat myCamera::_transfer(MV_FRAME_OUT& raw)
{
    cv::Mat img(cv::Size(raw.stFrameInfo.nWidth, raw.stFrameInfo.nHeight), CV_8U, raw.pBufAddr);
    
    MV_CC_PIXEL_CONVERT_PARAM cvt_param;

    cvt_param.nWidth = raw.stFrameInfo.nWidth;
    cvt_param.nHeight = raw.stFrameInfo.nHeight;

    cvt_param.pSrcData = raw.pBufAddr;
    cvt_param.nSrcDataLen = raw.stFrameInfo.nFrameLen;
    cvt_param.enSrcPixelType = raw.stFrameInfo.enPixelType;

    cvt_param.pDstBuffer = img.data;
    cvt_param.nDstBufferSize = img.total() * img.elemSize();
    cvt_param.enDstPixelType = PixelType_Gvsp_BGR8_Packed;

    auto pixel_type = raw.stFrameInfo.enPixelType;
    const static std::unordered_map<MvGvspPixelType, cv::ColorConversionCodes> type_map = {
      {PixelType_Gvsp_BayerGR8, cv::COLOR_BayerGR2RGB},
      {PixelType_Gvsp_BayerRG8, cv::COLOR_BayerRG2RGB},
      {PixelType_Gvsp_BayerGB8, cv::COLOR_BayerGB2RGB},
      {PixelType_Gvsp_BayerBG8, cv::COLOR_BayerBG2RGB}};
    cv::cvtColor(img, img, type_map.at(pixel_type));
    
    return img;
}




cv::Mat myCamera::read()
 {
    MV_FRAME_OUT raw;
    unsigned int nMsec = 100;
    int ret = MV_CC_GetImageBuffer(_handle_, &raw, nMsec);

    cv::Mat img = _transfer(raw);

    MV_CC_FreeImageBuffer(_handle_, &raw);
    return img;
}
