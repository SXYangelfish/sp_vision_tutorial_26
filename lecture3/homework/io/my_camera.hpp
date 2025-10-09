#pragma once
#include "hikrobot/include/MvCameraControl.h"
#include <opencv2/opencv.hpp>

class myCamera 
{
public:
    myCamera();
    ~myCamera();
    cv::Mat read();

private:
    void* _handle_;
    MV_CC_DEVICE_INFO_LIST _device_list_; 

    cv::Mat _transfer(MV_FRAME_OUT& raw); 
};
