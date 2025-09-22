#ifndef TOOLS_HPP
#define TOOLS_HPP

#include <opencv2/opencv.hpp>

struct Params 
{
    double scale;
    int offset_x;
    int offset_y;
};

cv::Mat fun(const cv::Mat& input, int target_width, int target_height, Params& params);
#endif // TOOLS_HPP