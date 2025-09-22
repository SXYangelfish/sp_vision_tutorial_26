#include "tools.hpp"
#include <opencv2/opencv.hpp>

cv::Mat fun(const cv::Mat& input, int target_width, int target_height, Params& params) 
{
    //计算缩放比例
    double scale_w = static_cast<double>(target_width) / input.cols;
    double scale_h = static_cast<double>(target_height) / input.rows;
    params.scale = std::min(scale_w, scale_h);
    
    //计算缩放后的尺寸
    int scaled_width = static_cast<int>(input.cols * params.scale);
    int scaled_height = static_cast<int>(input.rows * params.scale);
    
    //计算偏移
    params.offset_x = (target_width - scaled_width) / 2;
    params.offset_y = (target_height - scaled_height) / 2;
    
    //缩放图像
    cv::Mat scaled_image;
    cv::resize(input, scaled_image, cv::Size(scaled_width, scaled_height));
    
    //创建黑色画布
    cv::Mat canvas = cv::Mat::zeros(target_height, target_width, input.type());
    
    //将缩放后的图像放置在画布中央
    cv::Mat roi = canvas(cv::Rect(params.offset_x, params.offset_y, scaled_width, scaled_height));
    scaled_image.copyTo(roi);
    
    return canvas;
}