#include "tools.hpp"
#include <opencv2/opencv.hpp>
#include <fmt/core.h>

int main(int argc, char **argv)
{
     // get image
    if (argc < 2)
    {
        std::cerr << "Usage: " << argv[0] << " <input_image>\n";
        return 1;
    }
    std::string inPath = argv[1];
    std::cout << "Input image path: " << inPath;
   
 cv::Mat image = cv::imread(inPath);
    if (image.empty()) {
        fmt::print("fail to load image\n");
        return -1;
    }
    // scale image to 640*640
    Params params;
    cv::Mat result = fun(image, 640, 640, params);

    //显示图片
    cv::imshow("Scaled Image", result);
    cv::waitKey(0);

    //输出参数
    fmt::print("\nscale sizen : {:.4f}\n",params.scale);
    fmt::print("X_offset: {}\n", params.offset_x);
    fmt::print("Y_offset: {}\n", params.offset_y);

    return 0;
}