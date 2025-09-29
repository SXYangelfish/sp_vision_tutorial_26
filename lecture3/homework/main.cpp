#include "io/my_camera.hpp"
#include "tasks/yolo.hpp"
#include "opencv2/opencv.hpp"
#include "tools/img_tools.hpp"

int main()
{

    myCamera cam;

    YOLO yolo("yolo_model.onnx", 0.5); 

    cv::Mat img;
    
    while (true)
    {

        img = cam.read();


        std::vector<cv::Point> armor_centers = yolo.detect(img);


        for (const auto &pt : armor_centers) {
            tools::draw_point(img, pt, {0, 0, 255}, 5);
        }

        cv::Mat disp;
        cv::resize(img, disp, cv::Size(640, 480));
        cv::imshow("Camera & YOLO", disp);


        char key = static_cast<char>(cv::waitKey(1));
        if (key == 'q' || key == 27) {
            break;
        }
    }


    return 0;
}