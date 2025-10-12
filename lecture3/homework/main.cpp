#include <opencv2/opencv.hpp>
#include "io/my_camera.hpp"
#include "tasks/yolo.hpp"
#include "tasks/armor.hpp"
#include "tools/img_tools.hpp"

int main()
{

    myCamera cam;

    auto_aim::YOLO yolo("./configs/yolo.yaml");

    int frame_count = 0;
    while (true) {

        cv::Mat frame = cam.read();

        std::list<auto_aim::Armor> armors = yolo.detect(frame, frame_count++);


        for (const auto & armor : armors) {

            tools::draw_points(frame, armor.points, cv::Scalar(0, 0, 255), 2);

            cv::Point center_point(
                static_cast<int>(armor.center.x), 
                static_cast<int>(armor.center.y)
            );
            std::string label = auto_aim::ARMOR_NAMES[armor.name];
            tools::draw_text(frame, label, center_point, cv::Scalar(0, 255, 255), 0.7, 2);
        }


        cv::imshow("YOLO", frame);

        int key = cv::waitKey(1);
        if (key == 'Q' || key == 'q') {
            break;
        }
    }

    return 0;
}
