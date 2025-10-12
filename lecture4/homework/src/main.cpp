#include "tasks/buff_detector.hpp"
#include "io/camera.hpp"
#include <chrono>
#include <opencv2/opencv.hpp>
#include <nlohmann/json.hpp>
#include "tools/plotter.hpp"
#include "tasks/buff_solver.hpp"

int main()
{
    io::Camera camera(2.5,16.9,"2bdf:0001");
    std::chrono::steady_clock::time_point timestamp;

    tools::Plotter plotter;
    auto_buff::Buff_Detector detector;
    auto_buff::Buff_Solver solver;

    while(true){
        cv::Mat img;
        camera.read(img ,timestamp);
        auto fanblades = detector.detect(img);

        cv::Mat display_img = solver.visualizeResults(img,fanblades);

        if(!solver.displayAndWait(display_img)){
            break;
        }


        // plotjuggler
        nlohmann::json data;
        if(fanblades.size())
        {
            data["fanblade_point"] = fanblades[0].points.size();


             cv::Point2f rotation_center = solver.calculateRotationCenter(fanblades  [0]);
            data["rotation_center_x"] = rotation_center.x;
            data["rotation_center_y"] = rotation_center.y;
            
            data["fanblade_center_x"] = fanblades  [0].center.x;
            data["fanblade_center_y"] = fanblades  [0].center.y;
            
            data["fanblade_type"] = static_cast<int>(fanblades  [0].type);
            
            auto now = std::chrono::steady_clock::now();
            auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(
                now.time_since_epoch());
            data["timestamp_ms"] = duration.count();
        }
        else
        {
            data["fanblade_point"] = 0;
            data["rotation_center_x"] = 0.0;
            data["rotation_center_y"] = 0.0;
            data["fanblade_center_x"] = 0.0;
            data["fanblade_center_y"] = 0.0;
            data["fanblade_type"] = -1;
            data["timestamp_ms"] = std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::steady_clock::now().time_since_epoch()).count();
        }
        plotter.plot(data);
    }
    return 0;
}