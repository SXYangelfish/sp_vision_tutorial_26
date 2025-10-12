#include "tasks/buff_detector.hpp"
#include "io/camera.hpp"
#include <chrono>
#include <opencv2/opencv.hpp>
#include <nlohmann/json.hpp>
#include "tools/plotter.hpp"
#include "tasks/buff_solver.hpp"

int main()
{
    io::Camera camera (2.5,16.9,"2bdf:0001");
    std::chrono::steady_clock::time_point timestamp;

    tools::Plotter plotter;
    auto_buff::Buff_Detector detector;
    auto_buff::Buff_Solver solver;

    while(true){
        cv::Mat img;
        camera.read(img,timestamp);
        auto fanblades = detector.detect(img);

        cv::Mat display_img = solver.visualizeResults(img,fanblades);

        if(!solver.displayAndWait(display_img)){
            break;
        }

        // plotjuggler
        nlohmann::json data;
        if(fanblades.size())
        {

            cv::Mat rvec, tvec;
    
            //solvePnP
            if (fanblades.size() > 0) {
            solver.solvePnP(fanblades [0], rvec, tvec);  // 使用第一个检测到的叶片
        
            data["pose_x"] = tvec.at<double>(0);
            data["pose_y"] = tvec.at<double>(1);
            data["pose_z"] = tvec.at<double>(2); 
        
            // 欧拉角
            cv::Mat rotation_matrix;
            cv::Rodrigues(rvec, rotation_matrix);
        
            double pitch = atan2(-rotation_matrix.at<double>(2, 0), 
                            sqrt(pow(rotation_matrix.at<double>(2, 1), 2) + 
                                    pow(rotation_matrix.at<double>(2, 2), 2))) * 180 / CV_PI;
            double yaw = atan2(rotation_matrix.at<double>(1, 0), 
                            rotation_matrix.at<double>(0, 0)) * 180 / CV_PI;
            double roll = atan2(rotation_matrix.at<double>(2, 1), 
                            rotation_matrix.at<double>(2, 2)) * 180 / CV_PI;
        
            data["pitch"] = pitch;
            data["yaw"] = yaw;
            data["roll"] = roll;

            cv::Point3f rotation_center = solver.calculateRotationCenter(fanblades); 
            data["rotation_center_x"] = rotation_center.x;
            data["rotation_center_y"] = rotation_center.y;
            data["rotation_center_z"] = rotation_center.z;
            } 
        else {
        data["pose_x"] = 0.0;
        data["pose_y"] = 0.0;
        data["pose_z"] = 0.0;
        data["pitch"] = 0.0;
        data["yaw"] = 0.0;
        data["roll"] = 0.0;
        }
    
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
            data["pose_x"] = 0.0;
            data["pose_y"] = 0.0;
            data["pose_z"] = 0.0;
            data["pitch"] = 0.0;
            data["yaw"] = 0.0;
            data["roll"] = 0.0;
            data["timestamp_ms"] = std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::steady_clock::now().time_since_epoch()).count();
        }
        plotter.plot(data);
    }
    return 0;
}