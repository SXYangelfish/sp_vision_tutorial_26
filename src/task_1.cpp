#include <chrono>
#include <opencv2/opencv.hpp>
#include <yaml-cpp/yaml.h>

#include "io/camera.hpp"
#include "io/gimbal/gimbal.hpp"
#include "tasks/auto_aim/solver.hpp"
#include "tasks/auto_aim/yolo.hpp"
#include "tools/img_tools.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/plotter.hpp"
#include "tools/recorder.hpp"
#include "tools/exiter.hpp"

const std::string keys =
  "{help h usage ? | | 输出命令行参数说明}"
  "{@config-path   | | yaml配置文件路径 }";

using namespace std::chrono_literals;

int main(int argc, char * argv[])
{
  cv::CommandLineParser cli(argc, argv, keys);
  auto config_path = cli.get<std::string>("@config-path");
  if (cli.has("help") || !cli.has("@config-path")) {
    cli.printMessage();
    return 0;
  }

  // 初始化工具类
  tools::Exiter exiter;
  tools::Plotter plotter;

  // 初始化io类
  io::Camera camera(config_path);
  io::Gimbal gimbal(config_path);

  // 初始化auto_aim类
  auto_aim::YOLO yolo(config_path, true);
  auto_aim::Solver solver(config_path);

  cv::Mat img;
  Eigen::Quaterniond q;
  std::chrono::steady_clock::time_point t;

  auto yaml = YAML::LoadFile(config_path);
  double yaw_offset = yaml["yaw_offset"].as<double>();
  double pitch_offset = yaml["pitch_offset"].as<double>();
  yaw_offset *= CV_PI / 180.0;  // 转换为弧度
  pitch_offset *= CV_PI / 180.0;  // 转换为弧度


  while (!exiter.exit()) {
    // Your code start
    
    gimbal.state();
    camera.read(img,t);
    q = gimbal.q(t);
    solver.set_R_gimbal2world(q);

    auto armors = yolo.detect(img);

    if(!armors.empty()){
      cv::Point2f img_center(img.cols / 2.0f, img.rows / 2.0f);
      // 选择识别到的第一个装甲板作为目标
      auto & target_armor = armors.front();
      

        solver.solve(target_armor);

        double target_yaw = target_armor.ypd_in_world[0];
        double target_pitch = -target_armor.ypd_in_world[1];

        const double max_pitch = 20.0 * CV_PI / 180.0;
        const double min_pitch = -20.0 * CV_PI / 180.0;
        target_pitch = std::clamp(target_pitch, min_pitch, max_pitch);

        gimbal.send(true , false, target_yaw , target_pitch );//弧度!!

        nlohmann::json plot_data;
        plot_data["control_mode"] = 1;
        plot_data["target_yaw"] = target_yaw ;
        plot_data["target_pitch"] = target_pitch ;
        plot_data["armor_x"] = target_armor.xyz_in_world.x();
        plot_data["armor_y"] = target_armor.xyz_in_world.y();
        plot_data["armor_z"] = target_armor.xyz_in_world.z();
        plot_data["detected_armors"] = armors.size();
        plotter.plot(plot_data);

        cv::circle(img, target_armor.center, 5, cv::Scalar(0, 255, 0), 2);
        cv::putText(img, "Target", target_armor.center + cv::Point2f(10, -10),
        cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 2);
    } 
    

    cv::imshow("Result", img);
    cv::waitKey(1);

    if(cv::waitKey(1) == 'q' || cv::waitKey(1) == 'Q')
    
    break;
    // Your code end
  }
  
  return 0;
}