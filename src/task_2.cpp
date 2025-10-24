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
#include "tools/trajectory.hpp"

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

    // 从配置读取yaw、pitch偏移
    auto yaml = YAML::LoadFile(config_path);
    double yaw_offset = yaml["yaw_offset"].as<double>() * CV_PI / 180.0;
    double pitch_offset = yaml["pitch_offset"].as<double>() * CV_PI / 180.0;

   while (!exiter.exit())
{
    // === 获取状态与图像 ===
    auto gimbal_state = gimbal.state();
    camera.read(img, t);
    q = gimbal.q(t);
    solver.set_R_gimbal2world(q);

    // === 检测目标 ===
    auto armors = yolo.detect(img);
    if (armors.empty()) continue;

    // 选择最靠近中心的目标
    cv::Point2f img_center(img.cols / 2.0f, img.rows / 2.0f);
    auto target_armor = *std::min_element(
        armors.begin(), armors.end(),
        [img_center](const auto &a, const auto &b) {
            return cv::norm(a.center - img_center) < cv::norm(b.center - img_center);
        });

    // === 解算目标空间位置 ===
    solver.solve(target_armor);

    // === 弹道解算 ===
    double bullet_speed = gimbal_state.bullet_speed;
    if (bullet_speed <= 0) bullet_speed = 10.0;

    double dx = target_armor.xyz_in_world.x();
    double dy = target_armor.xyz_in_world.y();
    double dz = target_armor.xyz_in_world.z();

    double distance = std::sqrt(dx * dx + dy * dy);
    tools::Trajectory traj(bullet_speed, distance, dz);

    if (!traj.unsolvable)
    {
        double target_yaw = target_armor.ypd_in_world[0] ;
        double target_pitch = -traj.pitch;

        target_pitch = std::clamp(target_pitch, -20.0 * CV_PI / 180.0, 20.0 * CV_PI / 180.0);

        bool shoot_flag = false;
        int key = cv::waitKey(1);
        if (key == 32) shoot_flag = true;
        else if (key == 'q' || key == 'Q') break;

        gimbal.send(true, shoot_flag, target_yaw, target_pitch);

        // 绘制视觉标记
        cv::circle(img, target_armor.center, 5, cv::Scalar(0, 255, 0), 2);
        cv::putText(img, shoot_flag ? "FIRE!" : "TARGET",
                    target_armor.center + cv::Point2f(10, -10),
                    cv::FONT_HERSHEY_SIMPLEX, 0.6,
                    shoot_flag ? cv::Scalar(0, 0, 255) : cv::Scalar(0, 255, 0), 2);

        // 绘图输出
        nlohmann::json plot_data;
        plot_data["control_mode"] = shoot_flag ? 2 : 1;
        plot_data["target_yaw"] = target_yaw;
        plot_data["target_pitch"] = target_pitch;
        plot_data["armor_x"] = dx;
        plot_data["armor_y"] = dy;
        plot_data["armor_z"] = dz;
        plot_data["fly_time"] = traj.fly_time;
        plot_data["bullet_speed"] = bullet_speed;
        plot_data["detected_armors"] = armors.size();
        plotter.plot(plot_data);
    }

    cv::imshow("Result", img);
    cv::waitKey(1);
}
    return 0;
}