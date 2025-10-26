#include <chrono>
#include <opencv2/opencv.hpp>
#include <yaml-cpp/yaml.h>

#include "io/camera.hpp"
#include "io/gimbal/gimbal.hpp"
#include "tasks/auto_aim/solver.hpp"
#include "tasks/auto_aim/yolo.hpp"
#include "tasks/auto_aim/target.hpp"
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

    tools::Exiter exiter;
    tools::Plotter plotter;
    io::Camera camera(config_path);
    io::Gimbal gimbal(config_path);
    auto_aim::YOLO yolo(config_path, true);
    auto_aim::Solver solver(config_path);

    cv::Mat img;
    Eigen::Quaterniond q;
    std::chrono::steady_clock::time_point t;

    // 配置偏移
    auto yaml = YAML::LoadFile(config_path);
    double yaw_offset = yaml["yaw_offset"].as<double>() * CV_PI / 180.0;
    double pitch_offset = yaml["pitch_offset"].as<double>() * CV_PI / 180.0;

    // EKF Target管理
    std::shared_ptr<auto_aim::Target> current_target = nullptr;


    while(!exiter.exit())
    {
        auto gimbal_state = gimbal.state();
        camera.read(img, t);
        q = gimbal.q(t);
        solver.set_R_gimbal2world(q);

        auto armors = yolo.detect(img);
        if(armors.empty())
        {
            cv::imshow("Result", img);
            cv::waitKey(1);
            continue;
        }

     

        // 选择最靠近中心的目标
        cv::Point2f img_center(img.cols/2.0f, img.rows/2.0f);
        auto target_armor = *std::min_element(
            armors.begin(), armors.end(),
            [img_center](const auto &a, const auto &b){
                return cv::norm(a.center - img_center) < cv::norm(b.center - img_center);
            });

        // Solver计算空间位置
        solver.solve(target_armor);

        double center_angle = std::atan2(target_armor.xyz_in_world[1], target_armor.xyz_in_world[0]);
        double yaw_rate = -current_target->ekf_x()[7] *7.5;
 


        // EKF Target
        if(!current_target)
        {
            Eigen::VectorXd P0 = Eigen::VectorXd::Ones(11) * 0.01;
            current_target = std::make_shared<auto_aim::Target>(target_armor, t, P0, 0.1, 4);
        }
        current_target->update(target_armor);

        double bullet_speed = gimbal_state.bullet_speed;
        if(bullet_speed <= 0) bullet_speed = 10.0;

        // 弹道飞行时间估计
        double distance = target_armor.ypd_in_world[2];
        tools::Trajectory traj(bullet_speed, distance, target_armor.xyz_in_world.z());

        // 使用EKF预测目标在子弹飞行时间后的未来位置
        current_target->predict(traj.fly_time);

        // 弹道重力补偿
        double dx = target_armor.xyz_in_world[0];
        double dy = target_armor.xyz_in_world[1];
        double dz = target_armor.xyz_in_world[2];
        tools::Trajectory traj_future(bullet_speed, std::sqrt(dx*dx+dy*dy), dz);.5;
   


        if(!traj_future.unsolvable)
        {
            double target_yaw = std::atan2(dy, dx);
            double target_pitch = -traj_future.pitch;

            // 限制pitch

            
            target_pitch = std::clamp(target_pitch, -20.0*CV_PI/180.0, 20.0*CV_PI/180.0);

            // 自动开火条件：yaw/pitch误差小于阈值
            bool target_converged = current_target->convergened();
            double yaw_error = std::abs(tools::limit_rad(target_yaw - gimbal_state.yaw));
            double pitch_error = std::abs(target_pitch - gimbal_state.pitch);
            bool shoot_flag = target_converged && (yaw_error < 1.0*CV_PI/180.0) && (pitch_error < 1.0*CV_PI/180.0);

            gimbal.send(true, shoot_flag, target_yaw , target_pitch );

            // 绘制视觉标记
            cv::circle(img, target_armor.center, 5, cv::Scalar(0,255,0), 2);
            cv::putText(img, shoot_flag?"FIRE!":"TARGET",
                        target_armor.center + cv::Point2f(10,-10),
                        cv::FONT_HERSHEY_SIMPLEX, 0.6,
                        shoot_flag ? cv::Scalar(0,0,255) : cv::Scalar(0,255,0), 2);

            // 绘图输出
            nlohmann::json plot_data;
            plot_data["control_mode"] = shoot_flag?2:1;
            plot_data["target_yaw"] = target_yaw;
            plot_data["target_pitch"] = target_pitch;
            plot_data["armor_x"] = dx;
            plot_data["armor_y"] = dy;
            plot_data["armor_z"] = dz;
            plot_data["center_yaw_rate"] = yaw_rate;
            plot_data["fly_time"] = traj_future.fly_time;
            plot_data["bullet_speed"] = bullet_speed;
            plot_data["detected_armors"] = armors.size();
            plotter.plot(plot_data);
        }

        cv::imshow("Result", img);
        cv::waitKey(1);
        int key = cv::waitKey(1);
        if (key == 'q' || key == 'Q') {
            break;
    }

    }

    return 0;
}
