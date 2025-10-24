#include <chrono>
#include <opencv2/opencv.hpp>

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

while (!exiter.exit()) {
    // Your code start
    int frame_count = 0;
    auto last_time = std::chrono::steady_clock::now();

    const double LOW_MIN = 3.0;  
    const double LOW_MAX = 5.0;
    const double MID_MIN = 5.0;
    const double MID_MAX = 7.0;
    const double HIGH_MIN = 8.0;
    const double HIGH_MAX = 10.0;

const double LOW_TOL = 0.6;  
const double MID_TOL = 1.0;
const double HIGH_TOL = 1.5;

const int MAX_SHOTS_PER_STAGE = 10; 
enum class Stage { LOW = 0, MID = 1, HIGH = 2, DONE = 3 };
Stage stage = Stage::LOW;
int shots[3] = {0, 0, 0};

// EKF 
auto_aim::Target ekf_target;
bool have_target = false;

// 记录上次发射时间ms
int fire_cooldown_ms = 200;
auto last_fire_time = std::chrono::steady_clock::now() - 1s;

// 选择置信度最高的装甲
auto choose_best_armor = [](const std::list<auto_aim::Armor> & list) -> std::optional<auto_aim::Armor> {
  std::optional<auto_aim::Armor> best;
  double best_conf = -1.0;
  for (const auto & a : list) {
    if (a.confidence > best_conf) {
      best_conf = a.confidence;
      best = a;
    }
  }
  return best;
};

// 循环开始
while (!exiter.exit()) {
  // 读取图像和时间戳
  camera.read(img, t);
  frame_count++;

  //YOLO 检测装甲
  std::list<auto_aim::Armor> detected = yolo.detect(img, frame_count);

  // 对每个检测到的装甲求解位姿
  for (auto & a : detected) {
    try {
      solver.solve(a);
    } catch (...) {
      // 若 solver 失败，跳过该装甲
      continue;
    }
  }

  // 选置信度最高
  std::optional<auto_aim::Armor> observed = std::nullopt;
  if (!detected.empty()) observed = choose_best_armor(detected);

  //EKF 
  // 初始化 EKF
  if (observed.has_value()) {
    const auto & obs = observed.value();
    if (!have_target) {
      Eigen::VectorXd P0 = Eigen::VectorXd::Constant(11, 1e-2);
      ekf_target = auto_aim::Target(obs, t, P0, 0.2, 4);
      have_target = true;
    } else {
      ekf_target.update(obs);
      ekf_target.predict(t);
    }
  } else {
    if (have_target) {
      ekf_target.predict(t);
    }
  }

  // 从 EKF 中读取拟合的 yaw_rate
  double fitted_yaw_rate = 0.0;
  bool have_fitted = false;
  if (have_target) {
    try {
      Eigen::VectorXd x = ekf_target.ekf_x();
      if (x.size() >= 8) {
        fitted_yaw_rate = x(7);
        have_fitted = true;
      }
    } catch (...) {
      have_fitted = false;
    }
  }

  // 计算瞄准角
  bool do_control = false;
  float cmd_yaw = 0.0f;
  float cmd_pitch = 0.0f;
  if (observed.has_value()) {
    try {
      if (observed->ypr_in_gimbal.size() >= 2) {
        cmd_yaw = static_cast<float>(observed->ypr_in_gimbal(0));
        cmd_pitch = static_cast<float>(observed->ypr_in_gimbal(1));
        do_control = true;
      } else if (observed->ypr_in_world.size() >= 2) {
        cmd_yaw = static_cast<float>(observed->ypr_in_world(0));
        cmd_pitch = static_cast<float>(observed->ypr_in_world(1));
        do_control = true;
      }
    } catch (...) {
      do_control = false;
    }
  }

  // EKF 拟合到角速度并位于当前stage区间且未达到最大发数，则单发
  bool want_fire = false;
  if (do_control && have_fitted && stage != Stage::DONE) {
    double abs_rate = std::abs(fitted_yaw_rate);
    bool in_stage = false;
    if (stage == Stage::LOW) in_stage = (abs_rate >= LOW_MIN && abs_rate <= LOW_MAX);
    else if (stage == Stage::MID) in_stage = (abs_rate >= MID_MIN && abs_rate <= MID_MAX);
    else if (stage == Stage::HIGH) in_stage = (abs_rate >= HIGH_MIN && abs_rate <= HIGH_MAX);

    int idx = static_cast<int>(stage);
    if (in_stage && idx >= 0 && idx < 3 && shots[idx] < MAX_SHOTS_PER_STAGE) {
      auto nowt = std::chrono::steady_clock::now();
      int ms_since_last = std::chrono::duration_cast<std::chrono::milliseconds>(nowt - last_fire_time).count();
      if (ms_since_last >= fire_cooldown_ms) {
        want_fire = true;
        last_fire_time = nowt;
        shots[idx] += 1;
      }
    }
  }

  // 云台指令
  if (do_control) {
    try {
      gimbal.send(true, want_fire, cmd_yaw, cmd_pitch);
    } catch (...) {
      // 若发送失败，确保不触发发射
      try {
        gimbal.send(false, false, 0.0f, 0.0f);
      } catch (...) {
      }
    }
  } else {
    // 没有可瞄准目标，保持不控制且不开火
    try {
      gimbal.send(false, false, 0.0f, 0.0f);
    } catch (...) {
    }
  }

  // 当档位发满 MAX_SHOTS_PER_STAGE推进到下一档
  if (stage != Stage::DONE) {
    int idx = static_cast<int>(stage);
    if (shots[idx] >= MAX_SHOTS_PER_STAGE) {
      if (stage == Stage::LOW) stage = Stage::MID;
      else if (stage == Stage::MID) stage = Stage::HIGH;
      else if (stage == Stage::HIGH) stage = Stage::DONE;
    }
  }

  // lotter
  try {
    nlohmann::json plot_data;
    plot_data["control_mode"]    = do_control ? 1 : 0;
    plot_data["cmd_yaw"]         = cmd_yaw;                 // 发送给云台的yaw
    plot_data["cmd_pitch"]       = cmd_pitch;               // 发送给云台的pitch
    plot_data["fire"]            = want_fire ? 1 : 0;       // 本帧是否触发单发
    plot_data["detected_armors"] = static_cast<int>(detected.size());
    plot_data["fitted_yaw_rate"] = have_fitted ? fitted_yaw_rate : 0.0;
    plot_data["have_target"]     = have_target ? 1 : 0;

    if (observed.has_value()) {
      plot_data["target_yaw"]   = observed->ypr_in_world[0];
      plot_data["target_pitch"] = observed->ypr_in_world[1];
      plot_data["armor_x"]      = observed->xyz_in_world[0];
      plot_data["armor_y"]      = observed->xyz_in_world[1];
      plot_data["armor_z"]      = observed->xyz_in_world[2];
    } else if (have_target) {
      auto xyza_list = ekf_target.armor_xyza_list();
      if (!xyza_list.empty()) {
        plot_data["armor_x"] = xyza_list[0][0];
        plot_data["armor_y"] = xyza_list[0][1];
        plot_data["armor_z"] = xyza_list[0][2];
      } else {
        plot_data["armor_x"] = 0.0;
        plot_data["armor_y"] = 0.0;
        plot_data["armor_z"] = 0.0;
      }
      plot_data["target_yaw"] = ekf_target.ekf_x()[6];
      plot_data["target_pitch"] = 0.0;
    } else {
      plot_data["target_yaw"] = 0.0;
      plot_data["target_pitch"] = 0.0;
      plot_data["armor_x"] = 0.0;
      plot_data["armor_y"] = 0.0;
      plot_data["armor_z"] = 0.0;
    }

    plot_data["stage"]      = static_cast<int>(stage);
    plot_data["shots_low"]  = shots[0];
    plot_data["shots_mid"]  = shots[1];
    plot_data["shots_high"] = shots[2];

    plot_data["low_tol"]  = LOW_TOL;
    plot_data["mid_tol"]  = MID_TOL;
    plot_data["high_tol"] = HIGH_TOL;

    plotter.plot(plot_data);
  } catch (...) {
  }

  
} 
    // Your code end
  

  return 0;
}
}