#ifndef AUTO_BUFF__SOLVER_HPP
#define AUTO_BUFF__SOLVER_HPP

#include <opencv2/opencv.hpp>
#include <vector>
#include "buff_detector.hpp"
#include "buff_type.hpp"

namespace auto_buff
{
class Buff_Solver
{
public:
    void solvePnP(const FanBlade& fanblade, cv::Mat& rvec, cv::Mat& tvec);
    cv::Mat visualizeResults(const cv::Mat &img, const std::vector<FanBlade> &fanblades);
    bool displayAndWait(const cv::Mat &display_img);
    cv::Point3f calculateRotationCenter(const std::vector<FanBlade>& fanblades);

private:
    void getFanbladeStyle(FanBlade_type type, cv::Scalar &color, std::string &type_name);
    void drawFanblade(cv::Mat& display_img, const FanBlade& fanblade);
    void drawPoseInfo(cv::Mat& display_img, const cv::Mat& tvec, const cv::Mat& rvec);

    static const cv::Mat camera_matrix;
    static const cv::Mat distort_coeffs;
    static const std::vector<cv::Point3f> object_points;
    static constexpr float blade_length = 0.712f; //长度
};
};  // namespace auto_buff
#endif  // SOLVER_HPP