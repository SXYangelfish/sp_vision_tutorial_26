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
    void solvePnP();
    cv::Mat visualizeResults(const cv::Mat &img, const std::vector<FanBlade> &fanblades);
    bool displayAndWait(const cv::Mat &display_img);

    cv::Point2f calculateRotationCenter(const FanBlade & fanblade);

private:
    void getFanbladeStyle(FanBlade_type type, cv::Scalar &color, std::string &type_name);
    void drawFanblade(cv::Mat& display_img, const FanBlade& fanblade);

    static const cv::Mat camera_matrix;
    static const cv::Mat distort_coeffs;
};
}  // namespace auto_buff
#endif  // SOLVER_HPP