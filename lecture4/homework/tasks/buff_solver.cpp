#include "buff_solver.hpp"
#include <opencv2/opencv.hpp>
#include "buff_type.hpp"


namespace auto_buff
{   //相机内参
    const cv::Mat Buff_Solver::camera_matrix =
    (cv::Mat_<double>(3, 3) <<  1286.307063384126 , 0                  , 645.34450819155256, 
                                0                 , 1288.1400736562441 , 483.6163720308021 , 
                                0                 , 0                  , 1                   );
    // 畸变系数
    const cv::Mat Buff_Solver::distort_coeffs =
    (cv::Mat_<double>(1, 5) << -0.47562935060124745, 0.21831745829617311, 0.0004957613589406044, -0.00034617769548693592, 0);




    cv::Mat Buff_Solver::visualizeResults(const cv::Mat& img, const std::vector<FanBlade>& fanblades)
{
    cv::Mat display_img = img.clone();

    for (const auto& fanblade : fanblades)
    {
        drawFanblade(display_img, fanblade);

        if(fanblade.points.size() >= 5) {
            cv::Point2f r_center = calculateRotationCenter(fanblade);
            cv::circle(display_img, r_center, 8, cv::Scalar(255, 0, 255), -1);  // 紫色圆点
            cv::putText(display_img, "R_CENTER", 
                       cv::Point(r_center.x + 10, r_center.y - 10),
                       cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 0, 255), 2);
        }
    
    }
    
    cv::resize(display_img, display_img, {}, 0.8, 0.8);
    
    return display_img;
}

bool Buff_Solver::displayAndWait(const cv::Mat& display_img)
{

    cv::imshow("Detection Results", display_img);
    

    if (cv::waitKey(30) == 27) {
        return false;  // ESC键退出
    }
    
    return true;  // 继续运行
}

void Buff_Solver::getFanbladeStyle(FanBlade_type type, cv::Scalar& color, std::string& type_name)
{
    switch (type) {
        case _target:
            color = cv::Scalar(0, 255, 0);   
            type_name = "_target";
            break;
        case _light:
            color = cv::Scalar(0, 255, 255); 
            type_name = "_light";
            break;
        case _unlight:
            color = cv::Scalar(0, 0, 255);  
            type_name = "_unlight";
            break;
        default:
            color = cv::Scalar(255, 255, 255); 
            type_name = "unknown";
            break;
    }
}

void Buff_Solver::drawFanblade(cv::Mat& display_img, const FanBlade& fanblade)
{
    cv::Scalar color;
    std::string type_name;
    
    // 获取叶片的颜色和类型名称
    getFanbladeStyle(fanblade.type, color, type_name);
    
    // 绘制关键点
    for (size_t i = 0; i < fanblade.points.size(); ++i) {
        cv::circle(display_img, fanblade.points[i], 3, color, -1);
        cv::putText(display_img, std::to_string(i), 
                   cv::Point(fanblade.points[i].x + 5, fanblade.points[i].y - 5),
                   cv::FONT_HERSHEY_SIMPLEX, 0.5, color, 1);
    }
    
    // 绘制中心点
    cv::circle(display_img, fanblade.center, 5, color, -1);
    cv::putText(display_img, "CENTER", 
               cv::Point(fanblade.center.x + 10, fanblade.center.y - 10),
               cv::FONT_HERSHEY_SIMPLEX, 0.5, color, 1);
    
    // 绘制类型标签
    cv::putText(display_img, type_name, 
               cv::Point(fanblade.center.x - 20, fanblade.center.y - 20),
               cv::FONT_HERSHEY_SIMPLEX, 0.7, color, 2);
}

cv::Point2f Buff_Solver::calculateRotationCenter(const FanBlade& fanblade)
{
    if (fanblade.points.size() < 5) {
        return cv::Point2f(0, 0); 
    }
    
    cv::Point2f center = fanblade.points  [4]; 
    cv::Point2f blade_tip = fanblade.points [0];
    
    // 计算旋转符的方向向量
    cv::Point2f direction = center - blade_tip;
    

    double scale_factor = 5.5;
    cv::Point2f r_center = center + direction * scale_factor;
    
    return r_center;
}


void Buff_Solver::solvePnP(){

}

}  // namespace auto_buff
