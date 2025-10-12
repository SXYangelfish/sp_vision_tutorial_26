#include "buff_solver.hpp"
#include <opencv2/opencv.hpp>
#include "buff_type.hpp"
#include <fmt/core.h>

namespace auto_buff
{
    // 相机内参
    const cv::Mat Buff_Solver::camera_matrix =
    (cv::Mat_<double>(3, 3) <<  1776.9477196851155 , 0                  , 756.31235265560952, 
                                0                 , 1776.0591253569607 , 566.16539069551641 , 
                                0                 , 0                  , 1                   );
    // 畸变系数
    const cv::Mat Buff_Solver::distort_coeffs =
    (cv::Mat_<double>(1, 5) << -0.08382326954462313, 0.097449270330296239, -0.0012558283068959985, 0.0037372210254148081, 0);

    // 风扇叶3D点坐标
    const std::vector<cv::Point3f> Buff_Solver::object_points = {
        {-0.186f, -0.191f, 0.0f},  
        {0.186f, -0.191f, 0.0f},   
        {0.186f, 0.191f, 0.0f},    
        {-0.186f, 0.191f, 0.0f}    
    };




void Buff_Solver::solvePnP(const FanBlade& fanblade, cv::Mat& rvec, cv::Mat& tvec)
    {
        if (fanblade.points.size() < 4) {
            rvec = cv::Mat::zeros(3, 1, CV_64F);
            tvec = cv::Mat::zeros(3, 1, CV_64F);
            return; 
        }
        
        // 提取风扇叶4个点作为图像坐标系下的点
        std::vector<cv::Point2f> img_points;
        for (size_t i = 0; i < 4 && i < fanblade.points.size(); ++i) {
            img_points.push_back(fanblade.points[i]);
        }
        
        // solvePnP
        cv::solvePnP(object_points, img_points, camera_matrix, distort_coeffs, rvec, tvec);
    }



    cv::Mat Buff_Solver::visualizeResults(const cv::Mat& img, const std::vector<FanBlade>& fanblades)
{
    cv::Mat display_img = img.clone();

    for (const auto& fanblade : fanblades)
    {
        drawFanblade(display_img, fanblade);

        
        cv::Mat rvec, tvec;// solvePnP
        solvePnP(fanblade, rvec, tvec);
        
        drawPoseInfo(display_img, tvec, rvec);

        
        std::vector<cv::Point3f> axis_points = //3D坐标轴
        {
            {0, 0, 0},     
            {0.05, 0, 0},  
            {0, 0.05, 0},   
            {0, 0, 0.05}    
        };
        
        std::vector<cv::Point2f> projected_points;
        cv::projectPoints(axis_points, rvec, tvec, camera_matrix, distort_coeffs, projected_points);
        
        // 坐标轴
        cv::line(display_img, projected_points [0], projected_points [1], cv::Scalar(0, 0, 255), 3);   // X轴 
        cv::line(display_img, projected_points [0], projected_points [2], cv::Scalar(0, 255, 0), 3);   // Y轴 
        cv::line(display_img, projected_points [0], projected_points [3], cv::Scalar(255, 0, 0), 3);   // Z轴 
        
        // 坐标轴标签
        cv::putText(display_img, "X", projected_points  [1], cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 0, 255), 2);
        cv::putText(display_img, "Y", projected_points  [2], cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 0), 2);
        cv::putText(display_img, "Z", projected_points  [3], cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 0, 0), 2);
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
    
    return true;  // 继续
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
    
    // 叶片的颜色和类型名称
    getFanbladeStyle(fanblade.type, color, type_name);
    
    //关键点
    for (size_t i = 0; i < fanblade.points.size(); ++i) {
        cv::circle(display_img, fanblade.points[i], 3, color, -1);
        cv::putText(display_img, std::to_string(i), 
                   cv::Point(fanblade.points[i].x + 5, fanblade.points[i].y - 5),
                   cv::FONT_HERSHEY_SIMPLEX, 0.4, color, 1);
    }
    
    // 叶片轮廓
    if (fanblade.points.size() >= 4) {
        for (size_t i = 0; i < 4; ++i) {
            cv::line(display_img, fanblade.points[i], 
                    fanblade.points[(i + 1) % 4], color, 2);
        }
    }
    
    //中心点
    cv::circle(display_img, fanblade.center, 5, color, -1);
    
    //类型标签
    cv::putText(display_img, type_name, 
               cv::Point(fanblade.center.x + 10, fanblade.center.y),
               cv::FONT_HERSHEY_SIMPLEX, 0.6, color, 2);
}



void Buff_Solver::drawPoseInfo(cv::Mat& display_img, const cv::Mat& tvec, const cv::Mat& rvec)
{
    // 欧拉角
    cv::Mat rotation_matrix;
    cv::Rodrigues(rvec, rotation_matrix);
    
    // pitch、yaw、roll
    double pitch = atan2(-rotation_matrix.at<double>(2, 0), 
                        sqrt(pow(rotation_matrix.at<double>(2, 1), 2) + 
                             pow(rotation_matrix.at<double>(2, 2), 2))) * 180 / CV_PI;
    
    double yaw = atan2(rotation_matrix.at<double>(1, 0), 
                      rotation_matrix.at<double>(0, 0)) * 180 / CV_PI;
    
    double roll = atan2(rotation_matrix.at<double>(2, 1), 
                       rotation_matrix.at<double>(2, 2)) * 180 / CV_PI;
    
    // 位置和姿态信息
    std::string pose_info = fmt::format("Pos: ({:.2f}, {:.2f}, {:.2f})", 
                                       tvec.at<double>(0), 
                                       tvec.at<double>(1), 
                                       tvec.at<double>(2));
    
    std::string angle_info = fmt::format("Angles: P{:.1f} Y{:.1f} R{:.1f}", 
                                        pitch, yaw, roll);
    
    // 在图像左上角显示信息
    cv::putText(display_img, pose_info, cv::Point(20, 30), 
               cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 255, 255), 2);
    cv::putText(display_img, angle_info, cv::Point(20, 60), 
               cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 255, 255), 2);
    
    // 显示距离
    double distance = sqrt(pow(tvec.at<double>(0), 2) + 
                          pow(tvec.at<double>(1), 2) + 
                          pow(tvec.at<double>(2), 2));
    
    std::string dist_info = fmt::format("Distance: {:.2f}m", distance);
    cv::putText(display_img, dist_info, cv::Point(20, 90), 
               cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 255, 255), 2);
}

cv::Point3f Buff_Solver::calculateRotationCenter(const std::vector<FanBlade>& fanblades)
{
    if (fanblades.empty()) {
        return cv::Point3f(0, 0, 0);
    }
    
    std::vector<cv::Point3f> estimated_centers;
    
    for (const auto& blade : fanblades) {
        if (blade.points.size() < 5) {
            continue;
        }
        
        cv::Mat rvec, tvec;
        solvePnP(blade, rvec, tvec);
        
        cv::Mat rotation_matrix;
        cv::Rodrigues(rvec, rotation_matrix);
        
        
        // 物体坐标系中的点
        cv::Point3d center_object(0, 0, 0);
        cv::Point3d tip_object(0, blade_length, 0);
        
        // 将点转换为双精度
        //报错单精度转双精度后问ai解决的
        cv::Mat center_obj_mat = (cv::Mat_<double>(3,1) << center_object.x, center_object.y, center_object.z);
        cv::Mat tip_obj_mat = (cv::Mat_<double>(3,1) << tip_object.x, tip_object.y, tip_object.z);
        
        // 转换到相机坐标系
        cv::Mat center_camera_mat = rotation_matrix * center_obj_mat + tvec;
        cv::Mat tip_camera_mat = rotation_matrix * tip_obj_mat + tvec;
        
        // 转换回Point3f
        cv::Point3f center_camera(
            static_cast<float>(center_camera_mat.at<double>(0)),
            static_cast<float>(center_camera_mat.at<double>(1)),
            static_cast<float>(center_camera_mat.at<double>(2))
        );
        
        cv::Point3f tip_camera(
            static_cast<float>(tip_camera_mat.at<double>(0)),
            static_cast<float>(tip_camera_mat.at<double>(1)),
            static_cast<float>(tip_camera_mat.at<double>(2))
        );
        
        //方向向量
        cv::Point3f blade_direction = tip_camera - center_camera;
        float direction_length = cv::norm(blade_direction);
        
        if (direction_length > 0) {
            blade_direction /= direction_length;
            
            float distance_to_center = 0.15f;
            cv::Point3f rotation_center = center_camera - blade_direction * distance_to_center;
            estimated_centers.push_back(rotation_center);
        }
    }
    
    if (estimated_centers.empty()) {
        return cv::Point3f(0, 0, 0);
    }
    
    cv::Point3f final_center(0, 0, 0);
    for (const auto& center : estimated_centers) {
        final_center += center;
    }
    final_center /= static_cast<float>(estimated_centers.size());
    
    return final_center;
}


}//auto_buff