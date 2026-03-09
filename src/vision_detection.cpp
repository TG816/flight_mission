#include "vision_detection.h"
#include "flight_control.h"
#include "mission_header.h"

/************************************************************************
视觉相关函数实现
*************************************************************************/
//获取起飞时的颜色
void getColor(const cv::Mat &frame){
     cv::Mat hsv, mask, result;
    cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);
    bool flag_findColor=false;

    for (const auto &range : color_ranges)
    {
        cv::Mat temp_mask;
        cv::inRange(hsv, range.lower, range.upper, temp_mask);

  
        // 形态学操作去除噪声
        cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(15, 15));
        cv::morphologyEx(temp_mask, temp_mask, cv::MORPH_OPEN, kernel);
        cv::morphologyEx(temp_mask, temp_mask, cv::MORPH_CLOSE, kernel);

        // 查找轮廓
        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(temp_mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
     for (const auto &contour : contours)
        {
            double area = cv::contourArea(contour);
            if (area < 4500) continue; // 最小面积阈值
            else{
                target_color=range.name;flag_findColor=true;break;
            }
        }

        if(flag_findColor)break;
    }
}

bool detectColorFloor(const cv::Mat &frame, cv::Point2f &center, std::string &color_name)
{
    cv::Mat hsv, mask, result;
    cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);

    double max_area = 0;
    cv::Point2f best_center;
    std::string best_color = "";

    // 遍历所有颜色范围
    for (const auto &range : color_ranges)
    {
        cv::Mat temp_mask;
        cv::inRange(hsv, range.lower, range.upper, temp_mask);

        // 如果只检测特定颜色
        if (!target_color.empty() && range.name != target_color)
        {
            continue;
        }

        // 形态学操作去除噪声
        cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(15, 15));
        cv::morphologyEx(temp_mask, temp_mask, cv::MORPH_OPEN, kernel);
        cv::morphologyEx(temp_mask, temp_mask, cv::MORPH_CLOSE, kernel);

        // 查找轮廓
        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(temp_mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        for (const auto &contour : contours)
        {
            double area = cv::contourArea(contour);
            if (area > 4500)
            { // 最小面积阈值
                cv::Moments m = cv::moments(contour);
                if (m.m00 != 0)
                {
                    cv::Point2f c(m.m10 / m.m00, m.m01 / m.m00);

                    // 检查是否为方形（1x1地板应该是方形）
                    cv::Rect rect = cv::boundingRect(contour);
                    double aspect_ratio = (double)rect.width / rect.height;

                    if (aspect_ratio > 0.80 && aspect_ratio < 1.20)
                    {
                        if (area > max_area)
                        {
                            max_area = area;
                            best_center = c;
                            best_color = range.name;
                        }
                    }
                }
            }
        }
    }

    if (max_area > 4500)
    {
        center = best_center;
        color_name = best_color;
        return true;
    }

    return false;
}

void detect_face(const cv::Mat& frame){
    if(frame.empty()){
        return;
    }
    //预处理
    cv::Mat gray;
    cv::cvtColor(frame,gray,cv::COLOR_BGR2GRAY);
    cv::equalizeHist(gray,gray);
    //检测人脸
    vector<cv::Rect> faces;
     face_cascade.detectMultiScale(gray, faces, 1.1, 4, 0, cv::Size(30, 30));
     if(!faces.empty()){
        size_t biggest_idx=0;
        size_t max_area=0;
      for(size_t i=0;i<faces.size();i++){
        size_t area=faces[i].width*faces[i].height;
        if(area>max_area){biggest_idx=i;max_area=area;}
      }  
      face_rect=faces[biggest_idx];face_detected=true;
      float center_x=face_rect.x+face_rect.width/2.0;
      float center_y=face_rect.y+face_rect.height/2.0;
      face_world=change_to_world(center_x,center_y);
      ROS_INFO("检测到人脸，中心世界坐标：x:%f,y:%f",face_world.x,face_world.y);
     }
     else{
        ROS_WARN("NOT DETECTED FACE YET");
     }
     
}

//绕半径为a的粗略圆形飞行一周，同时找目标
void cruise_finding(float center_x,float center_y,float z,float target_yaw,float error_max,float a){
    int face_detect_count=0;//每5帧检测一次
    float angle_cruise=0;float d_angle=M_PI/3.0;
    while(!face_detected&&angle_cruise<d_angle*5.5){
        while(!mission_pos_cruise(center_x+a*cos(angle_cruise),center_y+a*sin(angle_cruise),z,target_yaw,error_max)){
            if(face_detect_count<4){face_detect_count++;}
            else{detect_face(current_frame);face_detect_count=0;}
        }
        angle_cruise+=d_angle;
    }
}

bool FindColorToMove()
{

    // 检测颜色地板
    cv::Point2f center;
    std::string color;
    bool detected = detectColorFloor(current_frame, center, color);

    if (detected)
    {
        color_center = center;
        detected_color = color;
        color_detected = true;

        // 绘制检测结果
        cv::circle(current_frame, center, 10, cv::Scalar(0, 255, 0), -1);
        cv::putText(current_frame, color, cv::Point(center.x - 30, center.y - 30),
                    cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 255, 0), 2);

        ROS_INFO_THROTTLE(0.5, "检测到 %s 地板，中心位置: (%.1f, %.1f)",
                          color.c_str(), center.x, center.y);
    }
    else
    {
        color_detected = false;
    }
    cr_world = change_to_world(color_center.x, color_center.y);
    if (color_detected)
    {
        fly(-0.05 * direction);
        ROS_WARN("find targets!");
        return true;
    }
    else
    {
        if (local_pos.pose.pose.position.x < -0.2)
            direction = 1.0;
        if (local_pos.pose.pose.position.x > 4.1)
            direction = -1.0;
        fly(0.3 * direction);
        ROS_INFO("still finding!");
        return false;
    }
}

string detectBeginColor()
{
    cv::Mat hsv;
    cv::cvtColor(current_frame, hsv, cv::COLOR_BGR2HSV);

    double max_area = 0;
    std::string begin_color = "";

    // 遍历所有颜色范围
    for (const auto &range : color_ranges)
    {
        cv::Mat temp_mask;
        cv::inRange(hsv, range.lower, range.upper, temp_mask);

        // 形态学操作去除噪声
        cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(15, 15));
        cv::morphologyEx(temp_mask, temp_mask, cv::MORPH_OPEN, kernel);
        cv::morphologyEx(temp_mask, temp_mask, cv::MORPH_CLOSE, kernel);

        // 查找轮廓
        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(temp_mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        for (const auto &contour : contours)
        {
            double area = cv::contourArea(contour);
            if (area > 4500)
            { // 最小面积阈值

                if (area > max_area)
                {
                    max_area = area;
                    begin_color = range.name;
                }
            }
        }
    }
    return begin_color;
}