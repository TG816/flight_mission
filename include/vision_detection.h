#ifndef VISION_DETECTION_H
#define VISION_DETECTION_H

#include "mission_header.h"

/************************************************************************
视觉相关函数声明
*************************************************************************/
void detect_QR_num_letter(const cv::Mat &frame);
void detect_circular(const cv::Mat &frame);
bool cruise_finding_circular(float center_x,float center_y,float z,float target_yaw,float error_max,float a);
bool detect_specific_char(const cv::Mat &frame ,const string& target_char,cv::Point2f &center);
bool detect_QR(const cv::Mat &frame,cv::Point2f &center);
bool move_to_target(const cv::Mat &frame,float target_yaw);
bool cruise_finding(float a, float b, float c, float d, float e, float f);
bool detect_aim(const cv::Mat &frame,int& idx);
#endif