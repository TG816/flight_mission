#ifndef VISION_DETECTION_H
#define VISION_DETECTION_H

#include "mission_header.h"

/************************************************************************
视觉相关函数声明
*************************************************************************/
void getColor(const cv::Mat &frame);
bool detectColorFloor(const cv::Mat &frame, cv::Point2f &center, std::string &color_name);
void detect_face(const cv::Mat& frame);
void cruise_finding(float center_x, float center_y, float z, float target_yaw, float error_max, float a);
bool FindColorToMove();
string detectBeginColor();

#endif