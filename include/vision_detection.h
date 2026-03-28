#ifndef VISION_DETECTION_H
#define VISION_DETECTION_H

#include "mission_header.h"

/************************************************************************
视觉相关函数声明
*************************************************************************/
cv::Mat getColorMask(const cv::Mat& frame, const cv::Scalar& low, const cv::Scalar& high);
bool findBlackSquare(const cv::Mat& frame, cv::Point& black_center, cv::Rect& black_rect, float& angle);

bool preciseClassify(const cv::Mat& frame, const cv::Rect& center_rect, 
                     std::string& cls_name, float& conf);
bool detectQRCodeAndExtractInfo();
UavDetectResult detectUavTarget();
bool onFrame(float t_yaw,double err_max);
std::string decodeQRCode(const cv::Mat& frame);
bool detectBlackSquareAndThrow(float throw_yaw, double err_max);
#endif