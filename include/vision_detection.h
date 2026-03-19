#ifndef VISION_DETECTION_H
#define VISION_DETECTION_H

#include "mission_header.h"

/************************************************************************
视觉相关函数声明
*************************************************************************/
cv::Mat getColorMask(const cv::Mat& frame, const cv::Scalar& low, const cv::Scalar& high);
bool findGrayRingCenter(const cv::Mat& frame, cv::Point& center, cv::Rect& gray_rect, int& radius) ;

bool findBlackSquareAroundCenter(const cv::Mat& frame, const cv::Point& gray_center, int search_radius,
                                cv::Rect& black_square, float& angle) ;
bool preciseClassify(const cv::Mat& frame, const cv::Rect& center_rect, 
                     std::string& cls_name, float& conf);
bool detectQRCodeAndExtractInfo();
UavDetectResult detectUavTarget();
bool onFrame(float t_yaw,double err_max);
std::string decodeQRCode(const cv::Mat& frame);

#endif