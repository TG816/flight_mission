#ifndef VISION_DETECTION_H
#define VISION_DETECTION_H

#include "mission_header.h"

/************************************************************************
视觉相关函数声明
*************************************************************************/
RecognizeResult recognizeTarget(const cv::Mat& frame, const Config& cfg);
RoughTargetInfo roughDetectTarget(const cv::Mat& frame);
cv::Rect getCenterImageRect(const RoughTargetInfo& rough_info);
bool preciseClassify(const cv::Mat& frame, const cv::Rect& center_rect, const Config& cfg,
                     std::string& cls_name, float& conf);
bool detectQRCodeAndExtractInfo();

#endif