#ifndef RING_DETECTION_H
#define RING_DETECTION_H

#include "mission_header.h"

/************************************************************************
穿环相关函数声明
*************************************************************************/
bool cross_ring(double x, double y, double z, double t_yaw, double err_max);

#endif