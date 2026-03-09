#ifndef FLIGHT_CONTROL_H
#define FLIGHT_CONTROL_H

#include "mission_header.h"

/************************************************************************
飞控控制函数声明
*************************************************************************/
bool mission_pos_cruise(float x, float y, float z, float target_yaw, float error_max);
bool precision_land();
bool move_in_drone_coordinate(double x, double y, double z, double target_yaw, double err_max, int mode = 0);
void fly(float v);

#endif