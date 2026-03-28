#ifndef OBSTACLE_AVOIDANCE_H
#define OBSTACLE_AVOIDANCE_H

#include "mission_header.h"

/************************************************************************
避障相关函数声明
*************************************************************************/
void find_obstacal();
bool collision_avoidance_mission(float target_x, float target_y, float target_z, float target_yaw, float err_max);
GridPoint out_of_danger(const GridPoint &now, int mode = 1);

#endif