#ifndef MISSION_CALLBACKS_H
#define MISSION_CALLBACKS_H

#include "mission_header.h"

/************************************************************************
回调函数声明
*************************************************************************/
void state_cb(const mavros_msgs::State::ConstPtr &msg);
void local_pos_cb(const nav_msgs::Odometry::ConstPtr &msg);
void livox_custom_cb(const livox_ros_driver::CustomMsg::ConstPtr &livox_msg);
void image_cb(const sensor_msgs::ImageConstPtr &msg);
void ego_sub_cb(const quadrotor_msgs::PositionCommand::ConstPtr &msg);
void rec_traj_cb(const std_msgs::Bool::ConstPtr &msg);


#endif