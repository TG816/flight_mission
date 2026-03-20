#include "flight_control.h"
#include "mission_header.h"

/************************************************************************
函数 3: 无人机位置控制
控制无人机飞向（x, y, z）位置，target_yaw为目标航向角，error_max为允许的误差范围
进入函数后开始控制无人机飞向目标点，返回值为bool型，表示是否到达目标点
*************************************************************************/
float mission_pos_cruise_last_position_x = 0;
float mission_pos_cruise_last_position_y = 0;
bool mission_pos_cruise_flag = false;
bool mission_pos_cruise(float x, float y, float z, float target_yaw, float error_max)
{
    if (mission_pos_cruise_flag == false)
    {
        mission_pos_cruise_last_position_x = local_pos.pose.pose.position.x;
        mission_pos_cruise_last_position_y = local_pos.pose.pose.position.y;
        mission_pos_cruise_flag = true;
    }
    setpoint_raw.type_mask = /*1 + 2 + 4 */ +8 + 16 + 32 + 64 + 128 + 256 + 512 /*+ 1024 */ + 2048;
    setpoint_raw.coordinate_frame = 1;
    setpoint_raw.position.x = x + init_position_x_take_off;
    setpoint_raw.position.y = y + init_position_y_take_off;
    setpoint_raw.position.z = z + init_position_z_take_off;
    setpoint_raw.yaw = target_yaw / 180.0 * M_PI;
    ROS_INFO("now (%.2f,%.2f,%.2f,%.2f) to ( %.2f, %.2f, %.2f, %.2f)", local_pos.pose.pose.position.x, local_pos.pose.pose.position.y, local_pos.pose.pose.position.z, yaw * 180.0 / M_PI, x + init_position_x_take_off, y + init_position_y_take_off, z + init_position_z_take_off, target_yaw);
  
    if (fabs(local_pos.pose.pose.position.x - x - init_position_x_take_off) < error_max && fabs(local_pos.pose.pose.position.y - y - init_position_y_take_off) < error_max && fabs(local_pos.pose.pose.position.z - z - init_position_z_take_off) < 0.05 && fabs(yaw - target_yaw / 180 * M_PI) < 0.1)
    {
        ROS_INFO("到达目标点，巡航点任务完成");
        mission_pos_cruise_flag = false;
        return true;
    }
    return false;
}

/************************************************************************
函数 4:降落
无人机当前位置作为降落点，缓慢下降至地面
返回值为bool型，表示是否降落完成
*************************************************************************/
float precision_land_init_position_x = 0;
float precision_land_init_position_y = 0;
bool precision_land_init_position_flag = false;
ros::Time precision_land_last_time;
bool precision_land()
{
    if (!precision_land_init_position_flag)
    {
        precision_land_init_position_x = local_pos.pose.pose.position.x;
        precision_land_init_position_y = local_pos.pose.pose.position.y;
        precision_land_last_time = ros::Time::now();
        precision_land_init_position_flag = true;
    }
    setpoint_raw.position.x = precision_land_init_position_x;
    setpoint_raw.position.y = precision_land_init_position_y;
    setpoint_raw.position.z = -0.15;
    setpoint_raw.type_mask = /*1 + 2 + 4 + 8 + 16 + 32*/ +64 + 128 + 256 + 512 /*+ 1024 + 2048*/;
    setpoint_raw.coordinate_frame = 1;
    if (ros::Time::now() - precision_land_last_time > ros::Duration(5.0))
    {
        ROS_INFO("Precision landing complete.");
        precision_land_init_position_flag = false; // Reset for next landing
        return true;
    }
    return false;
}

bool move_in_drone_coordinate(double x, double y, double z, double target_yaw, double err_max, int mode)
{
    if (!isinit)
        init_location(init_x, init_y, init_z, init_yaw);
    rotation(-yaw, x, y);
    // ROS_ERROR("(%f,%f,%f)", z, init_z, local_pos.pose.pose.position.z);
    if (mode)
    {
        return mission_pos_cruise(x + init_x, y + init_y, ALTITUDE, target_yaw + init_yaw, err_max);
    }
    else
    {
        ROS_WARN("yaw,target_yaw,init_yaw:(%f,%f,%f)", yaw, target_yaw, init_yaw);
        return mission_pos_cruise(x + init_x, y + init_y, z + init_z, target_yaw + init_yaw, err_max);
    }
}

// void fly(float v)
// {
//     setpoint_raw.type_mask = 1 + 2 + /*4 + 8 + 16 + 32*/ +64 + 128 + 256 + 512 /*+ 1024 + 2048*/;
//     setpoint_raw.coordinate_frame = 1;
//     setpoint_raw.position.x = local_pos.pose.pose.position.x;
//     setpoint_raw.position.y = local_pos.pose.pose.position.y;
//     setpoint_raw.position.z = 1.5f;

//     // 速度字段（原有逻辑保留，已补全）
//     setpoint_raw.velocity.x = v;
//     setpoint_raw.velocity.y = 2*(balance_y - local_pos.pose.pose.position.y);
//     setpoint_raw.velocity.z = 0.0f; // 保持高度

//     // 必选：补全加速度字段（飞控要求，设为0即可）
//     setpoint_raw.acceleration_or_force.x = 0.0f;
//     setpoint_raw.acceleration_or_force.y = 0.0f;
//     setpoint_raw.acceleration_or_force.z = 0.0f;

//     // 必选：补全姿态字段（飞控要求，用当前yaw）
//     setpoint_raw.yaw = yaw;       // 偏航角（rad，需从飞控订阅获取）
//     setpoint_raw.yaw_rate = 0.0f; // 偏航角速度（保持当前朝向）
// }



