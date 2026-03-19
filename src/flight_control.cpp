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


// 转圈巡航函数：counts=转圈数，times=  最大耗时（秒）
bool Circle_around(int counts, float times, double err_max)
{
    static int now_counts = 0;                        // 已完成转圈数
    static int mission = 1;                           // 当前巡航阶段（1-4为四个点，5为完成1圈）
    static ros::Time total_start = ros::Time::now();  // 总任务计时起点（控制总耗时）
    static ros::Time single_circle_start = ros::Time::now(); // 单圈计时起点（原last_request，解决命名冲突）
    // 总循环条件：
    // 1. 未完成指定圈数  2. ROS节点正常  3. 总耗时 ≤ 设定的总时间times
    while (now_counts < counts && ros::ok() && 
        (ros::Time::now() - total_start).toSec() <= times)
    {
        // 1. 通用日志：总耗时 + 单圈耗时 + 转圈进度
        ros::Duration total_dur = ros::Time::now() - total_start;
        ros::Duration single_dur = ros::Time::now() - single_circle_start;
        ROS_INFO("总耗时: %.2f s (上限%.2f s) | 单圈耗时: %.2f s | 已转%d圈, 正在转第%d圈",
                total_dur.toSec(), times, single_dur.toSec(), now_counts, now_counts + 1);
        // -------------------------- 可选保留：单圈超时提醒（非终止） --------------------------
        // 若需保留单圈超时判断，仅打印警告不终止，取消下面注释即可
        // if (single_dur.toSec() > (times / counts)) { // 单圈理论最大耗时=总时间/总圈数
        //     ROS_WARN("第%d圈单圈耗时超限（已%.2f s），建议检查巡航速度", now_counts + 1, single_dur.toSec());
        // }
        // 2. 分阶段巡航（四个点完成1圈）
        switch (mission)
        {
        case 1: // 第一点：(4.9, 0.6)
            if (mission_pos_cruise(4.9, 0.6, ALTITUDE, 0, err_max))
            {
                mission = 2;
                ROS_INFO("完成第%d圈第1个巡航点 (4.9, 0.6)", now_counts + 1);
            }else return false;
            break;
        case 2: // 第二点：(4.9, -0.6)
            if (mission_pos_cruise(4.9, -0.6, ALTITUDE, 0, err_max))
            {
                mission = 3;
                ROS_INFO("完成第%d圈第2个巡航点 (4.9, -0.6)", now_counts + 1);
            }else return false;
            break;
        case 3: // 第三点：(3.7, -0.6)
            if (mission_pos_cruise(3.7, -0.6, ALTITUDE, 0, err_max))
            {
                mission = 4;
                ROS_INFO("完成第%d圈第3个巡航点 (3.7, -0.6)", now_counts + 1);
            }else return false;
            break;
        case 4: // 第四点：(3.7, 0.6)
            if (mission_pos_cruise(3.7, 0.6, ALTITUDE, 0, err_max))
            {
                mission = 5;
                ROS_INFO("完成第%d圈第4个巡航点 (3.7, 0.6)", now_counts + 1);
            }else return false;
            break;
        case 5:                              // 完成1圈，重置状态
            now_counts++;                    // 已完成圈数+1
            mission = 1;                     // 回到第一点开始下一圈
            single_circle_start = ros::Time::now(); // 重置单圈计时起点（解决命名冲突后）
            ROS_INFO("✅ 完成第%d圈，累计完成%d圈（目标%d圈）",
                    now_counts, now_counts, counts);
            break;
        }
        ros::spinOnce();
        
        // ========== 新增：减缓循环频率（核心修改） ==========
        // 休眠 0.1 秒（循环频率 10Hz），可根据需要调整数值：
        // - 0.05秒 = 20Hz（更快）
        // - 0.2秒 = 5Hz（更慢）
        // - 1.0秒 = 1Hz（最慢）
        ros::Duration(0.1).sleep();
    }
    // 任务结束：打印最终状态
    ros::Duration total_dur = ros::Time::now() - total_start;
    if (now_counts >= counts)
    {
        ROS_INFO("🎉 转圈任务完成：共完成%d圈（目标%d圈），总耗时%.2f s（上限%.2f s）",
                now_counts, counts, total_dur.toSec(), times);
    }
    else if (total_dur.toSec() > times)
    {
        ROS_WARN("❌ 转圈任务总耗时超限：仅完成%d圈（目标%d圈），总耗时%.2f s（上限%.2f s）",
                now_counts, counts, total_dur.toSec(), times);
    }
    else
    {
        ROS_WARN("❌ 转圈任务异常终止：仅完成%d圈（目标%d圈），ROS节点退出",
                now_counts, counts);
    }
    return true;
}

void throwObject(Point obj,float t_yaw,double err_max){
    //下降高度
    ros::Time down = ros::Time::now();
    while(ros::ok()){
        // if(mission_pos_cruise(obj.x,obj.y,LOW_ALTITUDE,t_yaw,err_max)){
        //     break;
        // }else{
        //     if(ros::Time::now() - down >= ros::Duration(3.0)) break;
        // }
        if(ros::Time::now() - down >= ros::Duration(1.0)) break;
        ROS_INFO("降高度");
        ros::Duration(0.1).sleep();
    }
    //投掷
    ROS_INFO("开始投掷，投掷位置：%f %f",local_pos.pose.pose.position.x,local_pos.pose.pose.position.y);
    //暂时空着具体动作
    //上升高度
    ros::Time up = ros::Time::now();
    while(ros::ok()){
        // if(mission_pos_cruise(obj.x,obj.y,ALTITUDE,t_yaw,err_max)){
        //     break;
        // }else{
        //     if(ros::Time::now() - up >= ros::Duration(1.0)) break;
        // }
        if(ros::Time::now() - up >= ros::Duration(1.0)) break;
        ROS_INFO("升高度");
        ros::Duration(0.1).sleep();
    }
}