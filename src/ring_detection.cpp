#include "ring_detection.h"
#include "flight_control.h"
#include "mission_header.h"

/************************************************************************
穿环相关函数实现
*************************************************************************/
void stick::FindStick(std::vector<float> p)
{
    int start = 0, end = 0;
    double len = 0;
    int counting = 0;
    for (int i = 1; i < 180; i++)
    {
        if (counting)
        {
            if (p[i] < 5 && fabs(p[i] - p[i - 1]) < 0.5)
                continue;
            else
            {
                end = i - 1;
                if (end - start >= 1)
                {
                    len = call_len(p, start, end);
                    if (len < stick_width + std_stick_err && len > stick_width - std_stick_err)
                    {
                    	 if(!stick_angle.empty()){
                    	 	auto it = stick_angle.end() - 1;
                    	 	if(start - (*it).end <= 3 && fabs(p[start] - p[(*it).end]) < 0.3){
                    	 		start = (*it).start;
                    	 		stick_angle.pop_back();
                    	 		ROS_WARN("这个⬆️不算");
                    	 	}
                    	 }
                    	 len = call_len(p, start, end);
                    	 if (len < stick_width + std_stick_err && len > stick_width - std_stick_err){
                        stick_angle.push_back(Angle(start, end));
                        ROS_INFO("添加障碍物距离 %fm 范围(%d,%d)，宽度%f",p[end],start, end, len);
                        }
                    }
                }else continue;
                if (p[i] >= 5)
                {
                    counting = 0;
                }
                else
                {
                    start = i;
                }
            }
        }
        else
        {
            if (p[i] < 5)
            {
                start = i;
                //if (start - end <= 2) continue;
                counting = 1;
            }
            else
                continue;
        }
    }
    // 处理循环结束后仍在统计的尾部区间
    if (counting)
    {
        end = 269; // 尾部区间的结束角是最后一个角度（269）
        len = call_len(p, start, end);
        if (len < stick_width + std_stick_err && len > stick_width - std_stick_err)
            stick_angle.push_back(Angle(start, end));
    }
}

bool ring::IsRing(std::vector<float> p)
{
    double len;
    int flag = 1;
    bool exist = false;
    ring_ifo.clear();
    for (auto R_one = stick_angle.begin(); R_one != stick_angle.end(); ++R_one)
    {
        for (auto L_one = R_one + 1; L_one != stick_angle.end(); ++L_one)
        {
            len = call_len(p, R_one->end, L_one->start);
            int rs = (int)R_one->end, re = (int)L_one->start;
            ROS_INFO("障碍物间距 %f m", len);
            if (len < inner_width + std_ring_err && len > inner_width - std_ring_err)
            {
                for (int i = rs + 1; i < re; i++)
                {
                    if (p[i] < p[rs] || p[i] < p[re])
                    {
                        flag = 0;
                        break;
                    }
                }
                if (flag)
                {
                    ROS_WARN("符合条件");
                    // 将圆环左右边界写入，中心关于无人机的偏移量x和y先不计算，等返回时计算。AddPos(_start, _end,_x,_y,_d)
                    ring_ifo.push_back(AddPos(R_one->end, L_one->start, 0, 0, call_mid_len(p, R_one->end, L_one->start)));
                    exist = true;
                    flag = 0;
                }
            }
        }
    }
    return exist;
}

void ring::NearestRing(std::vector<float> p)
{
    auto min_i = ring_ifo.begin();
    for (auto it = ring_ifo.begin() + 1; it != ring_ifo.end(); ++it)
    {
        if (it->d < min_i->d)
            min_i = it;
    }

    // min_i->x=(-cal_x(min_i->R.start-90)+cal_x(min_i->R.end-90))/2;
    // min_i->y=(cal_y(min_i->R.start-90)+cal_y(min_i->R.end-90))/2;
    // return *min_i;

    ROS_INFO("min_i->R=(%d + %d)", min_i->R.start, min_i->R.end);
    ROS_INFO("nearest_ring[0]=(%f + %f) /2 ", cal_x(p, min_i->R.start), cal_x(p, min_i->R.end));
    ROS_INFO("nearest_ring[1]=(%f + %f) /2 ", cal_y(p, min_i->R.start), cal_y(p, min_i->R.end));
    nearest_ring[0] = (cal_x(p, min_i->R.start) + cal_x(p, min_i->R.end)) / 2;
    nearest_ring[1] = (cal_y(p, min_i->R.start) + cal_y(p, min_i->R.end)) / 2;
}

bool cross_ring(double x, double y, double z, double t_yaw, double err_max)
{
    static int counts = 1;
    ROS_ERROR("mode = %d", mode);
    double total_distance = sqrt((x - local_pos.pose.pose.position.x) * (x - local_pos.pose.pose.position.x) + (y - local_pos.pose.pose.position.y) * (y - local_pos.pose.pose.position.y));
    if (!find_ring)
    {
        if (!is_exist_ring(all_ring, distance_bins_rotate))
        {
            ROS_ERROR("开始判断");
            if (isobs(x, y, total_distance))
                return mission_pos_cruise(x, y, z, t_yaw, err_max);
            else if (move_in_drone_coordinate(0, 0, -0.05, 0, err_max))
                isinit = false;
        }
        else
        {
            double distance_r = sqrt(nearest_ring[0] * nearest_ring[0] + nearest_ring[1] * nearest_ring[1]);
            display[0] = nearest_ring[0];
            display[1] = nearest_ring[1];
            rotation(-yaw, display[0], display[1]);
            if (distance_r < total_distance && display[0] + local_pos.pose.pose.position.x < MAX_X && display[1] + local_pos.pose.pose.position.y < MAX_Y && display[0] + local_pos.pose.pose.position.x > MIN_X)
            {
                
                ROS_WARN("发现圆环，环中心位置(%f,%f)", display[0] + local_pos.pose.pose.position.x, display[1] + local_pos.pose.pose.position.y);
                find_ring = true;
                cross_ring_flag = true;
            }
            else
            {
                ROS_ERROR("无圆环");
                if (isobs(x, y, total_distance))
                    return mission_pos_cruise(x, y, z, t_yaw, err_max);
                else if (move_in_drone_coordinate(0, 0, -0.05, 0, err_max))
                    isinit = false;
            }
        }
    }
    else
    {
        switch (mode)
        {
        case 1:   //粗校准
        {
            if (move_in_drone_coordinate(nearest_ring[0] - 0.7, nearest_ring[1], 0, 0, err_max))
            {
                isinit = false;
                ROS_INFO("对准水平中心位置！");
                mode = 2;
            }
            break;
        }
        case 2: //二次识别
        {
            if (counts % 30 && !is_exist_ring(all_ring, distance_bins_rotate)) //多识别几次提高容错
            {
                ROS_INFO("第%d次判断", counts);
                counts++;
            }
            else
            {
                if (counts % 30 == 0)
                {
                    if (isobs(x, y, total_distance))
                    {
                        if (mission_pos_cruise(x, y, z, t_yaw, err_max))
                            mode = 7;
                    }
                    else
                    {
                        ROS_INFO("非圆环！");
                        mode = 7;
                    }
                }
                else
                {
                    display[0] = nearest_ring[0];
                    display[1] = nearest_ring[1];
                    rotation(-yaw, display[0], display[1]);
                    ROS_WARN("环中心位置(x,z):(%f,%f)", display[0] + local_pos.pose.pose.position.x, display[1] + local_pos.pose.pose.position.z);
                    mode = 3;
                }
                counts = 1;
            }
            break;
        }
        case 3:
        {
            if (move_in_drone_coordinate(nearest_ring[0] - 0.5, nearest_ring[1], 0, 0, err_max, 1)){ //精校准
                isinit = false;
                ROS_INFO("水平位置矫正完成！");
                mode = 6;
            }
            break;
        }
        // case 4:
        // {
        //     if (counts % 30 && !is_exist_ring(z_ring, distance_bins_up))
        //     {
        //         ROS_INFO("第%d次判断", counts);
        //         counts++;
        //     }
        //     else
        //     {
        //         if (counts % 30 == 0)
        //         {
        //             if (isobs(x, y, total_distance))
        //             {
        //                 if (mission_pos_cruise(x, y, z, t_yaw, err_max))
        //                     mode = 7;
        //             }
        //             else
        //             {
        //                 ROS_INFO("非圆环！");
        //                 mode = 7;
        //             }
        //         }
        //         else
        //         {
        //             ROS_WARN("环中心位置高度:(%f)", nearest_ring[1] + local_pos.pose.pose.position.z);
        //             mode = 5;
        //         }
        //         counts = 1;
        //     }
        //     break;
        // }
        // case 5:
        // {
        //     if (move_in_drone_coordinate(0, 0, -nearest_ring[1], 0, err_max))
        //     {
        //         isinit = false;
        //         ROS_INFO("竖直位置矫正完成！");
        //         mode = 6;
        //     }
        //     break;
        // }
        case 6:
        {
            if (move_in_drone_coordinate(2.0, 0, 0.05, 0, err_max))
            {
                isinit = false;
                ROS_INFO("穿环完成！");
                mode = 7;
            }
            break;
        }
        case 7:
        {
            if (mission_pos_cruise(x, y, z, t_yaw, err_max)){
                mode = 1;
                find_ring = false;
                cross_ring_flag = false;
                ROS_INFO("穿环完成！");
                return true;
            }
            break;
        }
        }
    }
    return false;
}