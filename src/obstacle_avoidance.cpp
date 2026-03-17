#include "obstacle_avoidance.h"
#include "mission_header.h"

/************************************************************************
避障相关函数实现
*************************************************************************/
// body
int PointToAngle(float x, float y)
{
    float t = atan2(y, x);
    int angle = static_cast<int>((t * 180.0 / M_PI)) + 90;
    if (t < 0)
        t += 360;
    if (t >= 360)
        t -= 360;
    return angle;
}

// body
Point AngleToPoint(int index)
{
    double angle = index / 180.0 * M_PI;
    double x1 = distance_bins_rotate[index] * sin(angle);
    double y1 = -distance_bins_rotate[index] * cos(angle);
    return Point(x1, y1);
}

void find_obstacal()
{
    int start = 0, end = 0;
    int counting = 0;
    Obs.all_obs = std::vector<Angle>();
    for (int i = 0; i < 180; i++)
    {
        if (counting)
        {
            if (distance_bins_rotate[i] < 5 && fabs(distance_bins_rotate[i] - distance_bins_rotate[i - 1]) < 0.15)
                continue;
            else
            {
                end = i - 1;
                counting = 0;
                if (end - start >= 3)
                {
                    Obs.all_obs.push_back({start, end});
                    // ROS_INFO("添加障碍物(%f,%f)", start, end);
                }
            }
        }
        else
        {
            if (distance_bins_rotate[i] < 5)
            {
                start = i;
                counting = 1;
            }
            else
                continue;
        }
    }
    // 处理循环结束后仍在统计的尾部区间
    if (counting)
    {
        end = 359; // 尾部区间的结束角是最后一个角度
        if (end - start >= 3)
        {
            Obs.all_obs.push_back({start, end});
            // ROS_INFO("添加障碍物(%f,%f)", start, end);
        }
    }
}

GridPoint out_of_danger(const GridPoint &now, int mode = 1)
{
    GridPoint safePoint(0, 0, -1, -1);
    if (mode == 1)
    {
        int count = 0;
        auto it = Path.rbegin();
        while (it != Path.rend() && count < 2)
        {
            count++;
            it++;
        }
        if (it == Path.rend())
            if (timepiece % 20)
            {
                ROS_INFO("无人机受困，正在进行第%d次尝试", timepiece);
                timepiece++;
            }
            else
            {
                ROS_ERROR("20次尝试无果，无人机迫降");
                ERROR_DET = true;
            }
        else
            timepiece = 1;
        GridPoint center = *it;
        safePoint = M.safe_Gpoint(center, center, EXPAND_TWO, true); // 因为设计问题，第二个参数在fusion==true时无效
    }
    else if (mode == 2)
    {
        while (!Path.empty())
        {
            safePoint = Path.back();
            if (M.Euclidean(now, safePoint) < 1.5)
            {
                Path.pop_back();
            }
            else
            {
                break;
            }
        }
        if (Path.empty())
            throw "终点附近有障碍物";
        safePoint = Path.back();
    }
    return safePoint;
}

bool collision_avoidance_mission(float target_x, float target_y, float target_z, float target_yaw, float err_max)
{
    GridPoint start = M.PointToGridPoint({local_pos.pose.pose.position.x, local_pos.pose.pose.position.y});
    GridPoint end = M.PointToGridPoint({target_x, target_y});
    GridPoint next = start;
    Point Pnext;
    // 加一个danger的判断条件，如果离真实障碍物过于近也判断为danger
    float min_obs = min_obs_fuc();

    // 是否处在障碍物扩张区 or 离障碍物过近
    bool danger = (M.Grid[start.x][start.y] == 1 || min_obs < 0.34f) ? true : false;

    // 如果danger，则前往最近且离终点最近的next点，几乎不可能出现的极端情况下前往附近的0点，否则正常更新
    if (danger)
    {
        ROS_INFO("危险区域");
        next = out_of_danger(start, 1);
        Pnext = M.GridPointToPoint(next);
    }
    else
    {
        // 如果在终点附近直接前往
        float dis = M.Euclidean(start, end);
        if (dis <= 5)
        {
            Pnext = {target_x, target_y};
            ROS_INFO("直接前往终点");
        }
        else
        {
            // 将所有障碍物存入Obs.all_obs里
            ROS_INFO("障碍物处理");
            find_obstacal();
            ROS_INFO("障碍物处理结束");

            // 更新地图M.Grid
            ROS_INFO("地图更新");
            M.update_map(EXPAND_SPL);
            ROS_INFO("地图更新结束");
            // Astar寻路
            ROS_INFO("开始寻路");
            if (M.Astar(start, end) == true)
            {
                ROS_INFO("寻路成功");
                ROS_INFO("多点综合");
                next = Fusion_Gpoint(3);
                ROS_INFO("综合完成%d %d", next.x, next.y);
            }
            else
            {
                ROS_INFO("寻路失败");
                ROS_INFO("重置地图");
                for (int i = 1; i < M.Xnum - 1; ++i)
                {
                    for (int j = 1; j < M.Ynum - 1; ++j)
                    {
                        M.Grid[i][j] = 0;
                    }
                }
            }
            Pnext = M.GridPointToPoint(next);
        }
    }

    print2DArrayROS(M.Grid, M.Xnum, M.Ynum);
    ROS_INFO("now (%.2f,%.2f,%.2f,%.2f)", local_pos.pose.pose.position.x, local_pos.pose.pose.position.y, local_pos.pose.pose.position.z, yaw * 180.0 / M_PI);

    // float dse_yaw = atan2(target_y-local_pos.pose.pose.position.y,target_x-local_pos.pose.pose.position.x);
    // ROS_INFO("atan2(%.2f-%.2f,%.2f-%.2f)=%.2f",target_y,local_pos.pose.pose.position.y,target_x,local_pos.pose.pose.position.x,dse_yaw);

    // ROS_INFO("切换定点模式");
    setpoint_raw.type_mask = /*1 + 2 + 4+*/ 8 + 16 + 32 + 64 + 128 + 256 + 512 /*+ 1024 */ + 2048;
    setpoint_raw.coordinate_frame = 1;
    setpoint_raw.position.x = Pnext.x;
    setpoint_raw.position.y = Pnext.y;
    setpoint_raw.position.z = target_z + init_position_z_take_off;
    setpoint_raw.yaw = target_yaw / 180.0 * M_PI;

    ROS_INFO("目标点为（%f，%f）", Pnext.x, Pnext.y);
    if (fabs(local_pos.pose.pose.position.x - target_x - init_position_x_take_off) < err_max && fabs(local_pos.pose.pose.position.y - target_y - init_position_y_take_off) < err_max && fabs(local_pos.pose.pose.position.z - target_z - init_position_z_take_off) < err_max && fabs(yaw - target_yaw / 180.0 * M_PI) < 0.1)
    {
        return true;
    }
    return false;
}