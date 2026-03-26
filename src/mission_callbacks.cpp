#include "mission_callbacks.h"
#include "mission_header.h"

/************************************************************************
函数 1：无人机状态回调函数
*************************************************************************/
void state_cb(const mavros_msgs::State::ConstPtr &msg)
{
    current_state = *msg;
}

/************************************************************************
函数 2：回调函数接收无人机的里程计信息
从里程计信息中提取无人机的位置信息和姿态信息
*************************************************************************/
void local_pos_cb(const nav_msgs::Odometry::ConstPtr &msg)
{
    local_pos = *msg;
    tf::quaternionMsgToTF(local_pos.pose.pose.orientation, quat);
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
    if (flag_init_position == false && (local_pos.pose.pose.position.z != 0))
    {
        init_position_x_take_off = local_pos.pose.pose.position.x;
        init_position_y_take_off = local_pos.pose.pose.position.y;
        init_position_z_take_off = local_pos.pose.pose.position.z;
        init_yaw_take_off = yaw;
        flag_init_position = true;
    }
}

/************************************************************************
函数 6:lidar_cb
点云回调函数，处理Livox雷达的点云数据，实现360度障碍物检测
/livox/lidar example:
*************************************************************************/
void livox_custom_cb(const livox_ros_driver::CustomMsg::ConstPtr &livox_msg)
{
    // 初始化bins
    // ROS_INFO("Received Livox point cloud with %d points", livox_msg->point_num);
    distance_bins.assign(num_bins, max_range);
    distance_bins_up.assign(num_bins, max_range);
    distance_bins_rotate.assign(num_bins, 0);
    count_bins.assign(num_bins, 0);
    points_on_ring.clear();
    int total_points = livox_msg->point_num;
    int plane_points = 0;
    // 遍历Livox自定义消息中的点
    for (int i = 0; i < total_points; i++)
    {
        const livox_ros_driver::CustomPoint &point = livox_msg->points[i];

        float x = point.x;
        float y = point.y;
        float z = point.z;

        // 筛选0度平面附近的点
        if (fabs(z - zero_plane_height) > height_threshold && fabs(y - zero_plane_height) > height_threshold)
        {
            continue;
        }
        plane_points++;

        if (fabs(z - zero_plane_height) <= height_threshold)
        {
            // 计算距离和角度
            float distance = sqrt(x * x + y * y);
            float angle = atan2(y, x); // 弧度  返回值[-pi,pi] 不用担心x，y同正同负的情况

            // 转换为角度并映射到0-359
            int angle_bin = static_cast<int>((angle * 180.0 / M_PI));

            // 转换为0-359范围
            if (angle_bin < 0)
                angle_bin += 360;
            if (angle_bin >= 360)
                angle_bin -= 360;

            // 确保在有效范围内
            if (angle_bin >= 0 && angle_bin < num_bins)
            {
                // 只保留每个角度bin的最小距离
                if (distance >= min_range && distance <= max_range &&
                    !std::isinf(distance) && !std::isnan(distance))
                {
                    if (distance < distance_bins[angle_bin])
                    {
                        distance_bins[angle_bin] = distance;
                    }
                    count_bins[angle_bin]++;
                }
            }
        }

        if (fabs(y - zero_plane_height) <= height_threshold)
        {
            // 计算距离和角度
            float distance = sqrt(x * x + z * z);
            float angle = atan2(x, z); // 弧度  返回值[-pi,pi] 不用担心x，y同正同负的情况

            // 转换为角度并映射到0-359
            int angle_bin = static_cast<int>((angle * 180.0 / M_PI));

            // 转换为0-359范围
            if (angle_bin < 0)
                angle_bin += 360;
            if (angle_bin >= 360)
                angle_bin -= 360;

            // 确保在有效范围内
            if (angle_bin >= 0 && angle_bin < num_bins)
            {
                // 只保留每个角度bin的最小距离
                if (distance >= min_range && distance <= max_range &&
                    !std::isinf(distance) && !std::isnan(distance))
                {
                    if (distance < distance_bins_up[angle_bin])
                    {
                        distance_bins_up[angle_bin] = distance;
                    }
                }
            }
        }
    }
    for (int i = 0; i < num_bins; i++)
    {
        if (distance_bins[i] == 0)
        {
            distance_bins[i] = max_range; // 如果该bin没有点，则设为最大距离
        }
        if (distance_bins_up[i] == 0)
        {
            distance_bins_up[i] = max_range; // 如果该bin没有点，则设为最大距离
        }
        // ROS_INFO("Angle Bin %d: Min Distance = %.2f m, Point Count = %d", i, distance_bins[i], count_bins[i]);
    }
    // 将无人机前方的角度整合
    for (int i = 0, j = 270; i < num_bins; ++i, ++j)
    {
        distance_bins_rotate.at(i) = distance_bins.at(j % 360);
    }
    //for(int i = 0;i < 180; i++) ROS_ERROR("%d : %f",i, distance_bins_rotate[i]); 
}

void image_cb(const sensor_msgs::ImageConstPtr &msg)
{
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception &e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    current_frame = cv_ptr->image;
    got_image = true;
}

/************************************************************************
函数1 :ego_planner导航
*************************************************************************/

void ego_sub_cb(const quadrotor_msgs::PositionCommand::ConstPtr &msg)
{
	ego_sub = *msg;
}

/************************************************************************
函数2 : ego_planner是否规划出航线
*************************************************************************/

void rec_traj_cb(const std_msgs::Bool::ConstPtr &msg)
{
	rec_traj_flag = msg->data;
	// ROS_WARN("rec_traj_flag: %d", rec_traj_flag);
}