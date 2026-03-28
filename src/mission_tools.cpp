#include "mission_header.h"


/************************************************************************
通用工具函数实现
*************************************************************************/
double call_len(std::vector<float> p, int angle1, int angle2) // angle1为start，angle2为end
{
    // 利用余弦定理
    double edge1 = p[angle1];
    double edge2 = p[angle2];
    int dif_angle = angle2 - angle1;
    return sqrt(edge1 * edge1 + edge2 * edge2 - 2 * edge1 * edge2 * cos((float)dif_angle / 180 * 3.1415926));
}

double call_mid_len(std::vector<float> p, int angle1, int angle2) // angle1为start，angle2为end
{
    // 利用余弦定理
    double edge1 = p[angle1];
    double edge2 = p[angle2];
    int dif_angle = angle2 - angle1;
    return sqrt(edge1 * edge1 + edge2 * edge2 + 2 * edge1 * edge2 * cos((float)dif_angle / 180 * 3.1415926)) / 2; // 注意此处变为加号,计算的是已知角度的补角
}

double cal_y(std::vector<float> p, int angle)
{
    return -cos((float)angle / 180 * 3.1415926) * p[angle];
}

double cal_x(std::vector<float> p, int angle)
{
    return sin((float)angle / 180 * 3.1415926) * p[angle];
}

/************************************************************************
函数 7: satfunc
数据饱和函数，限制数据在±Max范围内
*************************************************************************/
void satfunc(float *data, float Max)
{
    float total_vel = sqrt(data[0] * data[0] + data[1] * data[1]);
    if (total_vel > Max)
    {
        data[0] = Max * data[0] / total_vel;
        data[1] = Max * data[1] / total_vel;
    }
    return;
}

// body to world consider yaw
Point rotation_yaw(float yaw_angle, const Point &p)
{
    // ROS_INFO("进入变换函数angle=%f，cos（angle）=%f,sin(angle)=%f",yaw_angle,cos(yaw_angle),sin(yaw_angle));
    Point q;
    q.x = p.x * cos(yaw_angle) - p.y * sin(yaw_angle) + local_pos.pose.pose.position.x;
    q.y = p.x * sin(yaw_angle) + p.y * cos(yaw_angle) + local_pos.pose.pose.position.y;
    return q;
}

void rotation(float yaw_angle, double &x, double &y)
{
    double q[2];
    q[0] = x * cos(yaw_angle) + y * sin(yaw_angle);
    q[1] = y * cos(yaw_angle) - x * sin(yaw_angle);
    x = q[0];
    y = q[1];
}

float GridPointDiatance(const GridPoint &p1, const GridPoint &p2)
{
    return sqrt((p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y));
}

void print2DArrayROS(int** grid, int cols, int rows) {
    if (grid == nullptr || cols <= 0 || rows <= 0) {
        ROS_WARN("二维数组未初始化，无法打印！");
        return;
    }
    //--------------------
    for(auto it = Path.begin();it != Path.end();++it){
        grid[it->x][it->y] = 2 ;
    }
    //-------------------
    GridPoint gp=M.PointToGridPoint({local_pos.pose.pose.position.x,local_pos.pose.pose.position.y});

    ROS_INFO("========== 二维数组打印开始 [cols=%d, rows=%d] ==========", cols, rows);
    for (int x = min(gp.x+5,rows-1); x>=0&&x>gp.x-5; --x) {
        std::stringstream ss;
        for (int y = min(gp.y+10,cols-1); y>0&&y>gp.y-10; --y) {
            if(x==gp.x&&y==gp.y) ss << "❤" << " ";
            else if(grid[x][y] == 2) ss << "■" << " ";
            else ss << grid[x][y] << " ";
        }
        ROS_INFO_STREAM(ss.str());
    }
    ROS_INFO("========== 二维数组打印结束 ==========\n");

    //---------------------
    for(auto it = Path.begin();it != Path.end();++it){
        grid[it->x][it->y] = 0 ;
    }
    //---------------------
}
GridPoint Fusion_Gpoint(int fused_num)
{
    int count = 0;
    GridPoint average(0, 0);
    for (auto it = Path.rbegin(); it != Path.rend() && count < fused_num; ++it)
    { // 注意还是++
        average += *it;
        ++count;
    }
    if (count == 0)
        throw "路径栈异常";
    average /= count;
    return average;
}

bool isobs(double x, double y, double total)
{
    double dx = x - local_pos.pose.pose.position.x;
    double dy = y - local_pos.pose.pose.position.y;
    ROS_ERROR("dx,dy:%f %f", dx, dy);
    double angle = atan2(dy, dx) - yaw;
    int n_angle = angle / 3.1415926 * 180 + 360;
    int str_angle = n_angle % 360;
    //ROS_WARN("angle,n_angle,str_angle:%f %d %d",angle,n_angle,str_angle);
    for (int i = str_angle - 30; i < str_angle + 30; i++)
    {
        int instead_i = (i + 360) % 360;
        if (distance_bins[instead_i] < total)
            return false;
    }
    return true;
}

bool is_exist_ring(ring &tr, std::vector<float> p)
{
    tr.stick_angle.clear(); // 清空历史杆状障碍物
    tr.ring_ifo.clear();
    tr.FindStick(p);
    if (tr.IsRing(p))
    {
        tr.NearestRing(p);
        return true;
    }
    else
        return false;
}

void init_location(double &init_x, double &init_y, double &init_z, double &init_yaw)
{
    init_x = local_pos.pose.pose.position.x;
    init_y = local_pos.pose.pose.position.y;
    init_z = local_pos.pose.pose.position.z - init_position_z_take_off;
    init_yaw = yaw * 180 / M_PI;
    isinit = true;
}

float min_obs_fuc(){
    float min_obs=0.35f;
    for(int i=60;i<=120;++i){
        if(distance_bins_rotate.at(i)<min_obs) min_obs = distance_bins_rotate.at(i);
    }
    return min_obs;
}

geometry_msgs::Point change_to_world(float u, float v)
{
    // 1. 像素坐标 → 相机坐标（z=1平面）
    cv::Mat pixel = (cv::Mat_<double>(3, 1) << u, v, 1.0);
    cv::Mat cam_point = camera_matrix.inv() * pixel;

    // 2. 用无人机高度计算相机到地面的距离
    double camera_height = local_pos.pose.pose.position.z - 0.1; // 无人机高度 - 相机安装高度

    double x_c = cam_point.at<double>(0) * camera_height;
    double y_c = cam_point.at<double>(1) * camera_height;
    double z_c = camera_height;

    double cos_yaw = cos(yaw);
    double sin_yaw = sin(yaw);

    double x_w = local_pos.pose.pose.position.x + (x_c * cos_yaw - y_c * sin_yaw);
    double y_w = local_pos.pose.pose.position.y + (x_c * sin_yaw + y_c * cos_yaw);
    double z_w = 0.0;

    geometry_msgs::Point p;
    p.x = x_w;
    p.y = y_w;
    p.z = z_w;
    ROS_WARN("中心位置 x%f , y%f", p.x , p.y);
    return p;
}