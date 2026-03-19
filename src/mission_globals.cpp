#include "mission_header.h"

/************************************************************************
全局变量定义
*************************************************************************/
// 飞控相关
mavros_msgs::PositionTarget setpoint_raw;
mavros_msgs::State current_state;
nav_msgs::Odometry local_pos;
tf::Quaternion quat;
double roll, pitch, yaw;
float init_position_x_take_off = 0;
float init_position_y_take_off = 0;
float init_position_z_take_off = 0;
float init_yaw_take_off = 0;
bool flag_init_position = false;

// 激光雷达相关
double nearest_ring[2];
double display[2];
bool cross_ring_flag = false;
bool find_ring = false;
std::vector<float> distance_bins;
std::vector<float> distance_bins_rotate;
std::vector<float> distance_bins_up;
std::vector<int> count_bins;
std::vector<int> points_on_ring;
float distance_c;
int angle_c;
const float point_err_x = 0.03;
const float point_err_y = 0.01;
double zero_plane_height = 0.0;
double height_threshold = 0.05;
double min_range = 0.1;
double max_range = 6.0;
int num_bins = 360;

// 避障相关
int mode = 1;
double direction = 1.0;
float map_cellsize = 0.10f;
float map_width = 11.0f;
float map_length = 11.0f;

// 视觉相关
cv::Mat current_frame;
bool got_image = false;
cv::Mat front_frame;
bool got_front_image = false;

    bool QR_detected=false;//判断是否检测到QR
    string num_or_letter;
    bool num_or_letter_detected=false;
    bool detected;

int aim_num=0;
int detect_count=0;//每5帧检测一次
float angle_cruise=0;
float d_angle=M_PI/3.0;
bool center_detected=false;

    cv::Point2f circular_center;
    bool circular_found=false;

bool center_detected_circular=false;
int detect_count_circular=0;//每5帧检测一次
float angle_cruise_circular=0;
float d_angle_circular=M_PI/3.0;

    std::vector<ColorRange> color_ranges = {
        // 红色（有两个范围）
        {cv::Scalar(0, 70, 50), cv::Scalar(10, 255, 255), "red"},
        {cv::Scalar(170, 70, 50), cv::Scalar(180, 255, 255), "red"},
        // 绿色
        {cv::Scalar(40, 70, 50), cv::Scalar(80, 255, 255), "green"},
        // 蓝色
        {cv::Scalar(100, 70, 50), cv::Scalar(130, 255, 255), "blue"}};

    cv::Mat camera_matrix = (cv::Mat_<double>(3, 3) << 1520.0, 0.0, 960.0,
                            0.0, 1520.0, 540.0,
                            0.0, 0.0, 1.0);


// 通用工具相关
int timepiece = 1;
bool ERROR_DET = false;
bool isinit = false;
double init_x = 0, init_y = 0, init_z = 0, init_yaw = 0;

// 类实例化
ring all_ring(0.14, 0.06, 0.85, 0.05);
ring z_ring(0.25, 0.1, 0.85, 0.20);
Obstacle Obs;
Map M(map_width, map_length, map_cellsize);
std::vector<GridPoint> Path;



// 扩张用数组
int dx[EXPAND_TWO] = {1, 1, 0, -1, -1, -1, 0, 1, 2, 0, -2, 0, 2, 2, 1, -1, -2, -2, -2, -2, -1, 1, 2, 2};
int dy[EXPAND_TWO] = {0, -1, -1, -1, 0, 1, 1, 1, 0, -2, 0, 2, -1, -2, -2, -2, -2, -1, 1, 2, 2, 2, 2, 1};
int Dx[EXPAND_ONE] = {1, 0, -1, 0, 1, -1, -1, 1};
int Dy[EXPAND_ONE] = {0, -1, 0, 1, -1, -1, 1, 1};
float weight[EXPAND_ONE] = {1, 1, 1, 1, 1.414, 1.414, 1.414, 1.414};