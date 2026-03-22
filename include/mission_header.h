#ifndef MISSION_HEADER_H
#define MISSION_HEADER_H

#include <string>
#include <vector>
#include <queue>
#include <quirc.h>
#include <cmath>
#include <climits>
#include <cstring>
#include <cstdlib>
#include <numeric>
#include <algorithm>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <std_msgs/Bool.h>
#include <mavros_msgs/PositionTarget.h>
#include <cmath>
#include <tf/transform_listener.h>
#include <nav_msgs/Odometry.h>
#include <mavros_msgs/CommandLong.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Int32.h>
#include <livox_ros_driver/CustomMsg.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/objdetect.hpp>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/Image.h>
#include <opencv2/imgproc/imgproc_c.h>
#include <onnxruntime_cxx_api.h> //yolo新加
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
// 动态参数
// 回退逻辑

using namespace std;

/************************************************************************
全局常量定义
*************************************************************************/
#define ALTITUDE 2.0f
#define LOW_ALTITUDE 0.5f //待定
#define RING_ALTITUDE 1.45f  //由于仿真环境与真实比赛场地有差异，具体高度一定要进行实地考察 比赛改为 1.5/1.6
#define MAX_X 6.6f
#define MAX_Y -1.0f
#define MIN_X 5.4f
#define COUNTS 3  // 需要无人机转的圈数                        //待定，目标是超过70圈
#define TIMES 60.0f  //转圈的最大时长，一旦超时剩多少圈都不转了    //7分钟？
#define OBSTACLE_WIDTH 0.3
#define EPS 1e-3
#define EXPAND_ONE 8
#define EXPAND_SPL 24
#define EXPAND_TWO 24
#define FLOAT_MAX std::numeric_limits<float>::max()

/************************************************************************
全局变量声明
*************************************************************************/
// 飞控相关
extern mavros_msgs::PositionTarget setpoint_raw;
extern mavros_msgs::State current_state;
extern nav_msgs::Odometry local_pos;
extern tf::Quaternion quat;
extern double roll, pitch, yaw;
extern float init_position_x_take_off;
extern float init_position_y_take_off;
extern float init_position_z_take_off;
extern float init_yaw_take_off;
extern bool flag_init_position;
extern bool isThrow;
extern int throwNum;

// 激光雷达相关
extern double nearest_ring[2];
extern double display[2];
extern bool cross_ring_flag;
extern bool find_ring;
extern std::vector<float> distance_bins;
extern std::vector<float> distance_bins_rotate;
extern std::vector<float> distance_bins_up;
extern std::vector<int> count_bins;
extern std::vector<int> points_on_ring;
extern float distance_c;
extern int angle_c;
extern const float point_err_x;
extern const float point_err_y;
extern double zero_plane_height;
extern double height_threshold;
extern double min_range;
extern double max_range;
extern int num_bins;

// 避障相关
extern int mode;
extern double direction;
extern float map_cellsize;
extern float map_width;
extern float map_length;
extern int dx[EXPAND_TWO];
extern int dy[EXPAND_TWO];
extern int Dx[EXPAND_ONE];
extern int Dy[EXPAND_ONE];
extern float weight[EXPAND_ONE];

// 视觉相关
extern cv::Mat current_frame;
extern bool got_image;
extern cv::Mat camera_matrix;
extern int H_direction; 
extern std::vector<std::string> g_qrcode_classes;
extern const std::vector<std::string> CIFAR100_CLASSES;

// 注意：extern 仅做声明，具体定义需在某个 .cpp 文件中实现（无 extern 关键字）
extern const float CONF_THRESHOLD;
extern const float SEARCH_RADIUS_SCALE;  // 灰环中心搜索黑正方形的范围（直径2倍）
extern const float INNER_IMG_SCALE;      // 中心图片占白色圆比例
extern const float APPROX_EPSILON;      // 轮廓逼近阈值（宽松，适配透视）
extern const std::string ONNX_MODEL_PATH; // 需替换为实际模型路径
extern const bool USE_GPU;              // 是否使用GPU推理

// 颜色范围（HSV，适配无人机下视光照）
extern const cv::Scalar GRAY_LOW;
extern const cv::Scalar GRAY_HIGH;
extern const cv::Scalar BLACK_LOW;
extern const cv::Scalar BLACK_HIGH;
extern const cv::Scalar WHITE_LOW;
extern const cv::Scalar WHITE_HIGH;

extern const cv::Size MORPHO_KERNEL;  // 形态学核（轻量化）

// 通用工具相关
extern int timepiece;
extern bool ERROR_DET;
extern bool isinit;
extern double init_x, init_y, init_z, init_yaw;

/************************************************************************
结构体定义
*************************************************************************/
struct Angle
{
    int start, end;
    Angle(int _start, int _end) : start(_start), end(_end) {}
} ;

struct AddPos
{
    double x = 0, y = 0, d;
    Angle R;
    AddPos(int _start, int _end, double _x, double _y, double _d) : R(_start, _end), x(_x), y(_y), d(_d) {}
} ;

struct Point
{
    double x, y;
    Point(double x_ = 0.0f, double y_ = 0.0f) : x(x_), y(y_) {}
    Point(const Point &p) = default;
};

extern Point throw_pos;

struct GridPoint
{
    int x, y;
    float g, h;
    GridPoint(int x_ = 0, int y_ = 0, float g_ = 0, float h_ = 0) : x(x_), y(y_), g(g_), h(h_) {}
    GridPoint(const GridPoint &p) = default;
    bool operator==(const GridPoint &GP)
    {
        if (x == GP.x && y == GP.y)
            return true;
        return false;
    }
    const GridPoint &operator+=(const GridPoint &GP)
    {
        x += GP.x;
        y += GP.y;
        return *this;
    }
    const GridPoint &operator/=(int p)
    {
        x /= p;
        y /= p;
        return *this;
    }
    GridPoint &operator=(const GridPoint &GP)
    {
        x = GP.x;
        y = GP.y;
        g = GP.g;
        h = GP.h;
        return *this;
    }
};

// 障碍物类
class Obstacle
{
public:
    std::vector<Angle> all_obs;
    int min_width; // 障碍物最小宽度
    int max_width; // 障碍物最大宽度
    Obstacle(int _min_width = 0, int _max_width = INT_MAX)
    {
        min_width = _min_width;
        max_width = _min_width;
    }
};


// -------------------------- 结构体：无人机检测结果（供飞控调用） --------------------------

struct UavDetectResult {
    bool is_detected = false;          // 是否检测到靶子
    std::string class_name = "unknown";// 分类类别（修复：与实现对齐）
    float confidence = 0.0f;           // 分类置信度
    cv::Point gray_ring_center;        // 灰环中心（修复：与实现对齐）
    cv::Rect black_square;             // 黑色正方形
    float square_angle = 0.0f;         // 正方形偏转角度
    cv::Rect inner_image_rect;         // 中心图片区域（修复：与实现对齐）
};

/************************************************************************
类定义
*************************************************************************/
class stick
{
private:
    double stick_width;
    double std_stick_err;

public:
    std::vector<Angle> stick_angle; // 储存角度下标<start_angle,end_angle>
    stick(double _stick_width, double _std_stick_err) : stick_width(_stick_width),
                                                        std_stick_err(_std_stick_err) {};

    ~stick() = default;
    void FindStick(std::vector<float> p);
};

class ring : public stick
{
private:
    double inner_width;
    double std_ring_err;

public:
    std::vector<AddPos> ring_ifo; // 储存环的所有信息
    ring(double _stick_width, double _std_stick_err, double _inner_width, double _std_ring_err)
        : stick(_stick_width, _std_stick_err),
          inner_width(_inner_width),
          std_ring_err(_std_ring_err) {};
    ~ring() {};

    bool IsRing(std::vector<float> p);
    void NearestRing(std::vector<float> p); // 理应只返回偏移量x,y 但是懒得再额外写一个结构体 并且其他数据可能会发挥作用 故一块返回了
};

extern ring all_ring;
extern ring z_ring;

class Map
{
public:
    int **Grid;     // 地图栅格,0为可走,1为障碍物
    float width;    // 世界坐标系下y方向的长度
    float length;   // 世界坐标系下x方向的长度
    int Ynum;       // y方向格数
    int Xnum;       // x方向格数
    float CellSize; // 栅格边长
    int delta_x;
    int delta_y; // 世界坐标系（x，y）-->地图坐标系（x，y+delta_y）
    Map(float _width, float _length, float _cellsize);
    ~Map();
    GridPoint PointToGridPoint(const Point &P); // ceil上取整
    GridPoint PointToGridPoint_World(const Point &P);
    GridPoint PointToGridPoint(const Point &P, int judge); // round四舍五入，扩张用
    Point GridPointToPoint(const GridPoint &p);
    GridPoint safe_Gpoint(const GridPoint &now, const GridPoint &end, int EXPAND_NUM, bool Fusion);
    float Manhattan(const GridPoint &start, const GridPoint &end);
    float Euclidean(const GridPoint &p1, const GridPoint &p2);
    void update_map(int EXPAND_NUM);
    bool Astar(const GridPoint &start, const GridPoint &end);
    bool qualify(const GridPoint &p);
};

extern Obstacle Obs;
extern Map M;
extern std::vector<GridPoint> Path;

struct CompareF
{
    // 返回true则b排在a前面
    bool operator()(const GridPoint &a, const GridPoint &b)
    {
        if ((a.g + a.h) == (b.g + b.h))
            return a.h > b.h;
        return (a.g + a.h) > (b.g + b.h);
    }
};

/************************************************************************
通用工具函数声明
*************************************************************************/
double call_len(std::vector<float> p, int angle1, int angle2);     // angle1为start，angle2为end
double call_mid_len(std::vector<float> p, int angle1, int angle2); // angle1为start，angle2为end
double cal_y(std::vector<float> p, int angle);
double cal_x(std::vector<float> p, int angle);
void satfunc(float *data, float Max);
Point rotation_yaw(float yaw_angle, const Point &p);
void rotation(float yaw_angle, double &x, double &y);
int PointToAngle(float x, float y);
Point AngleToPoint(int index);
float GridPointDiatance(const GridPoint &p1, const GridPoint &p2);
void print2DArrayROS(int **grid, int cols, int rows);
GridPoint Fusion_Gpoint(int fused_num);
bool isobs(double x, double y, double total);
bool is_exist_ring(ring &tr, std::vector<float> p);
void init_location(double &init_x, double &init_y, double &init_z, double &init_yaw);
geometry_msgs::Point change_to_world(float u, float v);
float min_obs_fuc();

#endif