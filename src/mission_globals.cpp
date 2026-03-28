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
bool isThrow = false;
Point throw_pos;
int throwNum = 0;

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
cv::Mat camera_matrix = (cv::Mat_<double>(3, 3) << 1520.0, 0.0, 960.0,
                         0.0, 1520.0, 540.0,
                         0.0, 0.0, 1.0);
int H_direction = 0; //0就是没有，1就是left，-1就是right


// -------------------------- 配置参数（无人机场景优化） --------------------------
const float CONF_THRESHOLD = 0.4f;
const float SEARCH_RADIUS_SCALE = 2.0f;  // 灰环中心搜索黑正方形的范围（直径2倍）
const float INNER_IMG_SCALE = 0.7f;      // 中心图片占白色圆比例
const float APPROX_EPSILON = 0.03f;      // 轮廓逼近阈值（宽松，适配透视）
const std::string ONNX_MODEL_PATH = "/home/jetson/first_task_ws/src/flight_mission/best.onnx"; // 需替换为实际模型路径
const bool USE_GPU = false;              // 是否使用GPU推理

// 颜色范围（HSV，适配无人机下视光照）
const cv::Scalar GRAY_LOW = cv::Scalar(0, 0, 100);
const cv::Scalar GRAY_HIGH = cv::Scalar(180, 50, 200);
const cv::Scalar BLACK_LOW = cv::Scalar(0, 0, 0);
const cv::Scalar BLACK_HIGH = cv::Scalar(180, 255, 50);
const cv::Scalar WHITE_LOW = cv::Scalar(0, 0, 200);
const cv::Scalar WHITE_HIGH = cv::Scalar(180, 50, 255);

const cv::Size MORPHO_KERNEL = cv::Size(5, 5);  // 形态学核（轻量化）

// 通用工具相关
int timepiece = 1;
bool ERROR_DET = false;
bool isinit = false;
double init_x = 0, init_y = 0, init_z = 0, init_yaw = 0;

// 类实例化
ring all_ring(0.14, 0.06, 0.80, 0.15);
ring z_ring(0.25, 0.1, 0.85, 0.20);
Obstacle Obs;
Map M(map_width, map_length, map_cellsize);
std::vector<GridPoint> Path;

//检查类别定义
// CIFAR100 类别列表（与Python版本一致）
const std::vector<std::string> CIFAR100_CLASSES = {
    "apple", "aquarium_fish", "baby", "bear", "beaver", "bed", "bee", "beetle", 
    "bicycle", "bottle", "bowl", "boy", "bridge", "bus", "butterfly", "camel", 
    "can", "castle", "caterpillar", "cattle", "chair", "chimpanzee", "clock", 
    "cloud", "cockroach", "couch", "crab", "crocodile", "cup", "dinosaur", 
    "dolphin", "elephant", "flatfish", "forest", "fox", "girl", "hamster", 
    "house", "kangaroo", "keyboard", "lamp", "lawn_mower", "leopard", "lion",
    "lizard", "lobster", "man", "maple_tree", "motorcycle", "mountain", "mouse", 
    "mushroom", "oak_tree", "orange", "orchid", "otter", "palm_tree", "pear",
    "pickup_truck", "pine_tree", "plain", "plate", "poppy", "porcupine",
    "possum", "rabbit", "raccoon", "ray", "road", "rocket", "rose",
    "sea", "seal", "shark", "shrew", "skunk", "skyscraper", "snail", "snake",
    "spider", "squirrel", "streetcar", "sunflower", "sweet_pepper", "table",
    "tank", "telephone", "television", "tiger", "tractor", "train", "trout",
    "tulip", "turtle", "wardrobe", "whale", "willow_tree", "wolf", "woman", "worm"
};

// ========== 全局变量：存储二维码解析后的目标类别 ==========
std::vector<std::string> g_qrcode_classes;

// 扩张用数组
int dx[EXPAND_TWO] = {1, 1, 0, -1, -1, -1, 0, 1, 2, 0, -2, 0, 2, 2, 1, -1, -2, -2, -2, -2, -1, 1, 2, 2};
int dy[EXPAND_TWO] = {0, -1, -1, -1, 0, 1, 1, 1, 0, -2, 0, 2, -1, -2, -2, -2, -2, -1, 1, 2, 2, 2, 2, 1};
int Dx[EXPAND_ONE] = {1, 0, -1, 0, 1, -1, -1, 1};
int Dy[EXPAND_ONE] = {0, -1, 0, 1, -1, -1, 1, 1};
float weight[EXPAND_ONE] = {1, 1, 1, 1, 1.414, 1.414, 1.414, 1.414};