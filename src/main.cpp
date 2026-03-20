#include "flight_control.h"
#include "obstacle_avoidance.h"
#include "ring_detection.h"
#include "vision_detection.h"
#include "mission_callbacks.h"
#include "mission_header.h"

#define DELAY 1.0
#define LEFT 90
#define RIGHT -90
#define BACK -179
// 全局变量定义
int mission_num = 0;
float if_debug = 0;
float err_max = 0.2;
bool delay = false;
ros::Time last_request;
void print_param()
{
    std::cout << "=== 控制参数 ===" << std::endl;
    std::cout << "err_max: " << err_max << std::endl;
    std::cout << "ALTITUDE: " << ALTITUDE << std::endl;
    std::cout << "if_debug: " << if_debug << std::endl;
    if (if_debug == 1)
        cout << "自动offboard" << std::endl;
    else
        cout << "遥控器offboard" << std::endl;
}

void Delay(float delay_time)
{
    if (delay)
    {
        ros::Duration delta_time = ros::Time::now() - last_request;
        if (delta_time.toSec() > 0)
            ROS_WARN("延时%.2f s", delta_time.toSec());
        else
            last_request = ros::Time::now();
        if (ros::Time::now() - last_request >= ros::Duration(delay_time))
        {
            ROS_INFO("延时结束");
            delay = false;
            mission_num += 1;
        }
    }
    else
    {
        ROS_INFO("延时开始");
        last_request = ros::Time::now();
        delay = true;
    }
}

int main(int argc, char **argv)
{
    // 防止中文输出乱码
    setlocale(LC_ALL, "");

    // 初始化ROS节点
    ros::init(argc, argv, "flight_mission");
    ros::NodeHandle nh;

    // 订阅mavros相关话题
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, state_cb);
    ros::Subscriber local_pos_sub = nh.subscribe<nav_msgs::Odometry>("/mavros/local_position/odom", 10, local_pos_cb);
    ros::Subscriber image_sub = nh.subscribe<sensor_msgs::Image>("/camera/image_raw", 10, image_cb);
    // 发布无人机多维控制话题
    ros::Publisher mavros_setpoint_pos_pub = nh.advertise<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/local", 100);
    ros::Subscriber livox_sub = nh.subscribe<livox_ros_driver::CustomMsg>("/livox/lidar", 10, livox_custom_cb);

    // 创建服务客户端
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
    ros::ServiceClient ctrl_pwm_client = nh.serviceClient<mavros_msgs::CommandLong>("mavros/cmd/command");

    // 设置话题发布频率，需要大于2Hz，飞控连接有500ms的心跳包
    ros::Rate rate(20);

    // 参数读取

    nh.param<float>("err_max", err_max, 0.2);
    nh.param<float>("if_debug", if_debug, 0);
    nh.param<double>("zero_plane_height", zero_plane_height, 0);
    nh.param<double>("height_threshold", height_threshold, 0.05);
    nh.param<double>("min_range", min_range, 0.1);
    nh.param<double>("max_range", max_range, 30.0);
    nh.param<int>("num_bins", num_bins, 360);
    //nh.param<std::string>("target_color", target_color, "blue");

    nh.param<float>("map_cellsize", map_cellsize, 0.10);
    nh.param<float>("map_width", map_width, 10.0);
    nh.param<float>("map_length", map_length, 10.0);
    print_param();

    // std::string cascade_path = "/usr/share/opencv4/haarcascades/haarcascade_frontalface_default.xml";
    // if (!face_cascade.load(cascade_path))
    // {
    //     ROS_ERROR("无法加载人脸检测器分类器文件：%s", cascade_path.c_str());
    //     return -1;
    // }
    // else
    // {
    //     ROS_INFO("成功加载人脸检测器分类器文件");
    // }

    int choice = 0;
    std::cout << "1 to go on , else to quit" << std::endl;
    std::cin >> choice;
    if (choice != 1)
        return 0;
    ros::spinOnce();
    rate.sleep();

    // 等待连接到飞控
    while (ros::ok() && !current_state.connected)
    {
        ros::spinOnce();
        rate.sleep();
    }
    // 设置无人机的期望位置

    setpoint_raw.type_mask = /*1 + 2 + 4 + 8 + 16 + 32*/ +64 + 128 + 256 + 512 /*+ 1024 + 2048*/;
    setpoint_raw.coordinate_frame = 1;
    setpoint_raw.position.x = 0;
    setpoint_raw.position.y = 0;
    setpoint_raw.position.z = ALTITUDE;
    setpoint_raw.yaw = 0;

    // send a few setpoints before starting
    for (int i = 100; ros::ok() && i > 0; --i)
    {
        mavros_setpoint_pos_pub.publish(setpoint_raw);
        ros::spinOnce();
        rate.sleep();
    }
    std::cout << "ok" << std::endl;

    // 定义客户端变量，设置为offboard模式
    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    // 定义客户端变量，请求无人机解锁
    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    // 记录当前时间，并赋值给变量last_request
    ros::Time last_request = ros::Time::now();

    while (ros::ok())
    {
        if (current_state.mode != "OFFBOARD" && (ros::Time::now() - last_request > ros::Duration(3.0)))
        {
            if (if_debug == 1)
            {
                if (set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent)
                {
                    ROS_INFO("Offboard enabled");
                }
            }
            else
            {
                ROS_INFO("Waiting for OFFBOARD mode");
            }
            last_request = ros::Time::now();
        }
        else
        {
            if (!current_state.armed && (ros::Time::now() - last_request > ros::Duration(3.0)))
            {
                if (arming_client.call(arm_cmd) && arm_cmd.response.success)
                {
                    ROS_INFO("Vehicle armed");
                }
                last_request = ros::Time::now();
            }
        }
        // 当无人机到达起飞点高度后，悬停3秒后进入任务模式，提高视觉效果
        if (fabs(local_pos.pose.pose.position.z - init_position_z_take_off - ALTITUDE) < 0.05)
        {
            if (ros::Time::now() - last_request > ros::Duration(1.0))
            {
                mission_num = 1;
                last_request = ros::Time::now();
                break;
            }
        }

        mission_pos_cruise(0, 0, ALTITUDE, 0, err_max);
        mavros_setpoint_pos_pub.publish(setpoint_raw);
        ros::spinOnce();
        rate.sleep();
    }

    while (ros::ok())
    {
        ROS_WARN("mission_num = %d", mission_num);
        switch (mission_num)
        {
        case 1:  //起飞
            if (mission_pos_cruise(0, 0, ALTITUDE, 0, err_max))
            {
                Delay(DELAY);
            }
            break;
        case 2: //前进1米8 (1个我的距离)
            if (collision_avoidance_mission(1.8, 0, LOW_ALTITUDE, 0, err_max))
            {
                Delay(0.5);
            }
            break;
        case 3: //扫码
            if (detectQRCodeAndExtractInfo())
            {
                mission_num = 4; // 理论上完全可以Delay(0);
            }
            break;
        /* 

            //此处保留了原来的逻辑，需要对比时方便修改。

        case 4: // 准备转圈
            if (collision_avoidance_mission(3.7, 0.6, ALTITUDE, 0, err_max))
            {
                Delay(0.5);
            }
            break;

        case 5:
            if (Circle_around(COUNTS, TIMES, err_max))
            {
                Delay(DELAY);
            }
            break;
        */
        case 4: //准备转圈
            if (collision_avoidance_mission(3.7, 0.6, ALTITUDE, LEFT, err_max))
            {
                Delay(0.5);
            }
            break;

        case 5:
            if (Circle_around(3,60.0f,ALTITUDE,0.75,0.75,4.3,0, 0.8, 0.3))
                            /* int counts, float times, float z_h, float v0, float v1, float cx, float cy, float r_of_c, float err_max   */
                            /*设定的总圈数，设定的总时间，  设定高度 ，切向速度 ，纠正速度，圆心x坐标，圆心y坐标，  圆半径，    误差*/
                            /*
                            函数使用事项：
                            1.切向速度指无人机绕圆周转动速度。
                            2.纠正速度指无人机偏离圆周时将其拉回轨道的速度（理论上并不等于这个值，因为偏离圆周越远，这个速度就越大，但这个设定值能决定偏离速度随位置偏移的变化率
                            3.建议纠正速度 >= 切向速度
                            4.如果增大切向速度v，也应适度增大r_of_c
                            5.误差建议在0.3-0.5，但切记要保证误差不能大于 r_of_c,而且二者也不要太接近，这个误差最好不要设置太小，因为他不会影响飞行，单纯起到计数作用。
                            6.理论上一圈飞行时间为2PI*r/v, a = v^2/r,得到t = 2PI*v/a, 要保证时间尽量短而加速度尽量小（更稳定）的情况下，应同步减小v和r，即半径越小，速度适当减小，转的越快
                                当然，实际飞行考虑更多的因素，应结合实际条件多修改参数进行调整。
                            */
            {               
                Delay(DELAY);
            }
            break;

        case 6:
            if (collision_avoidance_mission(3.6, 1.6, ALTITUDE, 0, err_max))
            {
                Delay(2);
            }
            break;
        case 7:
            if (onFrame(0, err_max))
            {
                Delay(0.2);
            }
            break;
        case 8:
            if (collision_avoidance_mission(1.8, 1.6, ALTITUDE, 0, err_max))
            {
                Delay(2);
            }
            break;
        case 9:
            if (onFrame(0, err_max))
            {
                Delay(0.2);
            }
            break;
        case 10:
            if (collision_avoidance_mission(1.8, -1.6, ALTITUDE, 0, err_max))
            {
                Delay(2);
            }
            break;
        case 11:
            if (onFrame(0, err_max))
            {
                Delay(0.2);
            }
            break;
        case 12:
            if (collision_avoidance_mission(3.6, -1.6, ALTITUDE, 0, err_max))
            {
                Delay(2);
            }
            break;
        case 13:
            if (onFrame(0, err_max))
            {
                Delay(0.2);
            }
            break;
        case 14:
            if (collision_avoidance_mission(6.0, -2.5, ALTITUDE, 0, err_max))
            {
                Delay(DELAY);
            }
            break;

        case 15:
            if (collision_avoidance_mission(6.0,-2.5, 1.5, LEFT, err_max))
            {
                Delay(0.2);
            }
            break;
        case 16:
            if (cross_ring(6.0, 0.0, 1.5, LEFT, err_max))
            {
                Delay(0.2);
            }
            break;
        case 17:
            if (collision_avoidance_mission(6.0, 1.0, ALTITUDE, LEFT, err_max))
            {
                Delay(DELAY);
            }
            break;
        case 18:
            if (/*特殊靶识别并投标函数*/1)
            {
                Delay(DELAY);
            }
            break;
        case 19:
            if (collision_avoidance_mission(3.5, 1.0, ALTITUDE, LEFT, err_max))
            {
                Delay(DELAY);
            }
            break;
        case 20:
            if (collision_avoidance_mission(0, 1.6 * H_direction, ALTITUDE, LEFT, err_max))
            {
                Delay(0.2);
            }
            break;
        case 21:
            if (collision_avoidance_mission(0, 1.6 * H_direction, ALTITUDE, 0, err_max))
            {
                Delay(DELAY);
            }
            break;
            

            // case 11:
            //     if (collision_avoidance_mission(3.5 , -6.3 , ALTITUDE,0, err_max))
            //     {
            //         Delay(DELAY);
            //     }
            //     break;

            // case 12:
            //     if (mission_pos_cruise(3.5 , -6.3 , ALTITUDE, 0, err_max))
            //     {
            //         Delay(DELAY);
            //     }
            //     break;

            // case 13:
            //     if (collision_avoidance_mission(0 , 0 , ALTITUDE,0, err_max))
            //     {
            //         Delay(DELAY);
            //     }
            //     break;

            // case 14:
            //     if (mission_pos_cruise(0 , 0 , ALTITUDE,0, err_max))
            //     {
            //         Delay(DELAY);
            //     }
            //     break;

        case 22:
            if (precision_land())
            {
                mission_num = -1;
            }
            break;
        }
        mavros_setpoint_pos_pub.publish(setpoint_raw);
        ros::spinOnce();
        rate.sleep();

        if (mission_num == -1)
        {
            exit(0);
        }
    }
    return 0;
}
