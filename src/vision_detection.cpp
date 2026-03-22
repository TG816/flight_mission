#include "vision_detection.h"
#include "flight_control.h"
#include "mission_header.h"



/************************************************************************
视觉相关函数实现
*************************************************************************/

//用于任务二识别目标
void detect_QR_num_letter(const cv::Mat &frame){
    cv::Mat rotated_frame;
    rotated_frame = frame.clone();  // 0 度直接复制
    
    
    
    
   
        // ========== 图像预处理优化 ==========
        cv::Mat gray, blurred, binary, morph_closed, morph_opened;
        
        // 1. 转换为灰度图
        cv::cvtColor(rotated_frame, gray, cv::COLOR_BGR2GRAY);
        
        // 2. 高斯模糊去噪
        cv::GaussianBlur(gray, blurred, cv::Size(5, 5), 0);
        
        // 3. 自动全局阈值二值化
       double auto_thresh = cv::threshold(blurred, binary, 0, 255, cv::THRESH_BINARY + cv::THRESH_OTSU);
        ROS_INFO("OTSU自动计算的最优阈值：%.1f", auto_thresh);
        // 4. 形态学闭运算 - 填充字符内部空洞
        cv::Mat kernel_close = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
        cv::morphologyEx(binary, morph_closed, cv::MORPH_CLOSE, kernel_close, cv::Point(-1,-1), 2);
        
        // 5. 形态学开运算 - 去除小噪声点
        cv::Mat kernel_open = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(2, 2));
        cv::morphologyEx(morph_closed, morph_opened, cv::MORPH_OPEN, kernel_open, cv::Point(-1,-1), 1);
        
        // 6. 可选：对比度增强（如果光照条件差）
        // cv::equalizeHist(morph_opened, morph_opened);
        
        // ========== Tesseract OCR 识别 ==========
        static tesseract::TessBaseAPI* api = nullptr;
        static bool init_ok = false;
        
        if (!api) {
            api = new tesseract::TessBaseAPI();
            if (api->Init(NULL, "eng") != 0) {
                ROS_ERROR("Tesseract initialization failed. Make sure 'tesseract-ocr-eng' is installed.");
                delete api;
                api = nullptr;
                init_ok = false;
                return;
            }
            // 设置为单字符识别模式
            api->SetPageSegMode(tesseract::PSM_SINGLE_CHAR);
            api->SetVariable("tessedit_char_whitelist", "0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ");
            init_ok = true;
        }
        
        if (!init_ok) {
            ROS_ERROR("Tesseract not available, skipping OCR.");
            return;
        }
        
        // 设置处理后的图像
        api->SetImage((uchar*)morph_opened.data, morph_opened.cols, morph_opened.rows, 1, morph_opened.cols);
        
        // ========== 置信度过滤 + 取最佳字符 ==========
        char* outText = api->GetUTF8Text();
        
        // 获取置信度信息
        int confidence = api->MeanTextConf();
        
        if (outText && confidence > 10) {  // 置信度阈值设为30
            std::string result = outText;
            
            // 去除空格、换行等空白字符
            result.erase(std::remove_if(result.begin(), result.end(), ::isspace), result.end());
            
            if (result.length() > 0) {
                // 方案1：如果识别出多个字符，取置信度最高的单个字符
                if (result.length() > 1) {
                    ROS_WARN("识别出多个字符: %s (长度: %d), 平均置信度: %d", 
                             result.c_str(),  (int)result.length(), confidence);
                    
                    // 使用结果迭代器获取每个字符的置信度
                    tesseract::ResultIterator* it = api->GetIterator();
                    if (it) {
                        int best_conf = 0;
                        char best_char = 0;
                        
                        do {
                            const char* symbol = it->GetUTF8Text(tesseract::RIL_SYMBOL);
                            int conf = it->Confidence(tesseract::RIL_SYMBOL);
                            
                            if (symbol && symbol[0] != '\0' && conf > best_conf) {
                                best_conf = conf;
                                best_char = symbol[0];
                                ROS_INFO("字符 '%c' 置信度: %d", best_char, best_conf);
                            }
                            if (symbol) delete[] symbol;
                        } while (it->Next(tesseract::RIL_SYMBOL));
                        
                        delete it;
                        
                        // 使用置信度最高的字符
                        if (best_char != 0 && best_conf > 30) {
                            cv::imwrite("/home/jetson/first_task_ws/src/flight_mission/data/binary_dimian.png",binary);
                             cv::imwrite("/home/jetson/first_task_ws/src/flight_mission/data/morph_dimian.png",morph_opened);
                            num_or_letter = std::string(1, best_char);
                            num_or_letter_detected = true;
                            detected = true;
                            ROS_INFO("最终识别结果: %s (置信度: %d)", num_or_letter.c_str(), best_conf);
                        }
                    }
                }
                else {
                    // 只有一个字符，直接使用
                    num_or_letter = result.substr(0, 1);
                    num_or_letter_detected = true;
                    detected = true;
                    ROS_INFO("识别结果: %s (置信度: %d)", num_or_letter.c_str(), confidence);
                      cv::imwrite("/home/jetson/first_task_ws/src/flight_mission/data/binary_dimian.png",binary);
                             cv::imwrite("/home/jetson/first_task_ws/src/flight_mission/data/morph_dimian.png",morph_opened);
                }
            }
            delete[] outText;return;
        }
        else {
            ROS_WARN("OCR识别置信度过低: %d 或无结果", confidence);
        }
        
        // 注意：不调用 api->End() 或 delete api，保留对象以供下次使用
    
    cv::QRCodeDetector qrDecoder;
    std::vector<cv::Point2f> corners;
    std::string decoded_info = qrDecoder.detectAndDecode(frame, corners);
    QR_detected = !decoded_info.empty();
    if(QR_detected){
         if(decoded_info.empty()){
        ROS_WARN("二维码解码为空，视为误检");
        detected = false; QR_detected=false; // 当作没检测到
        }
        else{
        ROS_INFO("QR detected!");
        detected = true;
        }
    }
}


//用于任务四，识别投放标识
void detect_circular(const cv::Mat &frame){
    cv::Mat gray, blurred, binary, morph_closed, morph_opened;
    cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);

     // 双边滤波去噪（优化点：替代高斯模糊，保留边缘）
     cv::bilateralFilter(gray, blurred, 9, 75, 75);
     
     cv::adaptiveThreshold(blurred, binary, 255,
                          cv::ADAPTIVE_THRESH_GAUSSIAN_C,
                          cv::THRESH_BINARY, 15, 8);

     // 形态学闭运算 - 填充圆形内部空洞（与字母检测一致）
    cv::Mat kernel_close = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
    cv::morphologyEx(binary, morph_closed, cv::MORPH_CLOSE, kernel_close, cv::Point(-1,-1), 2);

     //  形态学开运算 - 去除小噪声点（与字母检测一致）
    cv::Mat kernel_open = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(2, 2));
    cv::morphologyEx(morph_closed, morph_opened, cv::MORPH_OPEN, kernel_open, cv::Point(-1,-1), 1);


    vector<vector<cv::Point>> contours;
     cv::findContours(morph_opened, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    cv::Point2f center(-1, -1);
    double maxCircularity = 0.5; // 圆形度阈值，越接近1越圆
    double maxArea = 0;           // 最大面积
int i=0;
    for (const auto& contour : contours) {
        i++;
        // 计算轮廓的面积和周长
        double area = cv::contourArea(contour);
        double perimeter = cv::arcLength(contour, true);
    
        if (area < 10000) {ROS_INFO("轮廓%d:area太小，area:%.2f",i,area); continue;}
        
        // 计算圆度：4 * π * area / (perimeter^2)
        double circularity = 4 * M_PI * area / (perimeter * perimeter);
        ROS_INFO("轮廓%d:area:%.2f, 圆度:%.2f",i,area,circularity);
        // 筛选圆形轮廓（取最圆且面积最大的）
        if (circularity > maxCircularity && area > maxArea) {

            cv::Point2f circle_center;
            float radius;
            cv::minEnclosingCircle(contour, circle_center, radius);//求最小外接圆的函数，来获得中心坐标
            center = circle_center;circular_found=true;
            maxArea = area;maxCircularity=circularity;
        }
    }
    if(circular_found){circular_center=center;ROS_INFO("max_area:%.2f,max_circularity:%.2f",maxArea,maxCircularity);}
}

//用于任务二，绕半径为a的粗略圆形飞行一周，同时找目标
bool cruise_finding(float center_x,float center_y,float z,float target_yaw,float error_max,float a){

    if(detect_count<4){detect_count++;}
    else{detect_QR_num_letter(current_frame);detect_count=0;}
    if(!center_detected&&mission_pos_cruise(center_x,center_y,z,target_yaw,error_max)){
     ROS_WARN("到达巡航中心点"); detect_QR_num_letter(current_frame); center_detected=true;
    }

    else if(center_detected&&angle_cruise<d_angle*5.5&&mission_pos_cruise(center_x+a*cos(angle_cruise),center_y+a*sin(angle_cruise),z,target_yaw,error_max)){
           ROS_WARN("到达目标点%d",aim_num);
           detect_QR_num_letter(current_frame);
           aim_num++;     
        angle_cruise+=d_angle;
    }
    
    else if(angle_cruise>d_angle*5.5){return true;}//检查完所有待检测点
    return false;
}




//用于任务四，巡航同时识别投放标识
bool cruise_finding_circular(float center_x,float center_y,float z,float target_yaw,float error_max,float a){

            if(detect_count_circular<4){detect_count_circular++;}
            else{detect_circular(current_frame);detect_count_circular=0;}

    if(!center_detected_circular&&mission_pos_cruise(center_x,center_y,z,target_yaw,error_max)){
      ROS_WARN("到达巡航中心点");   center_detected_circular=true;
    }
       else if(center_detected_circular&&angle_cruise_circular<d_angle_circular*5.5&&mission_pos_cruise(center_x+a*cos(angle_cruise),center_y+a*sin(angle_cruise),z,target_yaw,error_max)){
          ROS_WARN("到达目标点%d",aim_num_circular);aim_num_circular++;              
        angle_cruise_circular+= d_angle_circular;
        }
        else if(angle_cruise_circular>d_angle_circular*5.5){return true;}
 return false;   
}




//用于任务五，当任务二识别出字母或数字时在任务五识别对应的数字或字母
//用于任务五，当任务二识别出字母或数字时在任务五识别对应的数字或字母
// 修改：支持多个相同字符检测，返回置信度最高的那个
bool detect_specific_char(const cv::Mat &frame, const string& target_char, cv::Point2f &center){
    static tesseract::TessBaseAPI* api = nullptr;
    static bool init_ok = false;
    
    if (!api) {
        api = new tesseract::TessBaseAPI();
        if (api->Init(NULL, "eng") != 0) {
            ROS_ERROR("Tesseract initialization failed.");
            delete api;
            api = nullptr;
            init_ok = false;
            return false;
        }
        api->SetPageSegMode(tesseract::PSM_SINGLE_CHAR);
        init_ok = true;
    }
    
    if (!init_ok) {
        ROS_ERROR("Tesseract not available.");
        return false;
    }
    
    cv::Mat gray, processed,binary_contour,  morph_result;
    cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
    
    // ========== 图像预处理（优化小字符识别）==========
    cv::bilateralFilter(gray, processed, 5, 50, 50);  // 双边滤波保留边缘
    
    double auto_thresh = cv::threshold(processed, binary_contour, 0, 255, cv::THRESH_BINARY_INV + cv::THRESH_OTSU);
    
    morph_result = binary_contour.clone();
    
     std::vector<std::vector<cv::Point>> contours;
    cv::findContours(morph_result.clone(), contours, 
                     cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        // 轮廓过滤参数
    const int MIN_CONTOUR_AREA = 1000;
    const int MAX_CONTOUR_AREA = 50000;
    const float MIN_ASPECT_RATIO = 0.3;
    const float MAX_ASPECT_RATIO = 2.0;
    const int MIN_CONFIDENCE = 30;

    
    // ========== 收集所有匹配的目标字符 ==========
    bool found = false;
    int best_confidence = 0;
    cv::Point2f best_center(-1, -1);
      cv::Rect best_rect;
    int candidate_count = 0;
    
    // 获取结果迭代器，遍历每个识别到的符号
     for (size_t i = 0; i < contours.size(); i++) {
     cv::Rect rect = cv::boundingRect(contours[i]);
    int x = rect.x;
        int y = rect.y;
        int width = rect.width;
        int height = rect.height;
        int area = rect.width * rect.height;
           if (area < MIN_CONTOUR_AREA ) {
            ROS_WARN("过滤轮廓%lu: 面积=%d", i, area);
            continue;
        }
                float img_ratio_w = (float)width / frame.cols;
        float img_ratio_h = (float)height / frame.rows;
    
         candidate_count++;
         int roi_x = std::max(0, x - 10);
        int roi_y = std::max(0, y - 10);
        int roi_w = std::min((int)frame.cols - roi_x, width + 20);
        int roi_h = std::min((int)frame.rows - roi_y, height + 20);
        cv::Mat char_roi = frame(cv::Rect(roi_x, roi_y, roi_w, roi_h));
       cv::Mat binary_ocr_roi = binary_contour(cv::Rect(roi_x, roi_y, roi_w, roi_h));
       cv::Mat binary_ocr_inverted;
        cv::bitwise_not(binary_ocr_roi, binary_ocr_inverted);  // ← 反转给 Tesseract


        //  确保图像连续
        cv::Mat binary_ocr_inverted_cont;
        binary_ocr_inverted.copyTo(binary_ocr_inverted_cont);
       cv::Mat ocr_closed,ocr_opened;
        cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
cv::morphologyEx(binary_ocr_inverted_cont,  ocr_closed, cv::MORPH_CLOSE, kernel, cv::Point(-1,-1), 2);
 cv::morphologyEx(ocr_closed, ocr_opened, cv::MORPH_OPEN, kernel, cv::Point(-1,-1), 1);
        // Tesseract 识别
        api->SetVariable("tessedit_char_whitelist", target_char.c_str());
        api->SetImage((uchar*)ocr_opened.data, 
                      ocr_opened.cols, 
                      ocr_opened.rows, 
                      1, 
                      ocr_opened.step);  // 使用 step

     int conf = api->MeanTextConf();
     char* outText = api->GetUTF8Text();
std::string recognized_text;
if (outText) {
    recognized_text = std::string(outText);
    delete[] outText;
} else {
    recognized_text = "";
}
         

   bool char_matched = false;
for (char c : recognized_text) {
            if (c == target_char[0]) {
                char_matched = true;
                break;
            }
        }
if (char_matched && conf >= MIN_CONFIDENCE) {
            if (conf > best_confidence) {
                best_confidence = conf;
                best_center.x = x + width / 2.0f;
                best_center.y = y + height / 2.0f;
                best_rect = rect;
                found = true;
                
                std::string roi_path = "/home/jetson/first_task_ws/src/flight_mission/data/char_roi.png";
                cv::imwrite(roi_path, char_roi);
                
                std::string opened_path = "/home/jetson/first_task_ws/src/flight_mission/data/opened_roi.png";
                cv::imwrite(opened_path, ocr_opened);
                
                ROS_INFO("更新最佳候选：置信度=%d", conf);
            }
        }
      
         api->Clear();  
}


    // ========== 输出结果 ==========
    if (found) {
        cv::imwrite("/home/jetson/first_task_ws/src/flight_mission/data/morph_result.png", morph_result);
cv::imwrite("/home/jetson/first_task_ws/src/flight_mission/data/processed.png", processed);
        center = best_center;
        
            ROS_INFO("检测到 '%c' (置信度：%d)，位置：(%.1f, %.1f)", 
                    target_char[0], best_confidence, center.x, center.y);
        
    } else {
        ROS_WARN("未检测到目标字符 '%s'", target_char.c_str());
    }
    
    return found;
}


//用于任务五，当任务二识别出二维码时在任务五识别二维码
bool detect_QR(const cv::Mat &frame,cv::Point2f &center){
        cv::QRCodeDetector qrDecoder;
    std::vector<cv::Point2f> corners;
    bool q_detected = qrDecoder.detect(frame, corners);
    if (q_detected && corners.size() >= 4)
	{
		// 计算四个角点的中心
		for (int i = 0; i < 4; i++)
		{
			center.x += corners[i].x;
			center.y += corners[i].y;
		}
		center.x = center.x / 4.0;
		center.y = center.y / 4.0;
		ROS_INFO("find QR center in camera: (%.1f, %.1f)", center.x, center.y);
        return true;
	}
    else{return false;}
}



bool detect_aim(const cv::Mat &frame,int& idx){
       if (frame.empty()) {
    ROS_WARN("前置相机图像为空");
    return false;
     }
   bool flag=false;
   cv::Point2f center;
if(QR_detected){
     if(detect_QR(frame,center)){
          ROS_INFO("检测到二维码");flag=true;
     }
     else{
        ROS_WARN("未检测到二维码");
     }
    }
    else{
        if(detect_specific_char(frame,num_or_letter,center)){
             ROS_INFO("检测到%s",num_or_letter.c_str());flag=true;
        }
        else{
        ROS_WARN("未检测到%s",num_or_letter.c_str());
        }
    }
    if(flag){
        ROS_INFO("字符像素x坐标:%.2f",center.x);
        if(center.x<700){idx=0;}
        else if(center.x>1100){idx=2;}
        else{idx=1;}
    }
    return flag;
}

// //u根据相机内参而修改
// //用于任务五，用目标中心与相机光轴的水平像素偏差计算移动速度，直到目标中心与光轴几乎水平重合
// bool move_to_target(const cv::Mat &frame,float target_yaw){
//     if (frame.empty()) {
//     ROS_WARN("前置相机图像为空");
//     return false;
//      }
//     cv::Point2f center;
//     float u=960;//真实测的是333.636，gazebo是960
//     float v=0;
//      float dis=0;
//     if(QR_detected){
//      if(detect_QR(frame,center)){
//           ROS_INFO("检测到二维码");
//         dis=center.x-u;//当dis>0,图像中心偏右，无人机要右移
   
//         v=dis*0.005;
       
//      }
//      else{
//         ROS_WARN("未检测到二维码");dis=100;v=0;
//      }
//     }
//     else{
//         if(detect_specific_char(frame,num_or_letter,center)){
//              ROS_INFO("检测到%s",num_or_letter.c_str());
//            dis=center.x-u;

//         v=dis*0.005; 
       
//         }
//         else{
//         ROS_WARN("未检测到%s",num_or_letter.c_str());dis=100;v=0;
//         }
//     }

//             if(fabs(dis)<=20){
//              ROS_WARN("正对目标点,dis:%.2f",dis);
//              v=0;
//            }
//            else{ ROS_WARN("dis:%.2f,  v:%.2f",dis,v);}
//        setpoint_raw.type_mask = 1 + 2 + 4 /*+ 8 + 16 + 32*/ + 64 + 128 + 256 + 512 /*+ 1024 + 2048*/;
// 		setpoint_raw.coordinate_frame = 1;
// 		setpoint_raw.position.x = local_pos.pose.pose.position.x;
// 		setpoint_raw.position.y = local_pos.pose.pose.position.y;
// 		setpoint_raw.position.z = local_pos.pose.pose.position.z;

// 		// 速度字段（原有逻辑保留，已补全）
// 		setpoint_raw.velocity.x = 0.0f;
// 		setpoint_raw.velocity.y = v;
// 		setpoint_raw.velocity.z = 0.0f; // 保持高度

// 		// 必选：补全加速度字段（飞控要求，设为0即可）
// 		setpoint_raw.acceleration_or_force.x = 0.0f;
// 		setpoint_raw.acceleration_or_force.y = 0.0f;
// 		setpoint_raw.acceleration_or_force.z = 0.0f;

// 		// 必选：补全姿态字段（飞控要求，用当前yaw）
//         setpoint_raw.yaw = target_yaw / 180.0 * M_PI;	  
// 		setpoint_raw.yaw_rate = 0.0f; // 偏航角速度（保持当前朝向）
//         if(fabs(dis)<=20){
//              return true;
//            }
//            else{return false;}
        
// }


