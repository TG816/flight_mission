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
    
    cv::QRCodeDetector qrDecoder;
    std::vector<cv::Point2f> corners;
    QR_detected = qrDecoder.detect(rotated_frame, corners);
    
    if(QR_detected){
        ROS_INFO("QR detected!");
        detected = true;
    }
    else{
        // ========== 图像预处理优化 ==========
        cv::Mat gray, blurred, binary, morph_closed, morph_opened;
        
        // 1. 转换为灰度图
        cv::cvtColor(rotated_frame, gray, cv::COLOR_BGR2GRAY);
        
        // 2. 高斯模糊去噪
        cv::GaussianBlur(gray, blurred, cv::Size(5, 5), 0);
        
        // 3. 自适应阈值二值化（调整参数）
        cv::adaptiveThreshold(blurred, binary, 255,
                              cv::ADAPTIVE_THRESH_GAUSSIAN_C,
                              cv::THRESH_BINARY, 15, 8);
        
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
                             result.c_str(), result.length(), confidence);
                    
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
                }
            }
            delete[] outText;
        }
        else {
            ROS_WARN("OCR识别置信度过低: %d 或无结果", confidence);
        }
        
        // 注意：不调用 api->End() 或 delete api，保留对象以供下次使用
    }
}



//用于任务四，识别投放标识
void detect_circular(const cv::Mat &frame){
    cv::Mat gray;
    cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
     cv::GaussianBlur(gray, gray, cv::Size(5, 5), 0);
     cv::Mat binary;
    cv::adaptiveThreshold(gray, binary, 255, 
                          cv::ADAPTIVE_THRESH_GAUSSIAN_C, 
                          cv::THRESH_BINARY, 11, 2);
    vector<vector<cv::Point>> contours;
     cv::findContours(binary, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    cv::Point2f center(-1, -1);
    double maxCircularity = 0.8; // 圆形度阈值，越接近1越圆
    double maxArea = 0;           // 最大面积

    for (const auto& contour : contours) {
        // 计算轮廓的面积和周长
        double area = cv::contourArea(contour);
        double perimeter = cv::arcLength(contour, true);
    
        if (area < 10000) continue;
        
        // 计算圆度：4 * π * area / (perimeter^2)
        double circularity = 4 * M_PI * area / (perimeter * perimeter);
        
        // 筛选圆形轮廓（取最圆且面积最大的）
        if (circularity > maxCircularity && area > maxArea) {

            cv::Point2f circle_center;
            float radius;
            cv::minEnclosingCircle(contour, circle_center, radius);//求最小外接圆的函数，来获得中心坐标
            center = circle_center;circular_found=true;
            maxArea = area;
        }
    }
    if(circular_found)circular_center=center;
}

//用于任务四，巡航同时识别投放标识
bool cruise_finding_circular(float center_x,float center_y,float z,float target_yaw,float error_max,float a){

            if(detect_count_circular<4){detect_count_circular++;}
            else{detect_circular(current_frame);detect_count_circular=0;}

    if(!center_detected_circular&&!mission_pos_cruise(center_x,center_y,z,target_yaw,error_max)){
        center_detected_circular=true;return false;
    }
       else if(angle_cruise_circular<d_angle_circular*5.5&&!mission_pos_cruise(center_x+a*cos(angle_cruise),center_y+a*sin(angle_cruise),z,target_yaw,error_max)){
                 angle_cruise_circular+= d_angle_circular;return false;
        }
    
        else{return true;}
    
}


//用于任务五，当任务二识别出字母或数字时在任务五识别对应的数字或字母
bool detect_specific_char(const cv::Mat &frame ,const string& target_char,cv::Point2f &center){
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
        // 设置页面分割模式为自动（可根据需要调整）
        api->SetPageSegMode(tesseract::PSM_AUTO);
        init_ok = true;
    }
    if (!init_ok) {
        ROS_ERROR("Tesseract not available.");
        return false;
    }
    cv::Mat gray, processed;
    cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
    cv::GaussianBlur(gray, processed, cv::Size(3, 3), 0);
    cv::adaptiveThreshold(processed, processed, 255,cv::ADAPTIVE_THRESH_GAUSSIAN_C,cv::THRESH_BINARY_INV, 11, 2);
     // 设置字符白名单（只允许目标字符）
    api->SetVariable("tessedit_char_whitelist", target_char.c_str());
     api->SetImage(processed.data, processed.cols, processed.rows, 1, processed.cols);
     // 执行识别
    char* outText = api->GetUTF8Text();  // 文本结果（仅用于调试，可不使用）
    bool found = false;
    // 获取结果迭代器，遍历每个识别到的符号（字符级别）
    tesseract::ResultIterator* it = api->GetIterator();
    if (it) {
        do {
            const char* symbol = it->GetUTF8Text(tesseract::RIL_SYMBOL);
            if (symbol && symbol[0] == target_char[0]) { // 确保识别出的字符与目标匹配
                int x1, y1, x2, y2;
                it->BoundingBox(tesseract::RIL_SYMBOL, &x1, &y1, &x2, &y2);
                center.x = (x1 + x2) / 2.0f;
                center.y = (y1 + y2) / 2.0f;
                found = true;
                delete[] symbol;
                break;  // 只取第一个匹配的字符，可根据需要改为收集所有
            }
            delete[] symbol;
        } while (it->Next(tesseract::RIL_SYMBOL));
        delete it;
    }
    delete[] outText;
    api->Clear();  // 清除图像数据，准备下一次使用
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


//用于任务五，用目标中心与相机光轴的水平像素偏差计算移动速度，直到目标中心与光轴几乎水平重合
bool move_to_target(const cv::Mat &frame,float target_yaw){
    cv::Point2f center;
    float u=333.636;
    float v=0;
    if(QR_detected){
     if(detect_QR(frame,center)){
          ROS_INFO("检测到二维码");
        float dis=center.x-u;//当dis>0,图像中心偏右，无人机要右移
        if(fabs(dis)<=20){return true;}
        v=dis*0.005;
     }
     else{
        ROS_WARN("未检测到二维码");return false;
     }
    }
    else{
        if(detect_specific_char(frame,num_or_letter,center)){
             ROS_INFO("检测到%s",num_or_letter.c_str());
            float dis=center.x-u;
        if(fabs(dis)<=20){return true;}
        v=dis*0.005;
        }
        else{
        ROS_WARN("未检测到%s",num_or_letter.c_str());return false;
        }
    }

       setpoint_raw.type_mask = 1 + 2 + 4 /*+ 8 + 16 + 32*/ + 64 + 128 + 256 + 512 /*+ 1024 + 2048*/;
		setpoint_raw.coordinate_frame = 1;
		setpoint_raw.position.x = local_pos.pose.pose.position.x;
		setpoint_raw.position.y = local_pos.pose.pose.position.y;
		setpoint_raw.position.z = local_pos.pose.pose.position.z;

		// 速度字段（原有逻辑保留，已补全）
		setpoint_raw.velocity.x = 0.0f;
		setpoint_raw.velocity.y = v;
		setpoint_raw.velocity.z = 0.0f; // 保持高度

		// 必选：补全加速度字段（飞控要求，设为0即可）
		setpoint_raw.acceleration_or_force.x = 0.0f;
		setpoint_raw.acceleration_or_force.y = 0.0f;
		setpoint_raw.acceleration_or_force.z = 0.0f;

		// 必选：补全姿态字段（飞控要求，用当前yaw）
        setpoint_raw.yaw = target_yaw / 180.0 * M_PI;	  
		setpoint_raw.yaw_rate = 0.0f; // 偏航角速度（保持当前朝向）
        return false;
}