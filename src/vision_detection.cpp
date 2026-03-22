#include "vision_detection.h"
#include "flight_control.h"
#include "mission_header.h"



/************************************************************************
视觉相关函数实现
*************************************************************************/



// -------------------------- 独立实现 onFrame 函数 --------------------------
bool onFrame(float t_yaw, double err_max) {
    // 1. 打印帧处理日志
    ROS_INFO("[onFrame] 开始处理图像帧 | 目标偏航：%.2f | 最大误差：%.2f", t_yaw, err_max);
    
    // 2. 调用无人机核心检测函数，获取自定义检测结果
    UavDetectResult det_res = detectUavTarget();
    
    // 3. 检测失败处理：返回true（保留原有动态调整逻辑入口）
    if (!det_res.is_detected || det_res.confidence < CONF_THRESHOLD) {
        std::string fail_reason = det_res.is_detected ? "置信度低于阈值" : "未检测到靶子";
        ROS_WARN("[onFrame] 目标识别失败 | 原因：%s | 图像帧为空：%s | 置信度：%.2f (阈值：%.2f)",
                 fail_reason.c_str(),
                 current_frame.empty() ? "是" : "否",
                 det_res.confidence, CONF_THRESHOLD);
        
        // 动态调整逻辑示例（可按需扩展）：
        // - 调整颜色阈值/搜索范围
        // - 重试检测
        // - 发送飞控指令调整无人机姿态
        return true;
    }

    // 4. 检测成功：判断是否为二维码目标类别
    bool is_target_class = std::find(
        g_qrcode_classes.begin(), 
        g_qrcode_classes.end(), 
        det_res.class_name  // 修复：target_class → class_name
    ) != g_qrcode_classes.end();


    // 5. 非目标类别：打印日志并返回true
    if (!is_target_class) {
        ROS_INFO("[onFrame] 识别成功但非二维码目标 | 检测类别：%s | 目标列表包含类别数：%lu",
                 det_res.class_name.c_str(), // 修复：target_class → class_name
                 g_qrcode_classes.size());
        return true;
    }

    // 6. 是目标类别：计算位置偏移
    float target_x = static_cast<float>(det_res.gray_ring_center.x); // 修复：target_center → gray_ring_center
    float target_y = static_cast<float>(det_res.gray_ring_center.y); // 修复：target_center → gray_ring_center
    float drone_x = static_cast<float>(local_pos.pose.pose.position.x);
    float drone_y = static_cast<float>(local_pos.pose.pose.position.y);
    

     geometry_msgs::Point target_w = change_to_world(target_x,target_y);

    // 7. （传入靶子中心坐标）
    throw_pos = {target_w.x, target_w.y};
    isThrow = true;

    float dx = throw_pos.x - drone_x;
    float dy = throw_pos.x - drone_y;

    ROS_INFO("[onFrame] 识别到二维码目标 | 类别：%s | 靶子中心：(%.1f,%.1f) | 无人机位置：(%.1f,%.1f) | 偏移(x/y)：%.2f/%.2f",
             det_res.class_name.c_str(), // 修复：target_class → class_name
             throw_pos.x,throw_pos.y,
             drone_x, drone_y,
             dx, dy);

    return true;
}

bool detectGrayRingAndThrow(float throw_yaw, double err_max) {
    // 1. 输入校验：图像帧为空直接返回失败
    if (current_frame.empty()) {
        ROS_ERROR("[detectGrayRingAndThrow] 输入图像帧为空，灰环检测失败");
        return false;
    }

    // 2. 核心：仅检测灰色圆环（复用原有灰环检测逻辑）
    cv::Point gray_center;   // 灰环中心坐标
    cv::Rect gray_rect;      // 灰环包围矩形
    int gray_radius = 0;     // 灰环半径
    if (!findGrayRingCenter(current_frame, gray_center, gray_rect, gray_radius)) {
        ROS_WARN("[detectGrayRingAndThrow] 未检测到灰色圆环，投掷失败");
        return false;
    }

    // 4. 转换坐标（像素坐标→无人机世界坐标，适配throwObject入参）
    geometry_msgs::Point target_pos = change_to_world(gray_center.x, gray_center.y);

    // 5. （传入灰环中心+偏航角+误差阈值）
    throw_pos = {target_pos.x, target_pos.y};
    isThrow = true;

    ROS_INFO("[detectGrayRingAndThrow] 调用投掷函数 | 靶标位置：(%.2f,%.2f) | 目标偏航：%.2f | 误差阈值：%.2f",
             throw_pos.x,throw_pos.y, throw_yaw, err_max);

    // 6. 返回成功
    return true;
}



// -------------------------- 工具函数 --------------------------
cv::Mat getColorMask(const cv::Mat& frame, const cv::Scalar& low, const cv::Scalar& high) {
    cv::Mat hsv, mask;
    cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);
    cv::inRange(hsv, low, high, mask);
    
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, MORPHO_KERNEL);
    cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, kernel);
    cv::morphologyEx(mask, mask, cv::MORPH_OPEN, kernel);
    return mask;
}

bool findGrayRingCenter(const cv::Mat& frame, cv::Point& center, cv::Rect& gray_rect, int& radius) {
    cv::Mat gray_mask = getColorMask(frame, GRAY_LOW, GRAY_HIGH);
    
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(gray_mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    if (contours.empty()) return false;

    int max_idx = 0;
    double max_area = 0.0;
    for (int i = 0; i < contours.size(); ++i) {
        double area = cv::contourArea(contours[i]);
        if (area > max_area) {
            max_area = area;
            max_idx = i;
        }
    }
    std::vector<cv::Point> max_gray_cnt = contours[max_idx];

    cv::Moments m = cv::moments(max_gray_cnt);
    if (m.m00 < 1e-6) return false;
    center.x = static_cast<int>(m.m10 / m.m00);
    center.y = static_cast<int>(m.m01 / m.m00);

    gray_rect = cv::boundingRect(max_gray_cnt);
    radius = std::max(gray_rect.width, gray_rect.height) / 2;

    return true;
}

bool findBlackSquareAroundCenter(const cv::Mat& frame, const cv::Point& gray_center, int search_radius,
                                cv::Rect& black_square, float& angle) {
    int h = frame.rows;
    int w = frame.cols;

    int search_x1 = std::max(0, gray_center.x - search_radius);
    int search_y1 = std::max(0, gray_center.y - search_radius);
    int search_x2 = std::min(w, gray_center.x + search_radius);
    int search_y2 = std::min(h, gray_center.y + search_radius);
    cv::Mat search_roi = frame(cv::Rect(search_x1, search_y1, search_x2 - search_x1, search_y2 - search_y1));
    if (search_roi.empty()) return false;

    cv::Mat black_mask = getColorMask(search_roi, BLACK_LOW, BLACK_HIGH);
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(black_mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    if (contours.empty()) return false;

    int max_idx = 0;
    double max_area = 0.0;
    for (int i = 0; i < contours.size(); ++i) {
        double area = cv::contourArea(contours[i]);
        if (area > max_area) {
            max_area = area;
            max_idx = i;
        }
    }
    std::vector<cv::Point> max_black_cnt = contours[max_idx];

    std::vector<cv::Point> hull;
    cv::convexHull(max_black_cnt, hull);
    double epsilon = APPROX_EPSILON * cv::arcLength(hull, true);
    std::vector<cv::Point> approx;
    cv::approxPolyDP(hull, approx, epsilon, true);

    std::vector<cv::Point> approx_abs;
    for (const auto& p : approx) {
        approx_abs.emplace_back(p.x + search_x1, p.y + search_y1);
    }

    int x1 = search_x1, y1 = search_y1, x2 = search_x2, y2 = search_y2;
    if (!approx_abs.empty()) {
        x1 = std::min_element(approx_abs.begin(), approx_abs.end(), 
            [](const cv::Point& a, const cv::Point& b) { return a.x < b.x; })->x;
        y1 = std::min_element(approx_abs.begin(), approx_abs.end(), 
            [](const cv::Point& a, const cv::Point& b) { return a.y < b.y; })->y;
        x2 = std::max_element(approx_abs.begin(), approx_abs.end(), 
            [](const cv::Point& a, const cv::Point& b) { return a.x < b.x; })->x;
        y2 = std::max_element(approx_abs.begin(), approx_abs.end(), 
            [](const cv::Point& a, const cv::Point& b) { return a.y < b.y; })->y;
    }
    black_square = cv::Rect(x1, y1, x2 - x1, y2 - y1);

    angle = 0.0f;
    if (approx.size() >= 4) {
        cv::Point p1 = approx[0];
        cv::Point p2 = approx[1];
        float dx = p2.x - p1.x;
        float dy = p2.y - p1.y;
        angle = atan2(dy, dx) * 180.0f / CV_PI;
    }

    return true;
}

bool findInnerImageRect(const cv::Mat& frame, const cv::Point& gray_center, const cv::Rect& black_square,
                       cv::Rect& inner_img_rect) {
    if (black_square.empty()) return false;

    cv::Mat square_roi = frame(black_square);
    cv::Mat white_mask = getColorMask(square_roi, WHITE_LOW, WHITE_HIGH);
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(white_mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    if (contours.empty()) {
        int img_size = static_cast<int>(black_square.width * INNER_IMG_SCALE);
        inner_img_rect = cv::Rect(
            std::max(0, gray_center.x - img_size / 2),
            std::max(0, gray_center.y - img_size / 2),
            img_size,
            img_size
        );
        inner_img_rect &= cv::Rect(0, 0, frame.cols, frame.rows);
        return true;
    }

    int max_idx = 0;
    double max_area = 0.0;
    for (int i = 0; i < contours.size(); ++i) {
        double area = cv::contourArea(contours[i]);
        if (area > max_area) {
            max_area = area;
            max_idx = i;
        }
    }
    cv::Rect white_rect = cv::boundingRect(contours[max_idx]);
    cv::Rect white_rect_abs = cv::Rect(
        black_square.x + white_rect.x,
        black_square.y + white_rect.y,
        white_rect.width,
        white_rect.height
    );
    int wc_x = white_rect_abs.x + white_rect_abs.width / 2;
    int wc_y = white_rect_abs.y + white_rect_abs.height / 2;
    int img_w = static_cast<int>(white_rect_abs.width * INNER_IMG_SCALE);
    int img_h = static_cast<int>(white_rect_abs.height * INNER_IMG_SCALE);
    inner_img_rect = cv::Rect(
        std::max(0, wc_x - img_w / 2),
        std::max(0, wc_y - img_h / 2),
        img_w,
        img_h
    );
    inner_img_rect &= cv::Rect(0, 0, frame.cols, frame.rows);

    return true;
}

// -------------------------- 适配后的ONNX分类函数 --------------------------
bool preciseClassify(const cv::Mat& frame, const cv::Rect& center_rect, 
                     std::string& cls_name, float& conf) {
    ROS_INFO("[preciseClassify] 开始精确分类 | 中心区域位置：(%d,%d) 尺寸：%dx%d | 置信度阈值：%f",
             center_rect.x, center_rect.y, center_rect.width, center_rect.height, CONF_THRESHOLD);
    if (center_rect.empty()) {
        ROS_WARN("[preciseClassify] 中心区域为空，分类失败");
        return false;
    }
    cv::Mat roi = frame(center_rect & cv::Rect(0,0,frame.cols,frame.rows));
    ROS_INFO("[preciseClassify] 截取ROI | ROI最终尺寸：%dx%d", roi.cols, roi.rows);
    
    // 预处理+ONNX推理
    cv::Mat resized, rgb;
    cv::resize(roi, resized, cv::Size(32,32));
    cv::cvtColor(resized, rgb, cv::COLOR_BGR2RGB);
    cv::Mat float_img;
    rgb.convertTo(float_img, CV_32F, 1.0/255.0);
    
    double min_val, max_val;
    cv::minMaxLoc(float_img, &min_val, &max_val);
    ROS_DEBUG("[preciseClassify] 预处理完成 | 归一化后图像范围：%.2f~%.2f", min_val, max_val);

    std::vector<float> input_data(3*32*32);
    int idx = 0;
    for (int c=0; c<3; c++) {
        for (int h=0; h<32; h++) {
            for (int w=0; w<32; w++) {
                input_data[idx++] = float_img.at<cv::Vec3f>(h,w)[c];
            }
        }
    }

    try {
        static Ort::Env env(ORT_LOGGING_LEVEL_WARNING, "CIFAR100");
        static Ort::SessionOptions sess_opt;
        static Ort::Session sess(env, ONNX_MODEL_PATH.c_str(), sess_opt);
        
        // 适配GPU/CPU
        if (USE_GPU) {
            sess_opt.AppendExecutionProvider_CUDA(OrtCUDAProviderOptions{});
            ROS_INFO("[preciseClassify] 使用GPU推理 | ONNX模型路径：%s", ONNX_MODEL_PATH.c_str());
        } else {
            ROS_INFO("[preciseClassify] 使用CPU推理 | ONNX模型路径：%s", ONNX_MODEL_PATH.c_str());
        }
        
        // 获取输入输出名
        auto in_name_allocated = sess.GetInputNameAllocated(0, Ort::AllocatorWithDefaultOptions());
        auto out_name_allocated = sess.GetOutputNameAllocated(0, Ort::AllocatorWithDefaultOptions());
        const char* in_name = in_name_allocated.get();
        const char* out_name = out_name_allocated.get();
        ROS_DEBUG("[preciseClassify] ONNX输入名：%s | 输出名：%s", in_name, out_name);
        
        std::vector<int64_t> shape = {1,3,32,32};
        auto tensor = Ort::Value::CreateTensor<float>(Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeCPU),
            input_data.data(), input_data.size(), shape.data(), shape.size());
        
        auto outputs = sess.Run({}, &in_name, &tensor, 1, &out_name, 1);
        
        float* out_data = outputs[0].GetTensorMutableData<float>();
        int top_idx = std::max_element(out_data, out_data+CIFAR100_CLASSES.size()) - out_data;
        conf = out_data[top_idx];
        cls_name = CIFAR100_CLASSES[top_idx];
        
        ROS_INFO("[preciseClassify] 分类完成 | 最高置信度类别：%s | 置信度：%f | 索引：%d",
                 cls_name.c_str(), conf, top_idx);
        return conf >= CONF_THRESHOLD;
    } catch (const std::exception& e) {
        ROS_ERROR("[preciseClassify] ONNX推理异常：%s", e.what());
        return false;
    } catch (...) {
        ROS_ERROR("[preciseClassify] ONNX推理未知异常");
        return false;
    }
}

// -------------------------- 核心检测函数（集成分类） --------------------------
UavDetectResult detectUavTarget() {
    UavDetectResult result;
    if (current_frame.empty()) return result;

    // 步骤1：找灰色圆环
    cv::Point gray_center;
    cv::Rect gray_rect;
    int gray_radius = 0;
    if (!findGrayRingCenter(current_frame, gray_center, gray_rect, gray_radius)) {
        return result;
    }

    // 步骤2：找黑色正方形
    cv::Rect black_square;
    float square_angle = 0.0f;
    int search_radius = static_cast<int>(gray_radius * SEARCH_RADIUS_SCALE);
    if (!findBlackSquareAroundCenter(current_frame, gray_center, search_radius, black_square, square_angle)) {
        return result;
    }

    // 步骤3：定位中心图片
    cv::Rect inner_img_rect;
    if (!findInnerImageRect(current_frame, gray_center, black_square, inner_img_rect)) {
        return result;
    }

    // 步骤4：调用ONNX分类函数
    std::string cls_name;
    float conf = 0.0f;
    if (preciseClassify(current_frame, inner_img_rect, cls_name, conf)) {
        result.class_name = cls_name;
        result.confidence = conf;
    }

    // 填充结果
    result.is_detected = true;
    result.gray_ring_center = gray_center;
    result.black_square = black_square;
    result.square_angle = square_angle;
    result.inner_image_rect = inner_img_rect;

    return result;
}


/**
 * @brief 极简二维码识别+信息提取（格式确定，无冗余校验）
 * @param frame 输入图像（BGR格式）
 * @return bool 是否成功识别二维码
 * @note 二维码内容格式：英文逗号分隔的类别列表（如 "apple,man,left"）
 */
bool detectQRCodeAndExtractInfo() {
    ROS_INFO("[detectQRCodeAndExtractInfo] 开始二维码识别");
    // 1. 初始化检测器+解码二维码
    cv::Mat &frame = current_frame;
    std::string qr_text = decodeQRCode(frame);
    if (qr_text.empty()) {
        ROS_WARN("[detectQRCodeAndExtractInfo] 二维码解码失败 | 解码结果为空");
        H_direction = 0;          // 未识别到，重置方向
        g_qrcode_classes.clear(); // 清空类别
        return false;
    }
    ROS_INFO("[detectQRCodeAndExtractInfo] 二维码解码成功 | 原始文本：%s", qr_text.c_str());

    // 2. 直接分割qr_text，提取前两个+最后一个单词（核心简化逻辑）
    g_qrcode_classes.clear();
    H_direction = 0;
    int comma_count = 0;          // 记录逗号数量，定位前两个/最后一个单词
    std::string temp;             // 临时存储单个单词
    ROS_DEBUG("[detectQRCodeAndExtractInfo] 开始解析二维码文本 | 文本长度：%lu", qr_text.length());
    
    for (char c : qr_text) {
        if (c == ',') {
            // 遇到逗号：先把当前temp存入列表（仅前两个）
            if (comma_count < 2 && !temp.empty()) {
                g_qrcode_classes.push_back(temp);
                ROS_DEBUG("[detectQRCodeAndExtractInfo] 提取第%d个类别：%s", comma_count+1, temp.c_str());
            }
            temp.clear();
            comma_count++;
        } else {
            // 非逗号：拼接字符成单词（自动过滤空格）
            if (c != ' ') temp += c; // 可选：过滤空格，兼容"man, apple ,left"格式
        }
    }

    // 3. 处理最后一个单词（循环结束后temp就是最后一个）
    if (!temp.empty()) {
        ROS_DEBUG("[detectQRCodeAndExtractInfo] 提取方向信息：%s", temp.c_str());
        if (temp == "left") {
            H_direction = 1;
            ROS_INFO("[detectQRCodeAndExtractInfo] 识别到左方向 | H_direction = 1");
        } else if (temp == "right") {
            H_direction = -1;
            ROS_INFO("[detectQRCodeAndExtractInfo] 识别到右方向 | H_direction = -1");
        } else {
            ROS_WARN("[detectQRCodeAndExtractInfo] 方向信息无效 | 内容：%s", temp.c_str());
        }
    } else {
        ROS_WARN("[detectQRCodeAndExtractInfo] 未提取到方向信息 | 最后一个单词为空");
    }

    ROS_INFO("[detectQRCodeAndExtractInfo] 二维码解析完成 | 原始文本：%s | 解析出%d个目标类别",
             qr_text.c_str(), (int)g_qrcode_classes.size());

    // 第二步：逐条输出每个元素（带索引，便于定位）
    if (g_qrcode_classes.empty())
    {
        ROS_WARN("[detectQRCodeAndExtractInfo] 解析结果为空（无有效目标类别）");
    }
    else
    {
        for (size_t i = 0; i < g_qrcode_classes.size(); ++i)
        {
            ROS_INFO("[detectQRCodeAndExtractInfo] 第%d个目标类别：%s", (int)(i + 1), g_qrcode_classes[i].c_str());
        }
    }
    return true;
}

std::string decodeQRCode(const cv::Mat& frame) {
    ROS_INFO("[decodeQRCode] 开始QR码解码 | 输入图像尺寸：%dx%d | 通道数：%d",
             frame.cols, frame.rows, frame.channels());
    // 步骤1：转灰度图（quirc 只处理灰度图）
    cv::Mat gray;
    if (frame.channels() == 3) {
        cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
        ROS_DEBUG("[decodeQRCode] 转换为灰度图完成");
    } else {
        gray = frame.clone();
        ROS_DEBUG("[decodeQRCode] 输入已为灰度图，直接克隆");
    }

    // 步骤2：初始化 quirc 上下文
    struct quirc* q = quirc_new();
    if (!q) {
        ROS_ERROR("[decodeQRCode] quirc上下文初始化失败");
        return "";
    }
    ROS_DEBUG("[decodeQRCode] quirc上下文初始化成功");

    // 步骤3：设置图像尺寸
    if (quirc_resize(q, gray.cols, gray.rows) < 0) {
        ROS_ERROR("[decodeQRCode] quirc设置图像尺寸失败 | 目标尺寸：%dx%d", gray.cols, gray.rows);
        quirc_destroy(q);
        return "";
    }
    ROS_DEBUG("[decodeQRCode] quirc设置图像尺寸成功 | %dx%d", gray.cols, gray.rows);

    // 步骤4：把 OpenCV 图像数据拷贝到 quirc 缓冲区
    uint8_t* buf = quirc_begin(q, nullptr, nullptr);
    if (!buf) {
        ROS_ERROR("[decodeQRCode] 获取quirc缓冲区失败");
        quirc_destroy(q);
        return "";
    }
    for (int y = 0; y < gray.rows; y++) {
        memcpy(buf + y * gray.cols, gray.ptr(y), gray.cols);
    }
    quirc_end(q);
    ROS_DEBUG("[decodeQRCode] 图像数据拷贝到quirc缓冲区完成");

    // 步骤5：检测并解码 QR 码
    int num_codes = quirc_count(q);
    ROS_INFO("[decodeQRCode] QR码检测完成 | 检测到QR码数量：%d", num_codes);
    if (num_codes == 0) {
        quirc_destroy(q);
        return "";
    }

    struct quirc_code code;
    struct quirc_data data;
    quirc_extract(q, 0, &code); // 提取第一个 QR 码
    ROS_DEBUG("[decodeQRCode] 提取第一个QR码完成");
    
    int decode_ret = quirc_decode(&code, &data);
    if (decode_ret != 0) { // 解码
        ROS_ERROR("[decodeQRCode] QR码解码失败 | 错误码：%d", decode_ret);
        quirc_destroy(q);
        return "";
    }
    ROS_DEBUG("[decodeQRCode] QR码解码成功 | 数据长度：%d", data.payload_len);

    // 步骤6：释放资源 + 返回结果
    std::string result((const char*)data.payload, data.payload_len);
    quirc_destroy(q);
    ROS_INFO("[decodeQRCode] QR码解码完成 | 解码结果：%s", result.c_str());
    return result;
}

