#include "vision_detection.h"
#include "flight_control.h"
#include "mission_header.h"

/************************************************************************
视觉相关函数实现
*************************************************************************/

//============ cpp调用 ===============
bool onFrame() {
    RecognizeResult res = recognizeTarget(current_frame, cfg);

    if (res.success) {
        // 1. 判断当前识别类别是否在二维码目标列表中
        bool is_target_class = std::find(
            g_qrcode_classes.begin(), 
            g_qrcode_classes.end(), 
            res.cls_name
        ) != g_qrcode_classes.end();

        if (is_target_class) {
            // 2. 是目标类别：计算偏移并打印日志（fly函数你自行实现调用）
            float dx = res.x - local_pos.pose.pose.position.x;
            float dy = res.y - local_pos.pose.pose.position.y;
            std::cout << "识别成功：" << res.cls_name 
                      << " | 偏移：" << dx << "," << dy << std::endl;
            
            //此处应该调用无人机投掷的函数
            
            return true;
        } else {
            // 3. 非目标类别：打印日志并返回false
            std::cout << "识别成功但非二维码目标类别：" << res.cls_name << std::endl;
            return false;
        }
    }else{
        //此处需要动态调整的逻辑
    }

    return false;
}

// ========== 目标识别函数（极简） ==========
RecognizeResult recognizeTarget(const cv::Mat& frame, const Config& cfg) {
    RecognizeResult res;
    geometry_msgs::Point p;
    
    // 1. 粗识别（正方形+圆形合一）
    res.rough_info = roughDetectTarget(frame);
    if (!res.rough_info.is_valid) return res;
    
    // 2. 精确定位中心图片
    res.center_img = getCenterImageRect(res.rough_info);
    
    // 3. 精确分类
    if (!preciseClassify(frame, res.center_img, cfg, res.cls_name, res.conf)) return res;
    
    // 4. 计算相对位置
    p = change_to_world(res.rough_info.cx, res.rough_info.cy);
    res.x = p.x; res.y = p.y;
    res.success = true;


    return res;
}

// ========== 核心：粗识别函数（正方形+圆形合一） ==========
/**
 * 粗识别：一次性完成外层正方形+圆形检测
 * @param frame 输入图像
 * @return 粗识别结果（含正方形+圆形信息）
 */
RoughTargetInfo roughDetectTarget(const cv::Mat& frame) {
    RoughTargetInfo res;
    if (frame.empty()) return res;

    // 1. 预处理（灰度+模糊+边缘检测）
    cv::Mat gray, blur, edges;
    cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
    cv::GaussianBlur(gray, blur, cv::Size(7,7), 0);
    cv::Canny(blur, edges, 40, 120);

    // 2. 筛选正方形轮廓
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(edges.clone(), contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    if (contours.empty()) return res;

    // 找最大的正方形（4边+长宽比0.9~1.1+面积>1000）
    std::vector<cv::Rect> square_rects;
    for (const auto& cnt : contours) {
        double peri = cv::arcLength(cnt, true);
        std::vector<cv::Point> approx;
        cv::approxPolyDP(cnt, approx, 0.02*peri, true);
        if (approx.size() == 4) {
            cv::Rect rect = cv::boundingRect(cnt);
            float ratio = (float)rect.width/rect.height;
            if (0.9<=ratio<=1.1 && cv::contourArea(cnt)>1000) {
                square_rects.push_back(rect);
            }
        }
    }
    if (square_rects.empty()) return res;
    res.outer_square = *std::max_element(square_rects.begin(), square_rects.end(),
        [](const cv::Rect& a, const cv::Rect& b) { return a.area() < b.area(); });

    // 3. 检测外层圆形（基于正方形ROI）
    int x1 = res.outer_square.x, y1 = res.outer_square.y;
    int x2 = x1 + res.outer_square.width, y2 = y1 + res.outer_square.height;
    int square_size = std::min(x2-x1, y2-y1);
    cv::Mat square_roi = gray(cv::Range(y1,y2), cv::Range(x1,x2));
    
    std::vector<cv::Vec3f> circles;
    cv::HoughCircles(square_roi, circles, cv::HOUGH_GRADIENT, 1, square_roi.rows/8, 100, 30,
        (int)(square_size*MIN_CIRCLE_RATIO/2), (int)(square_size*MAX_CIRCLE_RATIO/2));
    
    if (circles.empty()) return res;
    cv::Vec3f max_circle = *std::max_element(circles.begin(), circles.end(),
        [](const cv::Vec3f& a, const cv::Vec3f& b) { return a[2] < b[2]; });
    
    // 转换为原图坐标
    res.cx = max_circle[0] + x1;
    res.cy = max_circle[1] + y1;
    res.radius = max_circle[2];
    res.is_valid = true;

    return res;
}

// ========== 精确定位+分类+坐标计算（精简版） ==========
/**
 * 精确定位中心图片（数学计算）
 */
cv::Rect getCenterImageRect(const RoughTargetInfo& rough_info) {
    float inner_radius = rough_info.radius * INNER_CIRCLE_RATIO;
    int img_size = (int)(inner_radius * 2 * CENTER_IMG_RATIO);
    int x1 = (int)(rough_info.cx - img_size/2), y1 = (int)(rough_info.cy - img_size/2);
    int x2 = x1 + img_size, y2 = y1 + img_size;
    return cv::Rect(std::max(0,x1), std::max(0,y1), x2-x1, y2-y1);
}

/**
 * 精确分类中心图片
 */
bool preciseClassify(const cv::Mat& frame, const cv::Rect& center_rect, const Config& cfg,
                     std::string& cls_name, float& conf) {
    if (center_rect.empty()) return false;
    cv::Mat roi = frame(center_rect & cv::Rect(0,0,frame.cols,frame.rows));
    
    // 预处理+ONNX推理（精简版）
    cv::Mat resized, rgb;
    cv::resize(roi, resized, cv::Size(32,32));
    cv::cvtColor(resized, rgb, cv::COLOR_BGR2RGB);
    cv::Mat float_img;
    rgb.convertTo(float_img, CV_32F, 1.0/255.0);

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
        if (cfg.use_gpu) sess_opt.AppendExecutionProvider_CUDA(OrtCUDAProviderOptions{});
        
        Ort::Session sess(env, cfg.model_path.c_str(), sess_opt);
        auto in_name = sess.GetInputName(0, Ort::AllocatorWithDefaultOptions());
        auto out_name = sess.GetOutputName(0, Ort::AllocatorWithDefaultOptions());
        
        std::vector<int64_t> shape = {1,3,32,32};
        auto tensor = Ort::Value::CreateTensor<float>(Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeCPU),
            input_data.data(), input_data.size(), shape.data(), shape.size());
        
        auto outputs = sess.Run({}, &in_name, &tensor, 1, &out_name, 1);
        float* out_data = outputs[0].GetTensorMutableData<float>();
        int top_idx = std::max_element(out_data, out_data+CIFAR100_CLASSES.size()) - out_data;
        
        conf = out_data[top_idx];
        cls_name = CIFAR100_CLASSES[top_idx];
        return conf >= cfg.conf_threshold;
    } catch (...) { return false; }
}

/**
 * @brief 极简二维码识别+信息提取（格式确定，无冗余校验）
 * @param frame 输入图像（BGR格式）
 * @return bool 是否成功识别二维码
 * @note 二维码内容格式：英文逗号分隔的类别列表（如 "apple,man,tree"）
 */
bool detectQRCodeAndExtractInfo(const cv::Mat& frame) {
    // 1. 初始化二维码检测器（静态创建，提升效率）
    static cv::QRCodeDetector qr_detector;
    
    // 2. 检测并解码二维码（格式确定，无需校验图像为空）
    std::string qr_text = qr_detector.detectAndDecode(frame);
    if (qr_text.empty()) return false;

    // 3. 解析二维码内容（固定逗号分隔，无需兼容其他格式）
    g_qrcode_classes.clear();
    std::istringstream iss(qr_text);
    std::string class_name;
    while (std::getline(iss, class_name, ',')) {
        g_qrcode_classes.push_back(class_name);
    }

    return true;
}