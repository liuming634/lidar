// detect模块头文件：定义Detect节点和Car数据结构
// Detect节点负责YOLO车辆检测、装甲板检测与分类
#ifndef RADAR_DETECT_H
#define RADAR_DETECT_H

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/msg/image.hpp>
#include "classify.hpp"
#include "cv_bridge/cv_bridge.h"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "NvidiaInterface.hpp"
#include "opencv2/opencv.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "vision_interface/msg/detect_result.hpp"
#include "yolos.hpp"
#include "BaseInfer.hpp"
#include <fstream>
namespace tdt_radar {

// 检测节点：加载YOLO+装甲板+分类器三个模型，处理图像并发布检测结果
class Detect final : public rclcpp::Node {
public:
    explicit Detect(const rclcpp::NodeOptions& options);
    void callback(const std::shared_ptr<sensor_msgs::msg::Image> msg);
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub;
    rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr
        compressed_image_sub;

private:
    std::shared_ptr<Infer<yolo::BoxArray>>     yolo;
    std::shared_ptr<Infer<yolo::BoxArray>>     armor_yolo;
    std::shared_ptr<Infer<int>> classifier;
    rclcpp::Publisher<vision_interface::msg::DetectResult>::SharedPtr pub;

    bool        if_rosbag = false;
    int         EnemyColor;
    int         debug;
    std::string yolo_path;
    std::string armor_path;
    std::string classify_path;
};
// Car数据结构：存储车辆矩形框、装甲板列表、中心点、编号和颜色
class Car {
public:
    cv::Rect       car_rect;
    yolo::Box      car;
    yolo::BoxArray armors;
    cv::Point2f    center;
    cv::Rect       center_rect;
    int            number = 0;
    int            color = 1;
};
}  // namespace tdt_radar

#endif
