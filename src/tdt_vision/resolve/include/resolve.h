// resolve模块头文件：定义Resolve节点，负责2D到3D坐标转换和地图映射
#ifndef RADAR_RESOLVE_H
#define RADAR_RESOLVE_H

#include <memory>
#include <geometry_msgs/msg/detail/point__struct.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <vision_interface/msg/match_info.hpp>
#include "cv_bridge/cv_bridge.h"
#include "geometry_msgs/msg/vector3.hpp"
#include "opencv2/opencv.hpp"
#include "pcl/point_types.h"
#include "pcl_conversions/pcl_conversions.h"
#include "radar_utils.h"
#include "vision_interface/msg/detect_result.hpp"
#include "vision_interface/msg/radar2_sentry.hpp"
namespace tdt_radar {

// 解算节点：将图像2D坐标转换为场地3D坐标，在小地图上绘制并发布点云
class Resolve final : public rclcpp::Node {
public:
    explicit Resolve(const rclcpp::NodeOptions& options);
    void callback(const std::shared_ptr<geometry_msgs::msg::Vector3> msg);
    void DetectCallback(
        const vision_interface::msg::DetectResult::SharedPtr msg);
    void MatchInfoCallback(
        const vision_interface::msg::MatchInfo::SharedPtr msg);
    rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr point_sub;
    rclcpp::Subscription<vision_interface::msg::DetectResult>::SharedPtr
        detect_sub;
    rclcpp::Subscription<vision_interface::msg::MatchInfo>::SharedPtr
            match_info_sub;
    parser* parser_;
    int     EnemyColor = 1;
    cv::Mat minimap;
    int     markers[6];
    int16_t match_time;
    uint8_t robot_hp[16];

private:
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub;
    rclcpp::Publisher<vision_interface::msg::DetectResult>::SharedPtr
        pub_radar;
};

// 地图上车辆的数据结构
class map_car {
public:
    float x;
    float y;
    int   id;
};
}  // namespace tdt_radar

#endif
