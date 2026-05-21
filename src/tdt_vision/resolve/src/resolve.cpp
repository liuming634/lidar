// 【resolve】坐标解算：将检测到的2D图像坐标映射到RM2025场地3D坐标
// 做法：利用标定得到的外参(rvec/tvec)，调用parser.parse()做透视变换
//       蓝色系坐标映射到场地右半区(y+15)，红色映射后保持原y
//       发布两种结果：DetectResult(二维坐标)和PointCloud2点云+地图可视化
#include "resolve.h"
#include <iostream>
#include <opencv2/opencv.hpp>
using namespace std;
using namespace cv;
#define TDT_INFO(msg) std::cout << msg << std::endl

namespace tdt_radar {

// 构造函数：加载小地图、订阅2D点和检测结果、发布3D点云和解算结果
Resolve::Resolve(const rclcpp::NodeOptions& node_options)
    : Node("radar_resolve_node", node_options)
{
    parser_ = new parser();
    // 读取地图路径配置
    {
        std::string map_path = "config/RM2025.png";
        try {
            cv::FileStorage fs("./config/params/elevation_meta.yaml", cv::FileStorage::READ);
            if (fs.isOpened()) {
                fs["field_map"] >> map_path;
                fs.release();
            }
        } catch (const cv::Exception& e) {
            RCLCPP_WARN(this->get_logger(), "Failed to read elevation_meta.yaml, use default map: %s", e.what());
        }
        minimap = cv::imread(map_path);
    }
    point_sub = this->create_subscription<geometry_msgs::msg::Vector3>(
        "camera_point2D", rclcpp::SensorDataQoS(),
        std::bind(&Resolve::callback, this, std::placeholders::_1));

    pub = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        "camera_point3D", rclcpp::SensorDataQoS());
    match_info_sub =
        this->create_subscription<vision_interface::msg::MatchInfo>(
            "match_info", rclcpp::SensorDataQoS(),
            std::bind(&Resolve::MatchInfoCallback, this,
                      std::placeholders::_1));

    detect_sub =
        this->create_subscription<vision_interface::msg::DetectResult>(
            "detect_result", rclcpp::SensorDataQoS(),
            std::bind(&Resolve::DetectCallback, this,
                      std::placeholders::_1));
    pub_radar = this->create_publisher<vision_interface::msg::DetectResult>(
        "resolve_result", rclcpp::SensorDataQoS());

    // 读取场地尺寸配置
    try {
        cv::FileStorage fs("./config/params/field_params.yaml", cv::FileStorage::READ);
        if (fs.isOpened()) {
            fs["field_width"] >> field_width_;
            fs["field_height"] >> field_height_;
            fs.release();
        }
    } catch (const cv::Exception& e) {
        RCLCPP_WARN(this->get_logger(), "Failed to read field_params.yaml, use defaults: %s", e.what());
    }

    TDT_INFO("Load radar resolve node success!");
}

// 接收比赛信息（标记点、血量、比赛时间）
void Resolve::MatchInfoCallback(
    const vision_interface::msg::MatchInfo::SharedPtr msg)
{
    for (int i = 0; i < 6; i++) {
        markers[i] = msg->marks[i];
    }
    for (int i = 0; i < 16; i++) {
        robot_hp[i] = msg->robot_hp[i];
    }
    match_time = msg->match_time;
}

// 单点2D->3D解算回调（旧接口，处理单个相机点坐标）
void Resolve::callback(const geometry_msgs::msg::Vector3::SharedPtr msg)
{
    cv::Point2f point;
    point.x = msg->x;
    point.y = msg->y;
    auto center_point = parser_->parse(point);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(
        new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointXYZ send_point;
    send_point.x = center_point.x;
    send_point.y = field_height_ - center_point.y;
    std::cout << center_point << std::endl;
    send_point.z = parser_->get_elevation(center_point.x, center_point.y);
    cloud->points.push_back(send_point);
    sensor_msgs::msg::PointCloud2 output;
    pcl::toROSMsg(*cloud, output);
    output.header.frame_id = "rm_frame";
    pub->publish(output);
}

// 批量解算回调：将6个蓝/红方车辆的图像坐标转为场地坐标，发布点云和地图图像
void Resolve::DetectCallback(
    const vision_interface::msg::DetectResult::SharedPtr msg)
{
    cv::Mat                                 Map_clone = minimap.clone();
    std::vector<map_car>                    cars;
    vision_interface::msg::DetectResult     send_data;
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud(
        new pcl::PointCloud<pcl::PointXYZRGBA>);
    for (int i = 0; i < 6; i++) {
        cv::Point2f blue_point;
        blue_point.x = msg->blue_x[i];
        blue_point.y = msg->blue_y[i];
        if (blue_point.x * blue_point.y) {
            auto center_point = parser_->parse(blue_point);
            center_point.y = field_height_ + center_point.y;
            send_data.blue_x[i] = center_point.x;
            send_data.blue_y[i] = center_point.y;
            pcl::PointXYZRGBA send_point;
            send_point.x = center_point.x;
            send_point.y = center_point.y;
            send_point.z = 1;
            send_point.r = 0;
            send_point.g = 0;
            send_point.b = 255;
            send_point.a = 255;
            cloud->points.push_back(send_point);
            cv::circle(
                Map_clone,
                cv::Point((Map_clone.cols * center_point.x) / field_width_,
                          Map_clone.rows * (field_height_ - center_point.y) / field_height_),
                20, cv::Scalar(200, 0, 0), -1);
            cv::putText(
                Map_clone, std::to_string(i + 1),
                cv::Point((Map_clone.cols * center_point.x) / field_width_ - 10,
                          Map_clone.rows * (field_height_ - center_point.y) / field_height_ + 10),
                cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 255, 255), 2);
        }
        cv::Point2f red_point;
        red_point.x = msg->red_x[i];
        red_point.y = msg->red_y[i];
        if (red_point.x * red_point.y) {
            auto center_point = parser_->parse(red_point);
            center_point.y = field_height_ + center_point.y;
            send_data.red_x[i] = center_point.x;
            send_data.red_y[i] = center_point.y;
            pcl::PointXYZRGBA send_point;
            send_point.x = center_point.x;
            send_point.y = center_point.y;
            send_point.z = 1;
            send_point.r = 255;
            send_point.g = 0;
            send_point.b = 0;
            send_point.a = 255;
            cloud->points.push_back(send_point);
            cv::circle(
                Map_clone,
                cv::Point((Map_clone.cols * center_point.x) / field_width_,
                          Map_clone.rows * (field_height_ - center_point.y) / field_height_),
                20, cv::Scalar(0, 0, 200), -1);
            cv::putText(
                Map_clone, std::to_string(i + 1),
                cv::Point((Map_clone.cols * center_point.x) / field_width_ - 10,
                          Map_clone.rows * (field_height_ - center_point.y) / field_height_ + 10),
                cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 255, 255), 2);
        }
    }

    send_data.header.stamp = msg->header.stamp;
    pub_radar->publish(send_data);
    auto cloud_msg = sensor_msgs::msg::PointCloud2();
    pcl::toROSMsg(*cloud, cloud_msg);
    cloud_msg.header.frame_id = "rm_frame";
    cloud_msg.header.stamp = msg->header.stamp;
    pub->publish(cloud_msg);
}
}  // namespace tdt_radar
RCLCPP_COMPONENTS_REGISTER_NODE(tdt_radar::Resolve)