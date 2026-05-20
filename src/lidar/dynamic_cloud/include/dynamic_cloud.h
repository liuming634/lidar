// dynamic_cloud.h：动态点云检测节点头文件，支持多线程KD树动态点提取
#include <pcl/filters/passthrough.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <chrono>
#include <vector>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <pcl/impl/point_types.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>
#include <vision_interface/msg/radar_warn.hpp>
#include "pcl/io/pcd_io.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl_conversions/pcl_conversions.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"

namespace tdt_radar {
class DynamicCloud : public rclcpp::Node {
public:
    DynamicCloud(const rclcpp::NodeOptions& node_options);
    ~DynamicCloud() {}

private:
    // 配置文件读取的数值
    float        crop_x_min_ = 3.0;
    float        crop_x_max_ = 28.0;
    float        crop_y_min_ = 0.0;
    float        crop_y_max_ = 15.0;
    float        crop_z_min_ = 0.0;
    float        crop_z_max_ = 1.4;
    float        kdtree_threshold_ = 0.1;
    std::string  pcd_path_ = "config/RM2025.pcd";

    int                                              accumulate_time = 3;
    int                                              accumulate_count = 0;
    pcl::PointCloud<pcl::PointXYZ>::Ptr              map_cloud;
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> accumulated_clouds_;
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>
        other_accumulated_clouds_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr    pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr other_pub_;
    rclcpp::Publisher<vision_interface::msg::RadarWarn>::SharedPtr
         detect_pub_;
    void callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
    void GetDynamicCloud(pcl::PointCloud<pcl::PointXYZ>& input_cloud,
                         pcl::PointCloud<pcl::PointXYZ>& output_cloud,
                         float threshold, int thread_num);
    pcl::KdTreeFLANN<pcl::PointXYZ> kd_Tree;
    tf2_ros::Buffer                 tf_buffer_;
    tf2_ros::TransformListener      tf_listener_;
};
}  // namespace tdt_radar