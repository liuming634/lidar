// ros2 run 一次即可将 /livox/lidar 的点云攒成一帧保存为 PCD
// 用法：先启动雷达，再运行本节点，等几秒后 Ctrl+C
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

class BagToPcd : public rclcpp::Node {
public:
    BagToPcd() : Node("bag_to_pcd") {
        // 从参数接收录制时长（秒）和输出路径
        this->declare_parameter("duration", 10.0);
        this->declare_parameter("output", "config/field.pcd");
        duration_ = this->get_parameter("duration").as_double();
        output_ = this->get_parameter("output").as_string();

        cloud_.reset(new pcl::PointCloud<pcl::PointXYZ>);
        sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/livox/lidar", rclcpp::SensorDataQoS(),
            std::bind(&BagToPcd::callback, this, std::placeholders::_1));

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(duration_ * 1000)),
            std::bind(&BagToPcd::finish, this));

        RCLCPP_INFO(this->get_logger(), "Recording for %.1f seconds...", duration_);
    }

private:
    void callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        pcl::PointCloud<pcl::PointXYZ> temp;
        pcl::fromROSMsg(*msg, temp);
        *cloud_ += temp;
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
            "Accumulated %zu points", cloud_->size());
    }

    void finish() {
        cloud_->header.frame_id = "livox_frame";
        pcl::io::savePCDFileBinary(output_, *cloud_);
        RCLCPP_INFO(this->get_logger(),
            "Saved %zu points to %s", cloud_->size(), output_.c_str());
        rclcpp::shutdown();
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
    rclcpp::TimerBase::SharedPtr timer_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_;
    double duration_;
    std::string output_;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<BagToPcd>());
    return 0;
}
