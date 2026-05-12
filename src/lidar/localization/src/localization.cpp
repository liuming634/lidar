// 【localization】LiDAR点云定位，用GICP将实时点云与场地地图配准
// 做法：1)加载场地PCD地图，累积20帧实时点云做球面栅格滤波(0.1°×0.1°取最远点)降噪
//      2)体素下采样(0.1m)，GICP配准得到livox_frame→rm_frame的变换矩阵
//      3)配准分数<0.2视为收敛，之后持续发布TF变换
#include <iostream>
#include <memory>
#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/registration/gicp.h>
#include <pcl/filters/voxel_grid.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <pcl/registration/icp.h>
#include <pcl/registration/registration.h>
#include <boost/shared_ptr.hpp>

namespace tdt_radar {
    struct Grid {
  pcl::PointXYZ farthestPoint;
  double maxDistance = -1.0;
};
class Localization : public rclcpp::Node {
public:
    // 构造函数：加载场地PCD地图，订阅/livox/lidar，每10秒发布一次地图点云用于调试
    Localization(const rclcpp::NodeOptions& node_options) : Node("localization", node_options) {
        std::string target_pcd_file = "config/RM2025.pcd";
        // 加载场地PCD作为配准的目标点云
        target_cloud_.reset(new pcl::PointCloud<pcl::PointXYZ>());
        if (pcl::io::loadPCDFile(target_pcd_file, *target_cloud_)) {
            RCLCPP_ERROR(this->get_logger(), "Failed to load %s", target_pcd_file.c_str());
            return;
        }

        subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/livox/lidar", 10, std::bind(&Localization::callback, this, std::placeholders::_1));

        // 发布场地点云到 /livox/map 话题
        publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/livox/map", 10);
        filter_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/filter_map", 10);
        
        //为map_pub设置计时器
        timer_ = this->create_wall_timer(std::chrono::seconds(10), [this]() {
            sensor_msgs::msg::PointCloud2 target_msg;
            pcl::toROSMsg(*target_cloud_, target_msg);
            target_msg.header.frame_id = "rm_frame";
            publisher_->publish(target_msg);
        });

        // 初始化TF广播
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    }

private:
    // 回调流程：
    // 1)接收实时点云，先累积20帧到accumulated_clouds_ FIFO缓冲区
    // 2)对累积点云做球面栅格滤波：按方位角/俯仰角(0.1°步长)分格，每格只保留最远点（消除近距离噪点）
    // 3)范围裁剪(x:5~30, y:-10~8, z<7)，体素下采样(0.1m叶大小)
    // 4)GICP配准实时点云(source)与场地地图(target)，fitness<0.2认为收敛，标记has_aligned_
    // 5)每帧发布TF: livox_frame → rm_frame
    void callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        if(!has_aligned_){

        // 从消息中转换 source_cloud
        pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::fromROSMsg(*msg, *source_cloud);
        if(accumulated_clouds_.size() < accumulate_time){
            accumulated_clouds_.push_back(source_cloud);
            return;
        }else{
            accumulated_clouds_.erase(accumulated_clouds_.begin());
            accumulated_clouds_.push_back(source_cloud);
        }
        //对于每个点云，首先输入到GirdMap中，保存每个栅格中的最远点
        for(auto accumulated_cloud : accumulated_clouds_){
            for (const auto& point : *accumulated_cloud)
                {
                double azimuth = std::atan2(point.y, point.x);
                double elevation = std::atan2(point.z, std::sqrt(point.x * point.x + point.y * point.y));

                int azimuthIndex = static_cast<int>(floor(azimuth * 180.0 / M_PI / gridSizeDegrees));
                int elevationIndex = static_cast<int>(floor(elevation * 180.0 / M_PI / gridSizeDegrees));

                double distance = std::sqrt(point.x * point.x + point.y * point.y + point.z * point.z);

                auto& grid = gridMap[std::make_pair(azimuthIndex, elevationIndex)];
                if (distance > grid.maxDistance)
                {
                    grid.farthestPoint = point;
                    grid.maxDistance = distance;
                }
            }
        }
        //然后把gripMap中的数据转换为点云
        pcl::PointCloud<pcl::PointXYZ>::Ptr result(new pcl::PointCloud<pcl::PointXYZ>());
        for (const auto& item : gridMap)
        {
            if (item.second.maxDistance > 0.0)
            {
                result->push_back(item.second.farthestPoint);
            }
        }

        //取x(0,30) , y(-10,10)的点云
        pcl::PointCloud<pcl::PointXYZ>::Ptr final_cloud(new pcl::PointCloud<pcl::PointXYZ>());
        for(auto point : result->points){
            if(point.x > 5 && point.x < 30 && point.y > -10 && point.y < 8&&point.z<7){
                final_cloud->push_back(point);
            }
        }
  
        // 下采样
        pcl::PointCloud<pcl::PointXYZ>::Ptr downsampled(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::VoxelGrid<pcl::PointXYZ> voxelgrid;
        voxelgrid.setLeafSize(0.1f, 0.1f, 0.1f);

        voxelgrid.setInputCloud(target_cloud_);
        voxelgrid.filter(*downsampled);
        *target_cloud_ = *downsampled;

        voxelgrid.setInputCloud(final_cloud);
        voxelgrid.filter(*downsampled);
        source_cloud = downsampled;

        sensor_msgs::msg::PointCloud2 filter_msg;
        pcl::toROSMsg(*source_cloud, filter_msg);
        filter_msg.header.frame_id = "livox_frame";
        filter_publisher_->publish(filter_msg);

        // GICP配准：source=实时点云, target=场地地图, 输出4×4变换矩阵
        std::cout << "--- pcl::GICP ---" << std::endl;
        boost::shared_ptr<pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ>> gicp(new pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ>());
        transform = align(gicp, target_cloud_, source_cloud);
        }
        publishTF(transform);
    }

    // GICP配准封装：执行配准，fitness score<0.2设置has_aligned_标志，返回变换矩阵
    pcl::Registration<pcl::PointXYZ, pcl::PointXYZ>::Matrix4 align(boost::shared_ptr<pcl::Registration<pcl::PointXYZ, pcl::PointXYZ>> registration, const pcl::PointCloud<pcl::PointXYZ>::Ptr& target_cloud, const pcl::PointCloud<pcl::PointXYZ>::Ptr& source_cloud) {
        registration->setInputTarget(target_cloud);
        registration->setInputSource(source_cloud);
        pcl::PointCloud<pcl::PointXYZ>::Ptr aligned(new pcl::PointCloud<pcl::PointXYZ>());
        auto t1 = std::chrono::system_clock::now();
        registration->align(*aligned);
        auto t2 = std::chrono::system_clock::now();
        // std::cout << "calib time   : " << std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count() << "[msec]" << std::endl;
        RCLCPP_WARN(this->get_logger(), "calib result : %f", registration->getFitnessScore());

        if(registration->getFitnessScore()<0.2){
        has_aligned_ = true;}

        //打印变换矩阵
        return registration->getFinalTransformation();
    }

    // 从4×4变换矩阵中提取平移(0,3)(1,3)(2,3)和旋转3×3子块→四元数，广播为TF
    void publishTF(const Eigen::Matrix4f& transform) {

        geometry_msgs::msg::TransformStamped transform_stamped;
        transform_stamped.header.stamp = this->now();
        transform_stamped.header.frame_id = "rm_frame";
        transform_stamped.child_frame_id = "livox_frame";
        transform_stamped.transform.translation.x = transform(0, 3);
        transform_stamped.transform.translation.y = transform(1, 3);
        transform_stamped.transform.translation.z = transform(2, 3);
        Eigen::Matrix3f rotation = transform.block<3, 3>(0, 0);
        Eigen::Quaternionf q(rotation);
        transform_stamped.transform.rotation.x = q.x();
        transform_stamped.transform.rotation.y = q.y();
        transform_stamped.transform.rotation.z = q.z();
        transform_stamped.transform.rotation.w = q.w();
        tf_broadcaster_->sendTransform(transform_stamped);
    }
    
    bool has_aligned_ = false;
    Eigen::Matrix4f transform;
    std::string target_pcd_file_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
    //创建一个数组用来积分点云
    pcl::PointCloud<pcl::PointXYZ>::Ptr accumulated_cloud_;
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> accumulated_clouds_;
    int accumulate_time = 20;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr filter_publisher_;
    //为map_pub设置计时器
    rclcpp::TimerBase::SharedPtr timer_;
    double gridSizeDegrees=0.1;//0.1°*0.1°
    std::map<std::pair<int, int>, Grid> gridMap;

    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};
} // namespace tdt_radar
RCLCPP_COMPONENTS_REGISTER_NODE(tdt_radar::Localization)
