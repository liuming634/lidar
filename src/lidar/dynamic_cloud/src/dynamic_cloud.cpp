// 【dynamic_cloud】LiDAR动态物体检测与飞镖/无人机预警
// 做法：1)通过TF将雷达点云转到场地坐标系 rm_frame
//      2)用KD-tree查询每个点在场地静态地图中的最近邻，距离>0.1m判定为动态点（多线程并行）
//      3)多帧FIFO累积(3帧)增强检测稳定性
//      4)对非场地内的"其他"点云用空间立方体过滤器判断飞镖(dart)和无人机(fly)的威胁等级
#include "dynamic_cloud.h"
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/time.hpp>
namespace tdt_radar {
DynamicCloud::DynamicCloud(const rclcpp::NodeOptions& node_options)
    : rclcpp::Node("dynamic_cloud_node", node_options),
      tf_buffer_(this->get_clock()),
      tf_listener_(tf_buffer_)
{
    RCLCPP_INFO(this->get_logger(), "Dynamic_cloud Node start");
    auto temp_cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(
        new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile<pcl::PointXYZ>("config/RM2025.pcd",
                                            *temp_cloud) == -1) {
        PCL_ERROR("Couldn't read file map.pcd \n");
    }
    pcl::VoxelGrid<pcl::PointXYZ> sor;
    sor.setInputCloud(temp_cloud);
    sor.setLeafSize(0.1f, 0.1f, 0.1f);
    auto result = pcl::PointCloud<pcl::PointXYZ>::Ptr(
        new pcl::PointCloud<pcl::PointXYZ>);
    sor.filter(*result);
    map_cloud = result;
    kd_Tree.setInputCloud(map_cloud);

    sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/livox/lidar", 10,
        std::bind(&DynamicCloud::callback, this, std::placeholders::_1));
    pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        "/livox/lidar_dynamic", 10);
    other_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        "/livox/lidar_other", 10);
    detect_pub_ = this->create_publisher<vision_interface::msg::RadarWarn>(
        "/lidar_detect", 10);
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Dynamic Cloud Launch!");
}

// 多线程动态点提取：将点云按线程数均分，每个线程对各自分片逐点做KD-tree最近邻查询
// 若到静态地图最近点的平方距离 > threshold^2，则判定为动态物体点
void DynamicCloud::GetDynamicCloud(
    pcl::PointCloud<pcl::PointXYZ>& input_cloud,
    pcl::PointCloud<pcl::PointXYZ>& output_cloud, float threshold,
    int thread_num)
{
    int                                         K = 1;
    std::vector<std::thread>                    threads;
    std::vector<pcl::PointCloud<pcl::PointXYZ>> clouds(thread_num);
    auto start = std::chrono::system_clock::now();
    int  cloud_size = input_cloud.points.size();
    int  step = cloud_size / thread_num;
    for (int i = 0; i < thread_num; i++) {
        threads.push_back(std::thread([i, step, cloud_size, &clouds,
                                       &input_cloud, this, threshold, K]() {
            for (int j = i * step; j < (i + 1) * step; j++) {
                std::vector<int>   pointIdxNKNSearch(K);
                std::vector<float> pointNKNSquaredDistance(K);
                if (kd_Tree.nearestKSearch(input_cloud.points[j], K,
                                           pointIdxNKNSearch,
                                           pointNKNSquaredDistance) > 0) {
                    if (pointNKNSquaredDistance[0] > threshold) {
                        clouds[i].points.push_back(input_cloud.points[j]);
                    }
                }
            }
        }));
    }
    for (auto& t : threads) {
        t.join();
    }
    for (auto& cloud : clouds) {
        output_cloud += cloud;
    }
    auto end = std::chrono::system_clock::now();
}

// 多线程点云坐标变换：从TF提取平移向量+旋转四元数构造Eigen::Affine3f矩阵
// 点云按线程数均分，每个线程对分片逐点做仿射变换，合并输出
void TransformCloud(pcl::PointCloud<pcl::PointXYZ>&      input_cloud,
                    pcl::PointCloud<pcl::PointXYZ>&      output_cloud,
                    geometry_msgs::msg::TransformStamped transform_stamped,
                    int                                  thread_num)
{
    std::vector<std::thread>                    threads;
    std::vector<pcl::PointCloud<pcl::PointXYZ>> clouds(thread_num);
    auto start = std::chrono::system_clock::now();
    int  cloud_size = input_cloud.points.size();
    int  step = cloud_size / thread_num;

    Eigen::Affine3f transform = Eigen::Affine3f::Identity();
    transform.translation() << transform_stamped.transform.translation.x,
        transform_stamped.transform.translation.y,
        transform_stamped.transform.translation.z;
    Eigen::Quaternionf rotation(transform_stamped.transform.rotation.w,
                                transform_stamped.transform.rotation.x,
                                transform_stamped.transform.rotation.y,
                                transform_stamped.transform.rotation.z);
    transform.rotate(rotation);

    for (int i = 0; i < thread_num; i++) {
        threads.push_back(std::thread(
            [i, step, cloud_size, &clouds, &input_cloud, transform]() {
                for (int j = i * step; j < (i + 1) * step && j < cloud_size;
                     j++) {
                    pcl::PointXYZ   point = input_cloud.points[j];
                    Eigen::Vector3f point_vec(point.x, point.y, point.z);
                    Eigen::Vector3f point_out = transform * point_vec;

                    pcl::PointXYZ transformed_point;
                    transformed_point.x = point_out.x();
                    transformed_point.y = point_out.y();
                    transformed_point.z = point_out.z();

                    clouds[i].points.push_back(transformed_point);
                }
            }));
    }

    for (auto& t : threads) {
        t.join();
    }
    for (auto& cloud : clouds) {
        output_cloud += cloud;
    }
    auto end = std::chrono::system_clock::now();
    RCLCPP_INFO(
        rclcpp::get_logger("rclcpp"), "transform time: %f",
        std::chrono::duration_cast<std::chrono::microseconds>(end - start)
                .count() /
            1000.0);
}

// 主回调流程：
// 1) lookupTransform获取雷达→场地坐标系的TF
// 2) 用Eigen Affine3f做pcl::transformPointCloud将全部点转到rm_frame
// 3) 空间范围过滤：只保留场地内的点(x:3~28, y:0~15, z:0~1.4)
//    同时分离出 "非场地但仍需监控的区域"（飞镖区+无人机通道）到other点云
// 4) GetDynamicCloud: 对场地内点云用KD-tree比对静态地图，提取动态点
// 5) 维护FIFO队列accumulated_clouds_（最多3帧），将多帧动态点求和累积增强稳定性
// 6) 对other点云做飞镖区检测(>5个点判定有飞镖)和无人机三级区域检测(>40个点触发)
void DynamicCloud::callback(
    const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
    vision_interface::msg::RadarWarn lidar_detect;
    // 飞镖检测：判断点是否在敌方基地前哨的飞镖发射器区域[空间立方体]
    auto dart_cloud_filter = [](pcl::PointXYZ& point) {
        return (point.x > 28 - 0.5889 - 0.1885 && point.x < 28 - 0.5889) &&
               (point.y > 3.925 && point.y < 4.525) &&
               (point.z > 2.4722 - 0.859 + 0.1 && point.z < 2.4722);
    };  // 飞镖检测
    // 飞机(sentry)安全区：己方半场上空，有目标则处于起飞/悬停状态
    auto fly_safe_filter = [](pcl::PointXYZ& point) {
        return (point.x > 28 - 2.775 && point.x < 27.5) &&
               (point.y > 0.2 && point.y < 2.2) &&
               (point.z > 1.7 && point.z < 3);
    };  // 飞机起飞

    // 飞机预警区：已飞到己方半场靠近基地位置，需要预警
    auto fly_warn_filter = [](pcl::PointXYZ& point) {
        return (point.x > 19.83 && point.x < 28 - 2.7) &&
               (point.y > 0.2 && point.y < 1.356 + 2.4 + 0.8) &&
               (point.z > 1.7 && point.z < 3);
    };  // 飞机飞到半场

    // 飞机报警区：已飞到场地中场，威胁等级最高，需立即响应
    auto fly_alarm_filter = [](pcl::PointXYZ& point) {
        return (point.x > 13 && point.x < 20.5) &&
               (point.y > 0.2 && point.y < 1.356 + 2.4 + 0.8) &&
               (point.z > 1.7 && point.z < 3);
    };  // 飞机飞到中场

    auto receive_cloud = pcl::PointCloud<pcl::PointXYZ>();
    pcl::fromROSMsg(*msg, receive_cloud);

    std::chrono::steady_clock::time_point t1 =
        std::chrono::steady_clock::now();
    geometry_msgs::msg::TransformStamped transform_stamped;
    auto ta = std::chrono::steady_clock::now();
    try {
        transform_stamped = tf_buffer_.lookupTransform(
            "rm_frame", msg->header.frame_id, tf2::TimePointZero);
    }
    catch (tf2::TransformException& ex) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Transform error: %s",
                     ex.what());
        return;
    }
    pcl::PointCloud<pcl::PointXYZ> transformed_cloud;
    auto                           transform = Eigen::Affine3f::Identity();
    transform.translation() << transform_stamped.transform.translation.x,
        transform_stamped.transform.translation.y,
        transform_stamped.transform.translation.z;
    Eigen::Quaternionf rotation(transform_stamped.transform.rotation.w,
                                transform_stamped.transform.rotation.x,
                                transform_stamped.transform.rotation.y,
                                transform_stamped.transform.rotation.z);
    transform.rotate(rotation);
    pcl::transformPointCloud(receive_cloud, transformed_cloud, transform);
    pcl::PointCloud<pcl::PointXYZ> filtered_cloud;
    pcl::PointCloud<pcl::PointXYZ> other_filtered_cloud;
    for (size_t i = 0; i < transformed_cloud.size(); i++) {
        auto& point = transformed_cloud.points[i];
        if (point.x < 3 || point.x > 28 || point.y < 0 || point.y > 15 ||
            point.z < 0 || point.z > 1.4 ||
            (point.y > 0 && point.y < 5 && point.x > 25) ||
            ((21.5 - 2.9 / sqrt(2)) < (point.x + point.y) &&
             (point.x + point.y) < (21.5 + 2.9 / sqrt(2)) &&
             (-6.5 - 0.9 / sqrt(2)) < (point.y - point.x) &&
             (point.y - point.x) < (-6.5 + 0.9 / sqrt(2)))) {
            if ((point.x > 28 - 0.5889 - 0.1885 && point.x < 28 - 0.5889) &&
                    (point.y > 3.925 && point.y < 4.525) &&
                    (point.z > 2.4722 - 0.859 + 0.1 && point.z < 2.4722) ||
                (point.x > 13 && point.x < 27.5) &&
                    (point.y > 0.2 && point.y < 1.356 + 2.4 + 0.8) &&
                    (point.z > 1.7 && point.z < 3)) {
                other_filtered_cloud.push_back(point);
            }
            continue;
        }
        filtered_cloud.push_back(point);
    }
    pcl::PointCloud<pcl::PointXYZ> dynamic_pointcloud;
    GetDynamicCloud(filtered_cloud, dynamic_pointcloud, 0.1, 12);

    if (accumulate_count < accumulate_time) {
        accumulated_clouds_.push_back(dynamic_pointcloud.makeShared());
        other_accumulated_clouds_.push_back(
            other_filtered_cloud.makeShared());
        accumulate_count++;
    } else {
        accumulated_clouds_.erase(accumulated_clouds_.begin());
        accumulated_clouds_.push_back(dynamic_pointcloud.makeShared());
        other_accumulated_clouds_.erase(other_accumulated_clouds_.begin());
        other_accumulated_clouds_.push_back(
            other_filtered_cloud.makeShared());
    }
    pcl::PointCloud<pcl::PointXYZ> accumulated_cloud;
    for (auto it = accumulated_clouds_.begin();
         it != accumulated_clouds_.end(); ++it) {
        accumulated_cloud += **it;
    }
    pcl::PointCloud<pcl::PointXYZ> other_accumulated_cloud;
    for (auto it = other_accumulated_clouds_.begin();
         it != other_accumulated_clouds_.end(); ++it) {
        other_accumulated_cloud += **it;
    }
    ta = std::chrono::steady_clock::now();
    sensor_msgs::msg::PointCloud2 output;
    accumulated_cloud.header.frame_id = "rm_frame";
    other_accumulated_cloud.header.frame_id = "rm_frame";
    pcl::toROSMsg(accumulated_cloud, output);
    output.header.frame_id = "rm_frame";
    output.header.stamp = msg->header.stamp;
    pub_->publish(output);

    pcl::toROSMsg(other_accumulated_cloud, output);
    other_pub_->publish(output);
    pcl::PointCloud<pcl::PointXYZ> dart_cloud;
    for (size_t i = 0; i < other_accumulated_cloud.size(); i++) {
        auto& point = other_accumulated_cloud.points[i];
        if (dart_cloud_filter(point)) {
            dart_cloud.push_back(point);
        }
    }

    if (dart_cloud.size() > 5) {
        RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "Find dart cloud!");
        RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "Dart cloud size: %d",
                    dart_cloud.size());
        lidar_detect.dart_state = 1;
    }

    pcl::PointCloud<pcl::PointXYZ> fly_safe_cloud;
    pcl::PointCloud<pcl::PointXYZ> fly_warn_cloud;
    pcl::PointCloud<pcl::PointXYZ> fly_alarm_cloud;

    for (size_t i = 0; i < other_accumulated_cloud.size(); i++) {
        auto& point = other_accumulated_cloud.points[i];
        if (fly_safe_filter(point)) {
            fly_safe_cloud.push_back(point);
        }
        if (fly_warn_filter(point)) {
            fly_warn_cloud.push_back(point);
        }
        if (fly_alarm_filter(point)) {
            fly_alarm_cloud.push_back(point);
        }
    }
    if (fly_safe_cloud.size() > 40) {
        lidar_detect.fly_state = 1;
    }

    if (fly_warn_cloud.size() > 40) {
        lidar_detect.fly_state = 2;
    }

    if (fly_alarm_cloud.size() > 40) {
        lidar_detect.fly_state = 3;
    }

    switch (lidar_detect.fly_state) {
    case 0:
        break;
    case 1:
        RCLCPP_WARN(rclcpp::get_logger("rclcpp"),
                    "Safe fly object detected!");
        break;
    case 2:
        RCLCPP_WARN(rclcpp::get_logger("rclcpp"),
                    "Warn fly object detected!");
        break;
    case 3:
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),
                     "Alarm fly object detected!");
        break;
    default:
        break;
    }

    detect_pub_->publish(lidar_detect);
    std::chrono::steady_clock::time_point t2 =
        std::chrono::steady_clock::now();
    RCLCPP_INFO(
        rclcpp::get_logger("rclcpp"), "Dynamic callback time: %f",
        std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1)
                .count() /
            1000.0);
}
}  // namespace tdt_radar

RCLCPP_COMPONENTS_REGISTER_NODE(tdt_radar::DynamicCloud)
