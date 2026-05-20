// 【debug_map】2D小地图可视化+英雄状态+决策发送
// 做法：1)从/kalman_detect接收融合后的蓝/红方6车位置，在RM2025.png地图上画圆圈+编号
//      2)用滑动时间窗(0.5s内更新有效)显示车辆最新位置
//      3)判断英雄机器人(id=0)的进退场状态(根据x坐标阈值判断在基地/前哨/其他)
//      4)通过marks分数(≥117→relax, <105→恢复)和更新新鲜度控制Radar2Sentry的发送频率
//      5)发送/hero_state和/Radar2Sentry给决策层和哨兵
#include <fstream>
#include <cv_bridge/cv_bridge.h>
#include <opencv4/opencv2/opencv.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <vision_interface/msg/detect_result.hpp>
#include <vision_interface/msg/match_info.hpp>
#include <vision_interface/msg/radar2_sentry.hpp>
#include <vision_interface/msg/radar_warn.hpp>
namespace tdt_radar {
class DebugMap : public rclcpp::Node {
public:
    // 构造函数：订阅kalman_detect、resolve_result、match_info，发布雷达决策信息
    explicit DebugMap(const rclcpp::NodeOptions& options)
        : Node("debug_map", options)
    {
        detect_result_sub =
            this->create_subscription<vision_interface::msg::DetectResult>(
                "/kalman_detect", 10,
                std::bind(&DebugMap::callback, this,
                          std::placeholders::_1));
        camera_detect_sub =
            this->create_subscription<vision_interface::msg::DetectResult>(
                "/resolve_result", rclcpp::SensorDataQoS(),
                std::bind(&DebugMap::camera_callback, this,
                          std::placeholders::_1));
        // 读取地图路径与显示缩放配置
        std::string map_path = "config/RM2025.png";
        int display_scale = 25;
        try {
            cv::FileStorage fs("./config/params/elevation_meta.yaml", cv::FileStorage::READ);
            if (fs.isOpened()) {
                fs["field_map"] >> map_path;
                fs["display_scale"] >> display_scale;
                fs.release();
            }
        } catch (const cv::Exception& e) {
            RCLCPP_WARN(this->get_logger(), "Failed to read elevation_meta.yaml, use default: %s", e.what());
        }
        map = cv::imread(map_path);
        match_info_sub =
            this->create_subscription<vision_interface::msg::MatchInfo>(
                "/match_info", 10,
                std::bind(&DebugMap::save_match_info, this,
                          std::placeholders::_1));
        radar_warn_pub =
            this->create_publisher<vision_interface::msg::RadarWarn>(
                "/hero_state", 10);
        debug_map_pub =
            this->create_publisher<sensor_msgs::msg::Image>("/map_2d", 10);
        radar2sentry_pub =
            this->create_publisher<vision_interface::msg::Radar2Sentry>(
                "/Radar2Sentry", rclcpp::SensorDataQoS());
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
        if (display_scale > 0) {
            cv::resize(map, map, cv::Size((int)(field_width_ * display_scale), (int)(field_height_ * display_scale)));
        }
    }
    void save_match_info(
        const std::shared_ptr<vision_interface::msg::MatchInfo> msg)
    {
        this->match_info = *msg;
        if (msg->self_color == 1)
            match_info.self_color = 2;
    }

    // 在小地图上绘制蓝/红方车辆位置（带编号），0.5秒内更新有效
    void show_map()
    {
        auto   now_time = std::chrono::system_clock::now();
        double time = std::chrono::duration_cast<std::chrono::milliseconds>(
                          now_time.time_since_epoch())
                          .count() /
                      1000.0;
        auto clone_map = map.clone();
        for (int i = 0; i < 6; i++) {
            int number = i + 1;
            if (number == 6)
                number++;
            if (blue_point[i].x * blue_point[i].y &&
                time - blue_update[i] < 0.5) {
                cv::Point2f point = cv::Point2f(
                    clone_map.cols * blue_point[i].x / field_width_,
                    clone_map.rows * (field_height_ - blue_point[i].y) / field_height_);
                cv::circle(clone_map, point, 10, cv::Scalar(200, 0, 0), -1);
                cv::putText(clone_map, std::to_string(number),
                            cv::Point(point.x - 6, point.y + 5),
                            cv::FONT_HERSHEY_SIMPLEX, 0.5,
                            cv::Scalar(255, 255, 255));
            }
            if (red_point[i].x * red_point[i].y &&
                time - red_update[i] < 0.5) {
                cv::Point2f point = cv::Point2f(
                    clone_map.cols * red_point[i].x / field_width_,
                    clone_map.rows * (field_height_ - red_point[i].y) / field_height_);
                cv::circle(clone_map, point, 10, cv::Scalar(0, 0, 200), -1);
                cv::putText(clone_map, std::to_string(number),
                            cv::Point(point.x - 6, point.y + 5),
                            cv::FONT_HERSHEY_SIMPLEX, 0.5,
                            cv::Scalar(255, 255, 255));
            }
        }
        cv::imshow("map", clone_map);
        cv::waitKey(1);
    }
    // 相机检测结果回调（低频率接收，处理首次检测到目标时初始化位置）
    void camera_callback(
        const std::shared_ptr<vision_interface::msg::DetectResult> msg)
    {
        auto   now = std::chrono::system_clock::now();
        double time = std::chrono::duration_cast<std::chrono::milliseconds>(
                          now.time_since_epoch())
                          .count() /
                      1000.0;
        for (int i = 0; i < 6; i++) {
            if (msg->blue_x[i] * msg->blue_y[i]) {
                if (time - blue_time[i] > 5) {
                    blue_point[i] =
                        cv::Point2f(msg->blue_x[i], msg->blue_y[i]);
                    if (!match_info.self_color) {
                        blue_point[i] = cv::Point2f(field_width_ - msg->blue_x[i],
                                                    field_height_ - msg->blue_y[i]);
                    }
                    blue_update[i] = time;
                }
            }
            if (msg->red_x[i] * msg->red_y[i]) {
                if (time - red_time[i] > 5) {
                    red_point[i] =
                        cv::Point2f(msg->red_x[i], msg->red_y[i]);
                    if (!match_info.self_color) {
                        red_point[i] = cv::Point2f(field_width_ - msg->red_x[i],
                                                   field_height_ - msg->red_y[i]);
                    }
                    red_update[i] = time;
                }
            }
        }
        show_map();
    }

    // kalman融合结果回调：更新地图位置，判断英雄机器人进退场，发送Radar2Sentry信息
    void
    callback(const std::shared_ptr<vision_interface::msg::DetectResult> msg)
    {
        auto   now = std::chrono::system_clock::now();
        double time = std::chrono::duration_cast<std::chrono::milliseconds>(
                          now.time_since_epoch())
                          .count() /
                      1000.0;
        for (int i = 0; i < 6; i++) {
            if (msg->blue_x[i] * msg->blue_y[i]) {
                blue_point[i] = cv::Point2f(msg->blue_x[i], msg->blue_y[i]);
                blue_time[i] = time;
                blue_update[i] = time;
            }
            if (msg->red_x[i] * msg->red_y[i]) {
                red_point[i] = cv::Point2f(msg->red_x[i], msg->red_y[i]);
                red_time[i] = time;
                red_update[i] = time;
            }
        }
        show_map();
        vision_interface::msg::RadarWarn radar_warn;
        if (hero_count1 > 10) {
            radar_warn.hero_state = 1;
            radar_warn_pub->publish(radar_warn);
        } else if (hero_count2 > 10) {
            radar_warn.hero_state = 2;
            radar_warn_pub->publish(radar_warn);
        }
        if (match_info.self_color == 0) {
            if (red_point[0].x > (field_width_ - 8.668)) {
                hero_count1++;
                hero_count2--;
            } else if (red_point[0].x < (field_width_ - 20.3) &&
                       red_point[0].x > (field_width_ - 25.075) &&
                       red_point[0].y < field_height_ && red_point[0].y > 10.3) {
                hero_count1--;
                hero_count2++;
            } else {
                hero_count1--;
                hero_count2--;
            }
        }
        if (match_info.self_color == 2) {
            if (blue_point[0].x < 8.668) {
                hero_count1++;
                hero_count2--;
            } else if (blue_point[0].x > 20.3 && blue_point[0].x < 25.075 &&
                       blue_point[0].y > 0 &&
                       blue_point[0].y < (field_height_ - 10.3)) {
                hero_count1--;
                hero_count2++;
            } else {
                hero_count1--;
                hero_count2--;
            }
        }

        vision_interface::msg::Radar2Sentry radar2sentry;
        if (match_info.self_color == 0) {
            for (int i = 0; i < 6; i++) {
                if (!relax[i]) {
                    if (match_info.marks[i] >= 117) {
                        relax[i] = true;
                        relax_time[i] = time;
                    } else {
                        if (time - red_update[i] < 0.5) {
                            radar2sentry.radar_enemy_x[i] = red_point[i].x;
                            radar2sentry.radar_enemy_y[i] = red_point[i].y;
                        }
                    }
                } else {
                    if (match_info.marks[i] < 105) {
                        relax[i] = false;
                        if (time - red_update[i] < 0.5) {
                            radar2sentry.radar_enemy_x[i] = red_point[i].x;
                            radar2sentry.radar_enemy_y[i] = red_point[i].y;
                        }
                    } else if (time - relax_time[i] > 0.35) {
                        relax_time[i] = time;
                        if (time - red_update[i] < 0.5) {
                            radar2sentry.radar_enemy_x[i] = red_point[i].x;
                            radar2sentry.radar_enemy_y[i] = red_point[i].y;
                        }
                    }
                }
            }
        }
        if (match_info.self_color == 2) {
            for (int i = 0; i < 6; i++) {
                if (!relax[i]) {
                    if (match_info.marks[i] >= 117) {
                        relax[i] = true;
                        relax_time[i] = time;
                    } else {
                        if (time - blue_update[i] < 0.5) {
                            radar2sentry.radar_enemy_x[i] = blue_point[i].x;
                            radar2sentry.radar_enemy_y[i] = blue_point[i].y;
                        }
                    }
                } else {
                    if (match_info.marks[i] < 105) {
                        relax[i] = false;
                        if (time - blue_update[i] < 0.5) {
                            radar2sentry.radar_enemy_x[i] = blue_point[i].x;
                            radar2sentry.radar_enemy_y[i] = blue_point[i].y;
                        }
                    } else if (time - relax_time[i] > 0.35) {
                        relax_time[i] = time;
                        if (time - blue_update[i] < 0.5) {
                            radar2sentry.radar_enemy_x[i] = blue_point[i].x;
                            radar2sentry.radar_enemy_y[i] = blue_point[i].y;
                        }
                    }
                }
            }
        }
        radar2sentry_pub->publish(radar2sentry);
    }
    rclcpp::Subscription<vision_interface::msg::DetectResult>::SharedPtr
        detect_result_sub;
    rclcpp::Subscription<vision_interface::msg::DetectResult>::SharedPtr
                                                          camera_detect_sub;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr debug_map_pub;
    rclcpp::Publisher<vision_interface::msg::RadarWarn>::SharedPtr
        radar_warn_pub;
    rclcpp::Publisher<vision_interface::msg::Radar2Sentry>::SharedPtr
        radar2sentry_pub;
    rclcpp::Subscription<vision_interface::msg::MatchInfo>::SharedPtr
        match_info_sub;

    double blue_time[6];
    double red_time[6];

    bool   relax[6];
    double relax_time[6];

    double blue_update[6];
    double red_update[6];

    int hero_count1;
    int hero_count2;

    cv::Point2f blue_point[6];
    cv::Point2f red_point[6];

    vision_interface::msg::MatchInfo match_info;
    cv::Mat                          map;
    int                              count = 0;
    float                            field_width_ = 28.0;
    float                            field_height_ = 15.0;
};
}  // namespace tdt_radar

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node_options = rclcpp::NodeOptions();
    rclcpp::spin(std::make_shared<tdt_radar::DebugMap>(node_options));
    rclcpp::shutdown();
    return 0;
}
