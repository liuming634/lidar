// 【radar_utils.cpp】2D→3D坐标透视变换解算工具
// parser类: 利用相机外参(rvec/tvec)+内参做cv::projectPoints→cv::getPerspectiveTransform
//    →cv::perspectiveTransform，将图像2D坐标映射到场地水平面坐标
//    parse()流程: 先判断点落在哪个场地区域(道路/要塞/高地)→获取该区域高度→用对应高度做透视变换
// Parser_Points类: 从RM2025_Points.yaml读取各区域(道路/要塞等)的3D多边形顶点
//   通过World2Camera()投影到图像平面，用于点归属判断(pointPolygonTest)和UI绘制
#include "radar_utils.h"
#include <iostream>
#include <vector>
#include <opencv2/opencv.hpp>

using namespace cv;
using namespace std;

namespace tdt_radar {
bool isPointInsideScreen(cv::Point2f point, int screenWidth,
                         int screenHeight)
{
    return point.x >= 0 && point.x <= screenWidth && point.y >= 0 &&
           point.y <= screenHeight;
}
// parser构造函数：加载外参(rvec/tvec)和内参(camera_matrix/dist_coeffs)，初始化场地道路区域
parser::parser()
{
    cv::FileStorage fs;
    fs.open("./config/out_matrix.yaml", cv::FileStorage::READ);
    fs["world_tvec"] >> this->world_tvec;
    fs["world_rvec"] >> this->world_rvec;
    std::cout << world_tvec << std::endl;
    std::cout << world_rvec << std::endl;
    fs.release();

    cv::FileStorage fs1;
    fs1.open("./config/camera_params.yaml", cv::FileStorage::READ);
    fs1["camera_matrix"] >> this->camera_matrix;
    fs1["dist_coeffs"] >> this->dist_coeffs;
    fs1.release();
    std::cout << "Resolve camera" << camera_matrix << std::endl;
    std::cout << "Resolve dist" << dist_coeffs << std::endl;

    // 读取统一配置文件
    {
        cv::FileStorage fs_rc;
        fs_rc.open("./config/radar_config.yaml", cv::FileStorage::READ);
        if (fs_rc.isOpened()) {
            // 场地基本尺寸
            fs_rc["field_width"] >> field_width_;
            fs_rc["field_height"] >> field_height_;

            // 透视变换参考点
            cv::FileNode per = fs_rc["perspective"];
            per["armor_height"] >> armor_height_;
            cv::FileNode r1 = per["ref1"]; r1["x"] >> ref_x_[0]; r1["y"] >> ref_y_[0];
            cv::FileNode r2 = per["ref2"]; r2["x"] >> ref_x_[1]; r2["y"] >> ref_y_[1];
            cv::FileNode r3 = per["ref3"]; r3["x"] >> ref_x_[2]; r3["y"] >> ref_y_[2];
            cv::FileNode r4 = per["ref4"]; r4["x"] >> ref_x_[3]; r4["y"] >> ref_y_[3];

            // 高程图
            std::string ele_path;
            fs_rc["elevation"]["image"] >> ele_path;
            fs_rc["elevation"]["z_min"] >> z_min_;
            fs_rc["elevation"]["z_max"] >> z_max_;
            fs_rc["elevation"]["no_data"] >> no_data_;
            fs_rc.release();

            elevation_map_ = cv::imread(ele_path, cv::IMREAD_GRAYSCALE);
            if (elevation_map_.empty())
                std::cerr << "[parser] 加载高程图失败: " << ele_path << std::endl;
            else
                std::cout << "[parser] 加载高程图: " << ele_path << " ("
                          << elevation_map_.cols << "x" << elevation_map_.rows << ")" << std::endl;
        } else {
            std::cerr << "[parser] 无法打开 radar_config.yaml，使用默认参数" << std::endl;
            elevation_map_ = cv::imread("./config/elevation/2_elevation_gray256.png", cv::IMREAD_GRAYSCALE);
        }
    }

    // 加载多边形边框（仅用于 draw_ui 可视化，不影响高度计算）
    // 如果 RM2025_Points.yaml 不存在，直接跳过，不报错
    {
        cv::FileStorage fs_check("./config/RM2025_Points.yaml", cv::FileStorage::READ);
        if (fs_check.isOpened()) {
            fs_check.release();
            const char* polygon_regions[] = {
                "Middle_Line", "Left_Road", "Right_Road",
                "Enemy_Buff", "Self_Fortress", "Enemy_Fortress"
            };
            for (int i = 0; i < 6; i++) {
                auto* pp = new Parser_Points(polygon_regions[i]);
                if (!pp->Points_3D.empty()) {
                    points_map[polygon_regions[i]] = pp;
                } else {
                    delete pp;
                }
            }
        }
    }
}
// 更新外参（标定后重新加载），并更新所有区域点
//我这里接受相对于这个图片的位置的像素坐标
float parser::get_elevation(float field_x, float field_y)
{
    if (elevation_map_.empty()) return 0;

    int cx = int(elevation_map_.cols * field_x / field_width_);
    int cy = int(elevation_map_.rows * (field_y + field_height_) / field_height_);

    int x_s = std::max(0, cx - 5);
    int x_e = std::min(elevation_map_.cols - 1, cx + 5);
    int y_s = std::max(0, cy - 5);
    int y_e = std::min(elevation_map_.rows - 1, cy + 5);

    int sum = 0, cnt = 0;
    for (int r = y_s; r <= y_e; r++) {
        for (int c = x_s; c <= x_e; c++) {
            uchar v = elevation_map_.at<uchar>(r, c);
            if ((int)v != no_data_) {
                sum += v;
                cnt++;
            }
        }
    }

    if (cnt == 0) return 0;
    float avg = float(sum) / cnt;
    return z_min_ + (avg / 254.0f) * (z_max_ - z_min_);
}

void parser::Change_Matrix()
{
    cv::FileStorage fs;
    fs.open("./config/out_matrix.yaml", cv::FileStorage::READ);
    fs["world_tvec"] >> this->world_tvec;
    fs["world_rvec"] >> this->world_rvec;
    fs.release();

    for (auto& points : points_map) {
        points.second->Update();
    }
}
// 在图像上绘制场地各区域的多边形边框（仅在 points_map 非空时绘制）
void parser::draw_ui(cv::Mat& img)
{
    for (auto& points : points_map) {
        cv::polylines(img, points.second->Points_2D, true,
                      cv::Scalar(255, 255, 255));
    }
}
// 核心解算函数：先低高度算粗略 (x, y)，再用高程图查真实高度迭代一次
cv::Point2f parser::parse(cv::Point2f& input_point)
{
    cv::Point2f rough = get_2d(input_point, 0);
    float real_height = get_elevation(rough.x, rough.y);
    return get_2d(input_point, real_height);
}
// 判断2D点落在哪个场地区域，返回对应高度
float parser::get_height(cv::Point2f& input_point)
{
    //这里直接返回0,不使用这个方案
    return 0;
}
// 透视变换解算：用PNP投影得到图像和场地间的映射矩阵，再利用perspectiveTransform得到场地坐标
cv::Point2f parser::get_2d(cv::Point2f& input_point, float height)
{
    std::vector<cv::Point3f> world_points;
    for (int i = 0; i < 4; i++) {
        world_points.push_back(cv::Point3f(ref_x_[i], ref_y_[i], armor_height_ + height));
    }
    std::vector<cv::Point2f> image_points;
    cv::projectPoints(world_points, world_rvec, world_tvec, camera_matrix,
                      dist_coeffs, image_points);
    std::vector<cv::Point2f> world_points2D;
    for (int i = 0; i < 4; i++) {
        world_points2D.push_back(cv::Point2f(ref_x_[i], ref_y_[i]));
    }
    cv::Mat Perspective_matrix =
        cv::getPerspectiveTransform(image_points, world_points2D);
    cv::Mat srcPointMat(1, 1, CV_32FC2);
    srcPointMat.at<cv::Point2f>(0, 0) = input_point;
    cv::perspectiveTransform(srcPointMat, srcPointMat, Perspective_matrix);
    return srcPointMat.at<cv::Point2f>(0, 0);
}
std::vector<cv::Point3f>
Parser_Points::ReadPoints(const std::string& points_name)
{
    cv::FileStorage fs("./config/RM2025_Points.yaml",
                       cv::FileStorage::READ);  // 打开YAML文件

    if (!fs.isOpened()) {
        std::cout << "[Parser_Points] 无法打开 RM2025_Points.yaml" << std::endl;
        return {};
    }

    std::vector<cv::Point3f> points;

    cv::FileNode pointsNode = fs[points_name];
    if (pointsNode.type() != cv::FileNode::SEQ) {
        std::cout << "[Parser_Points] " << points_name << " 不是序列" << std::endl;
        return {};
    }

    for (auto&& it : pointsNode) {
        cv::Point3f point;
        it["x"] >> point.x;
        it["y"] >> point.y;
        it["z"] >> point.z;

        points.push_back(point);
    }
    return points;
}
std::vector<cv::Point>
Parser_Points::Float2Int(std::vector<cv::Point2f>& FloatPoint)
{
    std::vector<cv::Point> dstPoint;
    for (auto& i : FloatPoint) {
        dstPoint.emplace_back(int(i.x), int(i.y));
    }
    return dstPoint;
}
void Parser_Points::World2Camera()
{
    std::vector<cv::Point2f> temp_2D;
    cv::projectPoints(Points_3D, world_rvec, world_tvec, camera_matrix,
                      dist_coeffs, temp_2D);
    Points_2D = Float2Int(temp_2D);
}
// Parser_Points构造函数：从yaml读取3D点，计算到2D投影
Parser_Points::Parser_Points(const std::string& points_name)
{
    cv::FileStorage fs;
    fs.open("./config/camera_params.yaml", cv::FileStorage::READ);
    fs["camera_matrix"] >> this->camera_matrix;
    fs["dist_coeffs"] >> this->dist_coeffs;
    fs.release();

    fs.open("./config/out_matrix.yaml", cv::FileStorage::READ);
    fs["world_tvec"] >> this->world_tvec;
    fs["world_rvec"] >> this->world_rvec;
    fs.release();
    std::vector<cv::Point3f> temp_3d = ReadPoints(points_name);
    this->Points_3D = temp_3d;
    World2Camera();
}

float Parser_Points::return_height(cv::Point2f& input_point)
{
    bool inside = false;
    if (cv::pointPolygonTest(
            Points_2D, cv::Point((int)input_point.x, (int)input_point.y),
            false) > 0) {
        return this->Height;
    } else {
        return 0;
    }
}
void Parser_Points::Update()
{
    cv::FileStorage fs;
    fs.open("./config/out_matrix.yaml", cv::FileStorage::READ);
    fs["world_tvec"] >> this->world_tvec;
    fs["world_rvec"] >> this->world_rvec;
    fs.release();
    World2Camera();
}
}  // namespace tdt_radar