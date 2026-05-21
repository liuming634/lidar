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

#define ARMOR_HEIGHT 0.15
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

    // 读取高程图配置
    {
        cv::FileStorage fs_ele;
        fs_ele.open("./config/params/elevation_meta.yaml", cv::FileStorage::READ);
        if (fs_ele.isOpened()) {
            std::string ele_path;
            fs_ele["elevation_image"] >> ele_path;
            fs_ele["field_width"] >> field_width_;
            fs_ele["field_height"] >> field_height_;
            fs_ele["z_min"] >> z_min_;
            fs_ele["z_max"] >> z_max_;
            fs_ele["no_data"] >> no_data_;
            fs_ele.release();
            elevation_map_ = cv::imread(ele_path, cv::IMREAD_GRAYSCALE);
            if (elevation_map_.empty())
                std::cerr << "[parser] 加载高程图失败: " << ele_path << std::endl;
            else
                std::cout << "[parser] 加载高程图: " << ele_path << " ("
                          << elevation_map_.cols << "x" << elevation_map_.rows << ")" << std::endl;
        } else {
            std::cerr << "[parser] 无法打开 elevation_meta.yaml，使用默认参数" << std::endl;
            elevation_map_ = cv::imread("./config/elevation/2_elevation_gray256.png", cv::IMREAD_GRAYSCALE);
        }
    }

    points_map["Middle_Line"] = new Parser_Points("Middle_Line");
    points_map["Left_Road"] = new Parser_Points("Left_Road");
    points_map["Right_Road"] = new Parser_Points("Right_Road");
    points_map["Enemy_Buff"] = new Parser_Points("Enemy_Buff");
    points_map["Self_Fortress"] = new Parser_Points("Self_Fortress");
    points_map["Enemy_Fortress"] = new Parser_Points("Enemy_Fortress");

    points_map["Middle_Line"]->Height = 0.3;
    points_map["Left_Road"]->Height = 0.2;
    points_map["Right_Road"]->Height = 0.2;
    points_map["Enemy_Buff"]->Height = 0.6;
    points_map["Self_Fortress"]->Height = 0.15;
    points_map["Enemy_Fortress"]->Height = 0.15;
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
// 在图像上绘制场地各区域的多边形边框
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
    world_points.push_back(cv::Point3f(12, -6, ARMOR_HEIGHT + height));
    world_points.push_back(cv::Point3f(16, -6, ARMOR_HEIGHT + height));
    world_points.push_back(cv::Point3f(16, -8, ARMOR_HEIGHT + height));
    world_points.push_back(cv::Point3f(12, -8, ARMOR_HEIGHT + height));
    std::vector<cv::Point2f> image_points;
    cv::projectPoints(world_points, world_rvec, world_tvec, camera_matrix,
                      dist_coeffs, image_points);
    for (auto& point : image_points) {
    }
    std::vector<cv::Point2f> world_points2D;
    world_points2D.push_back(cv::Point2f(12, -6));
    world_points2D.push_back(cv::Point2f(16, -6));
    world_points2D.push_back(cv::Point2f(16, -8));
    world_points2D.push_back(cv::Point2f(12, -8));
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
        std::cout << "无法打开文件" << std::endl;
        exit(-1);
    }

    std::vector<cv::Point3f> points;

    cv::FileNode pointsNode = fs[points_name];
    if (pointsNode.type() != cv::FileNode::SEQ) {
        std::cout << "points节点不是序列" << std::endl;
        exit(-1);
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