// radar_utils.h：坐标解算工具头文件
// parser类：2D图像坐标到3D场地坐标的转换
// Parser_Points类：管理场地多边形区域（道路、要塞等）的3D点云和2D投影
#ifndef RADAR_UTILS_H
#define RADAR_UTILS_H
#include <iostream>
#include <string>
#include "vector"
#include <opencv2/opencv.hpp>

namespace tdt_radar
{
    class Parser_Points
    {
    public:
        
        Parser_Points(const std::string& points_name);
        float return_height(cv::Point2f &input_point);
        void Update();
        void World2Camera();
        std::vector<cv::Point3f> ReadPoints(const std::string &points_name);
        std::vector<cv::Point> Float2Int(std::vector<cv::Point2f> &FloatPoint);
        std::vector<cv::Point3f> Points_3D;
        std::vector<cv::Point> Points_2D;
        float Height=0;
    private:
        cv::Mat world_rvec;
        cv::Mat world_tvec;
        cv::Mat camera_matrix;
        cv::Mat dist_coeffs;
    };
    class parser
    {
    public:
        float get_elevation(float field_x, float field_y);
        parser();
        void Change_Matrix();
        cv::Point2f parse(cv::Point2f &input_point);
        void draw_ui(cv::Mat &img);
        float get_height(cv::Point2f &input_point);
        cv::Point2f get_2d(cv::Point2f &input_point,float height);
        cv::Mat world_rvec;
        cv::Mat world_tvec;
        cv::Mat camera_matrix;
        cv::Mat dist_coeffs;
        std::map <std::string,Parser_Points*> points_map;

        // 高程图配置（从 elevation_meta.yaml 读取）
        cv::Mat elevation_map_;
        float   field_width_ = 28.0f;
        float   field_height_ = 15.0f;
        float   z_min_ = -0.187180f;
        float   z_max_ = 1.969253f;
        int     no_data_ = 255;
    };
}
#endif //RADAR_UTILS_H
