/* PCD → 场地配置文件 (field_geom.yaml + field_map.png)
 * 用法: ./pcd_to_config <input.pcd> <output.yaml> <output.png>
 *
 * ─────────────────────────────────────────────────────────────
 * 一、画图 (field_map.png)
 *    用「预处理后的全点云（不聚类）」直接投影到 2D 图像
 *
 *    for (遍历每个点) {
 *        // 点坐标 (x, y, z) → 像素坐标 (u, v)
 *        u = (p.x - minPt.x) / resolution;
 *        v = rows - 1 - (p.y - minPt.y) / resolution;  // y 翻转
 *
 *        // 每个像素只保留「最高」的点（避免物体被地面覆盖）
 *        height_img.at<float>(v, u) = max(p.z, 之前的值)
 *    }
 *
 *    然后遍历 height_img 每个像素，按 z 值上色：
 *        z=0      深灰（无数据）
 *        z<0.08   灰色（地面）
 *        z<0.18   绿色（要塞）
 *        z<0.25   青色（道路）
 *        z<0.45   蓝色（中线）
 *        z>=0.45  红色（增益区、补给区）
 *
 * ─────────────────────────────────────────────────────────────
 * 二、场地尺寸 (field_size_x, field_size_y)
 *    RANSAC 拟合地面 → 地面点 → getMinMax3D → X/Y 范围
 *
 * ─────────────────────────────────────────────────────────────
 * 三、区域 (field_geom.yaml 多边形 + height)
 *    RANSAC 的非地面点 → 欧式聚类 →
 *    每个 cluster:
 *        凸包 (ConvexHull) → 多边形顶点 [{x,y},{x,y},...]
 *        z 均值 → height
 *        中心位置 → 匹配语义名（己方要塞、敌方要塞、道路……）
 *
 *    写 YAML:
 *        field_size_x: 28.0
 *        field_size_y: 15.0
 *        Region_Name:
 *          height: 0.15
 *          points: [ {x:..., y:...}, {x:..., y:...}, ... ]
 */
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl/common/common.h>
#include <pcl/common/centroid.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>

#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

#include <pcl/surface/convex_hull.h>

#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>

#include <opencv2/opencv.hpp>

#include <yaml-cpp/yaml.h>

#include <iostream>
#include <string>
#include <vector>
#include <algorithm>
#include <cmath>

#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>

using PointT = pcl::PointXYZ;
using Cloud = pcl::PointCloud<PointT>;
using CloudPtr = Cloud::Ptr;
using CloudConstPtr = Cloud::ConstPtr;





struct ClusterResult {
    std::vector<CloudPtr> clusters;
    std::vector<PointT> centers;
};


struct Region {
    std::string name;
    float height;
    std::vector<std::pair<float, float>> polygon; // 凸包顶点 (x, y)
    float center_x, center_y;
};


//这里做的这个是来过滤大部分的不合格的这个点云数据的
CloudPtr preprocess(const CloudConstPtr& input) {

    auto cloud = std::make_shared<Cloud>(*input);

    //这个必须要做，不然报错
    
    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(*cloud, *cloud, indices);

    //这里不做这个直通滤波的处理，为了做这个适配大部分场景就不做这个了，
    //通过生成了相关的数据之后再做人为的裁减就可以了，或者是之后加上在这里

    pcl::PassThrough<PointT> pass;
    pass.setInputCloud(cloud);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(0.0, 2.0);
    pass.filter(*cloud);

    pcl::VoxelGrid<PointT> voxel;
    voxel.setInputCloud(cloud);
    voxel.setLeafSize(0.1f, 0.1f, 0.1f);
    voxel.filter(*cloud);

    pcl::StatisticalOutlierRemoval<PointT> sor;
    sor.setInputCloud(cloud);
    sor.setMeanK(50);
    sor.setStddevMulThresh(1.0);
    sor.filter(*cloud);

    return cloud;
}


//这里按照一般的流程处理的话是需要做这个动态监测，不过我这里是做的pcd的数据来反映这个场地大小
//尺寸数据用来适配大部分场地的所以不应该这样做动态监测我这里处理的是pcd的数据


//这里下面做的这个是点云的一个提取，将他的这个数据量减少一下，并且确定一下这个真实的物体的点云

//这里使用这个欧式聚类来做这个

ClusterResult euclideanCluster(const CloudConstPtr& cloud,
                               float tolerance = 0.25f, int min_size = 5,
                               int max_size = 10000)
{
    ClusterResult result;

    // 构建 KdTree（聚类算法需要它来快速找邻居）
    pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
    tree->setInputCloud(cloud);

    // 欧式聚类
    pcl::EuclideanClusterExtraction<PointT> ec;
    ec.setClusterTolerance(tolerance);  // 同簇最大间距（米）
    ec.setMinClusterSize(min_size);     // 最少点数（过滤小噪点团）
    ec.setMaxClusterSize(max_size);     // 最多点数（过滤超大物体）
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud);

    // 执行聚类，结果保存在 cluster_indices 中

    std::vector<pcl::PointIndices> cluster_indices;
    ec.extract(cluster_indices);

    for (const auto& indices : cluster_indices) {
        auto cluster_cloud = std::make_shared<Cloud>();
        PointT center(0, 0, 0);

        for (int idx : indices.indices) {

            cluster_cloud->push_back(cloud->points[idx]);
            center.x += cloud->points[idx].x;
            center.y += cloud->points[idx].y;
            center.z += cloud->points[idx].z;

        }

        center.x /= indices.indices.size();
        center.y /= indices.indices.size();
        center.z /= indices.indices.size();

        result.clusters.push_back(cluster_cloud);
        result.centers.push_back(center);

        

    }

    return result;

    }


//这个这里已经将这个在场上大概代表实际物体的点提取出来了









int main(int argc, char** argv) {


    if (argc < 4) {
        std::cout << "Usage: " << argv[0] << " <input.pcd> <output.yaml> <output.png>" << std::endl;
        return 1;
    }





























    return 0;
}
