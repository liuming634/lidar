// Tool to convert PCD binary_compressed to CSV for Python analysis
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <iostream>
#include <fstream>

int main(int argc, char** argv) {
    if (argc < 3) {
        std::cerr << "Usage: " << argv[0] << " <input.pcd> <output.csv>" << std::endl;
        return 1;
    }

    pcl::PointCloud<pcl::PointXYZRGBNormal> cloud;
    if (pcl::io::loadPCDFile(argv[1], cloud) == -1) {
        pcl::PointCloud<pcl::PointXYZ> cloud_xyz;
        if (pcl::io::loadPCDFile(argv[1], cloud_xyz) == -1) {
            std::cerr << "Failed to load PCD file" << std::endl;
            return 1;
        }
        std::ofstream f(argv[2]);
        f << "x,y,z\n";
        for (const auto& p : cloud_xyz)
            f << p.x << "," << p.y << "," << p.z << "\n";
        std::cout << "Written " << cloud_xyz.size() << " points (XYZ)" << std::endl;
        return 0;
    }

    std::ofstream f(argv[2]);
    f << "x,y,z,nx,ny,nz\n";
    for (const auto& p : cloud)
        f << p.x << "," << p.y << "," << p.z << ","
          << p.normal_x << "," << p.normal_y << "," << p.normal_z << "\n";
    std::cout << "Written " << cloud.size() << " points (XYZ + Normals)" << std::endl;
    return 0;
}
