/**
 * @file generate_map.cpp
 * @brief 从已保存的PCD序列生成拼接地图
 * 
 * 功能:
 *   1. 读取指定目录下的所有PCD文件
 *   2. 从每个PCD的VIEWPOINT字段读取位姿
 *   3. 变换并拼接所有点云
 *   4. 输出合并后的地图PCD
 * 
 * 使用方法:
 *   rosrun rosbag2pcd generate_map \
 *       _input_dir:=/path/to/pcd_folder \
 *       _output_file:=/path/to/map.pcd \
 *       _voxel_size:=0.1
 */

#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>

#include <Eigen/Dense>
#include <fstream>
#include <sstream>
#include <vector>
#include <algorithm>
#include <filesystem>

namespace fs = std::filesystem;

//=============================================================================
// 从PCD文件读取VIEWPOINT
//=============================================================================
bool readViewpoint(const std::string& filepath, Eigen::Matrix4d& transform) {
    std::ifstream file(filepath);
    if (!file.is_open()) return false;
    
    std::string line;
    while (std::getline(file, line)) {
        if (line.find("VIEWPOINT") != std::string::npos) {
            std::istringstream iss(line);
            std::string keyword;
            double tx, ty, tz, qw, qx, qy, qz;
            iss >> keyword >> tx >> ty >> tz >> qw >> qx >> qy >> qz;
            
            Eigen::Quaterniond q(qw, qx, qy, qz);
            q.normalize();
            
            transform = Eigen::Matrix4d::Identity();
            transform.block<3,3>(0,0) = q.toRotationMatrix();
            transform(0,3) = tx;
            transform(1,3) = ty;
            transform(2,3) = tz;
            
            return true;
        }
        if (line.find("DATA") != std::string::npos) break;
    }
    
    // 默认单位变换
    transform = Eigen::Matrix4d::Identity();
    return true;
}

//=============================================================================
// 主函数
//=============================================================================
int main(int argc, char** argv) {
    ros::init(argc, argv, "generate_map");
    ros::NodeHandle nh("~");
    
    // 获取参数
    std::string input_dir, output_file;
    double voxel_size;
    bool apply_filter;
    
    nh.param<std::string>("input_dir", input_dir, "");
    nh.param<std::string>("output_file", output_file, "map.pcd");
    nh.param<double>("voxel_size", voxel_size, 0.1);
    nh.param<bool>("apply_filter", apply_filter, true);
    
    if (input_dir.empty()) {
        ROS_ERROR("Please specify input_dir parameter!");
        ROS_INFO("Usage: rosrun rosbag2pcd generate_map _input_dir:=/path/to/pcds _output_file:=map.pcd");
        return 1;
    }
    
    ROS_INFO("===========================================");
    ROS_INFO("         Map Generator from PCDs");
    ROS_INFO("===========================================");
    ROS_INFO("Input dir:   %s", input_dir.c_str());
    ROS_INFO("Output file: %s", output_file.c_str());
    ROS_INFO("Voxel size:  %.3f m", voxel_size);
    ROS_INFO("===========================================");
    
    // 收集所有PCD文件
    std::vector<std::string> pcd_files;
    for (const auto& entry : fs::directory_iterator(input_dir)) {
        if (entry.path().extension() == ".pcd") {
            std::string filename = entry.path().filename().string();
            // 排除map文件
            if (filename.find("map") == std::string::npos) {
                pcd_files.push_back(entry.path().string());
            }
        }
    }
    
    std::sort(pcd_files.begin(), pcd_files.end());
    
    if (pcd_files.empty()) {
        ROS_ERROR("No PCD files found in %s", input_dir.c_str());
        return 1;
    }
    
    ROS_INFO("Found %zu PCD files", pcd_files.size());
    
    // 拼接点云
    pcl::PointCloud<pcl::PointXYZI>::Ptr global_map(new pcl::PointCloud<pcl::PointXYZI>());
    int processed = 0;
    
    for (const auto& pcd_file : pcd_files) {
        // 读取点云
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>());
        if (pcl::io::loadPCDFile<pcl::PointXYZI>(pcd_file, *cloud) == -1) {
            ROS_WARN("Failed to load: %s", pcd_file.c_str());
            continue;
        }
        
        // 读取位姿
        Eigen::Matrix4d transform;
        readViewpoint(pcd_file, transform);
        
        // 变换并添加
        pcl::PointCloud<pcl::PointXYZI>::Ptr transformed(new pcl::PointCloud<pcl::PointXYZI>());
        pcl::transformPointCloud(*cloud, *transformed, transform);
        *global_map += *transformed;
        
        processed++;
        if (processed % 50 == 0) {
            ROS_INFO("  Processed %d/%zu files, map size: %zu points",
                     processed, pcd_files.size(), global_map->size());
        }
    }
    
    ROS_INFO("Total map points before filtering: %zu", global_map->size());
    
    // 体素滤波
    pcl::PointCloud<pcl::PointXYZI>::Ptr final_map = global_map;
    
    if (apply_filter && !global_map->empty()) {
        ROS_INFO("Applying voxel filter...");
        
        pcl::VoxelGrid<pcl::PointXYZI> vg;
        vg.setInputCloud(global_map);
        vg.setLeafSize(voxel_size, voxel_size, voxel_size);
        
        final_map.reset(new pcl::PointCloud<pcl::PointXYZI>());
        vg.filter(*final_map);
        
        ROS_INFO("Map points after filtering: %zu", final_map->size());
    }
    
    // 保存地图
    if (!final_map->empty()) {
        if (pcl::io::savePCDFileBinary(output_file, *final_map) == 0) {
            ROS_INFO("===========================================");
            ROS_INFO("Map saved successfully!");
            ROS_INFO("  Output: %s", output_file.c_str());
            ROS_INFO("  Points: %zu", final_map->size());
            ROS_INFO("===========================================");
        } else {
            ROS_ERROR("Failed to save map!");
            return 1;
        }
    } else {
        ROS_ERROR("Map is empty!");
        return 1;
    }
    
    return 0;
}
