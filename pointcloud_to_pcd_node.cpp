/**
 * @file pointcloud_to_pcd_node.cpp
 * @brief 实时订阅点云话题并保存为PCD文件
 * 
 * 功能:
 *   1. 实时订阅/rslidar_points话题
 *   2. 从txt文件读取位姿（按时间戳匹配或按帧索引）
 *   3. 保存每帧PCD，VIEWPOINT字段存储位姿
 *   4. 可选实时拼接地图
 * 
 * 使用方法:
 *   rosrun rosbag2pcd pointcloud_to_pcd_node \
 *       _pose_file:=/path/to/poses.txt \
 *       _output_dir:=/path/to/output \
 *       _topic:=/rslidar_points
 * 
 *   # 然后播放rosbag
 *   rosbag play data.bag
 */

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>

#include <Eigen/Dense>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <vector>
#include <mutex>
#include <filesystem>

namespace fs = std::filesystem;

//=============================================================================
// 位姿结构
//=============================================================================
struct Pose {
    double timestamp;
    Eigen::Vector3d translation;
    Eigen::Quaterniond rotation;
    
    Pose() : timestamp(0), 
             translation(Eigen::Vector3d::Zero()),
             rotation(Eigen::Quaterniond::Identity()) {}
    
    Eigen::Matrix4d toMatrix() const {
        Eigen::Matrix4d mat = Eigen::Matrix4d::Identity();
        mat.block<3,3>(0,0) = rotation.toRotationMatrix();
        mat.block<3,1>(0,3) = translation;
        return mat;
    }
};

//=============================================================================
// 全局变量
//=============================================================================
std::vector<Pose> g_poses;
std::string g_output_dir;
std::string g_map_file;
double g_voxel_size;
bool g_build_map;
int g_frame_idx = 0;
int g_pose_format;

pcl::PointCloud<pcl::PointXYZI>::Ptr g_global_map(new pcl::PointCloud<pcl::PointXYZI>());
std::mutex g_map_mutex;

//=============================================================================
// 读取位姿文件
//=============================================================================
bool loadPoses(const std::string& filepath, int format) {
    std::ifstream file(filepath);
    if (!file.is_open()) {
        ROS_ERROR("Cannot open pose file: %s", filepath.c_str());
        return false;
    }
    
    g_poses.clear();
    std::string line;
    int line_num = 0;
    
    while (std::getline(file, line)) {
        if (line.empty() || line[0] == '#') continue;
        
        std::istringstream iss(line);
        Pose pose;
        
        if (format == 1) {
            // timestamp tx ty tz qw qx qy qz
            double qw, qx, qy, qz;
            iss >> pose.timestamp 
                >> pose.translation.x() >> pose.translation.y() >> pose.translation.z()
                >> qw >> qx >> qy >> qz;
            pose.rotation = Eigen::Quaterniond(qw, qx, qy, qz);
        } 
        else if (format == 2) {
            // timestamp tx ty tz qx qy qz qw
            double qw, qx, qy, qz;
            iss >> pose.timestamp 
                >> pose.translation.x() >> pose.translation.y() >> pose.translation.z()
                >> qx >> qy >> qz >> qw;
            pose.rotation = Eigen::Quaterniond(qw, qx, qy, qz);
        }
        else if (format == 3) {
            // tx ty tz qw qx qy qz (无时间戳)
            double qw, qx, qy, qz;
            iss >> pose.translation.x() >> pose.translation.y() >> pose.translation.z()
                >> qw >> qx >> qy >> qz;
            pose.rotation = Eigen::Quaterniond(qw, qx, qy, qz);
            pose.timestamp = line_num;
        }
        
        pose.rotation.normalize();
        g_poses.push_back(pose);
        line_num++;
    }
    
    ROS_INFO("Loaded %zu poses", g_poses.size());
    return !g_poses.empty();
}

//=============================================================================
// 保存PCD（带VIEWPOINT）
//=============================================================================
bool savePCDWithViewpoint(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud,
                          const std::string& filepath,
                          const Pose& pose) {
    if (pcl::io::savePCDFileBinary(filepath, *cloud) == -1) {
        return false;
    }
    
    // 读取并修改VIEWPOINT
    std::ifstream infile(filepath);
    std::stringstream buffer;
    buffer << infile.rdbuf();
    infile.close();
    
    std::string content = buffer.str();
    
    std::ostringstream viewpoint;
    viewpoint << std::fixed << std::setprecision(8)
              << "VIEWPOINT "
              << pose.translation.x() << " "
              << pose.translation.y() << " "
              << pose.translation.z() << " "
              << pose.rotation.w() << " "
              << pose.rotation.x() << " "
              << pose.rotation.y() << " "
              << pose.rotation.z();
    
    size_t pos = content.find("VIEWPOINT ");
    if (pos != std::string::npos) {
        size_t end = content.find('\n', pos);
        content.replace(pos, end - pos, viewpoint.str());
    }
    
    std::ofstream outfile(filepath);
    outfile << content;
    outfile.close();
    
    return true;
}

//=============================================================================
// 点云回调函数
//=============================================================================
void pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg) {
    // 转换为PCL
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::fromROSMsg(*msg, *cloud);
    
    if (cloud->empty()) {
        g_frame_idx++;
        return;
    }
    
    // 获取位姿
    Pose pose;
    if (g_frame_idx < static_cast<int>(g_poses.size())) {
        pose = g_poses[g_frame_idx];
    } else {
        ROS_WARN("No more poses available! Frame %d", g_frame_idx);
        g_frame_idx++;
        return;
    }
    
    // 生成文件名并保存
    std::ostringstream filename;
    filename << g_output_dir << "/" << std::setfill('0') << std::setw(6) << g_frame_idx << ".pcd";
    
    if (savePCDWithViewpoint(cloud, filename.str(), pose)) {
        ROS_INFO("Saved frame %d: %s", g_frame_idx, filename.str().c_str());
        
        // 拼接地图
        if (g_build_map) {
            std::lock_guard<std::mutex> lock(g_map_mutex);
            pcl::PointCloud<pcl::PointXYZI>::Ptr transformed(new pcl::PointCloud<pcl::PointXYZI>());
            pcl::transformPointCloud(*cloud, *transformed, pose.toMatrix());
            *g_global_map += *transformed;
        }
    }
    
    g_frame_idx++;
}

//=============================================================================
// 保存地图
//=============================================================================
void saveMap() {
    if (g_global_map->empty()) {
        ROS_WARN("Global map is empty, nothing to save.");
        return;
    }
    
    ROS_INFO("Saving global map...");
    
    // 体素滤波
    pcl::VoxelGrid<pcl::PointXYZI> vg;
    vg.setInputCloud(g_global_map);
    vg.setLeafSize(g_voxel_size, g_voxel_size, g_voxel_size);
    
    pcl::PointCloud<pcl::PointXYZI>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZI>());
    vg.filter(*filtered);
    
    std::string map_path = g_output_dir + "/" + g_map_file;
    if (pcl::io::savePCDFileBinary(map_path, *filtered) == 0) {
        ROS_INFO("Map saved to: %s (%zu points)", map_path.c_str(), filtered->size());
    } else {
        ROS_ERROR("Failed to save map!");
    }
}

//=============================================================================
// 主函数
//=============================================================================
int main(int argc, char** argv) {
    ros::init(argc, argv, "pointcloud_to_pcd_node");
    ros::NodeHandle nh("~");
    
    // 获取参数
    std::string pose_file, topic;
    
    nh.param<std::string>("pose_file", pose_file, "");
    nh.param<std::string>("output_dir", g_output_dir, "./pcd_output");
    nh.param<std::string>("topic", topic, "/rslidar_points");
    nh.param<std::string>("map_file", g_map_file, "map.pcd");
    nh.param<int>("pose_format", g_pose_format, 1);
    nh.param<double>("voxel_size", g_voxel_size, 0.1);
    nh.param<bool>("build_map", g_build_map, true);
    
    if (pose_file.empty()) {
        ROS_ERROR("Please specify pose_file parameter!");
        return 1;
    }
    
    // 打印配置
    ROS_INFO("===========================================");
    ROS_INFO("   PointCloud to PCD Node (Real-time)");
    ROS_INFO("===========================================");
    ROS_INFO("Pose file:   %s", pose_file.c_str());
    ROS_INFO("Output dir:  %s", g_output_dir.c_str());
    ROS_INFO("Topic:       %s", topic.c_str());
    ROS_INFO("Build map:   %s", g_build_map ? "true" : "false");
    ROS_INFO("===========================================");
    
    // 创建输出目录
    fs::create_directories(g_output_dir);
    
    // 读取位姿
    if (!loadPoses(pose_file, g_pose_format)) {
        return 1;
    }
    
    // 订阅话题
    ros::Subscriber sub = nh.subscribe(topic, 100, pointCloudCallback);
    
    ROS_INFO("Waiting for point cloud messages on %s ...", topic.c_str());
    ROS_INFO("(Play your rosbag now: rosbag play xxx.bag)");
    
    ros::spin();
    
    // 保存地图
    if (g_build_map) {
        saveMap();
    }
    
    ROS_INFO("===========================================");
    ROS_INFO("Done! Saved %d frames.", g_frame_idx);
    ROS_INFO("===========================================");
    
    return 0;
}
