/**
 * @file bag_to_pcd_offline.cpp
 * @brief 离线将rosbag中的点云转换为PCD文件，并生成拼接地图
 * 
 * 功能:
 *   1. 从rosbag读取/rslidar_points话题的点云数据
 *   2. 从txt文件读取每帧位姿 (timestamp tx ty tz qw qx qy qz)
 *   3. 输出逐帧PCD文件，VIEWPOINT字段存储位姿
 *   4. 输出拼接后的完整地图PCD
 * 
 * 使用方法:
 *   rosrun rosbag2pcd bag_to_pcd_offline \
 *       _bag_file:=/path/to/data.bag \
 *       _pose_file:=/path/to/poses.txt \
 *       _output_dir:=/path/to/output \
 *       _topic:=/rslidar_points \
 *       _map_file:=map.pcd
 */

#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
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
#include <map>
#include <algorithm>
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
// 位姿文件读取器
//=============================================================================
class PoseReader {
public:
    /**
     * @brief 从txt文件读取位姿
     * 
     * 支持的格式:
     *   格式1: timestamp tx ty tz qw qx qy qz
     *   格式2: timestamp tx ty tz qx qy qz qw
     *   格式3: tx ty tz qw qx qy qz (无时间戳，按行号对应帧)
     * 
     * @param filepath 位姿文件路径
     * @param format 格式类型 (1, 2, 或 3)
     * @return 是否成功
     */
    bool load(const std::string& filepath, int format = 1) {
        std::ifstream file(filepath);
        if (!file.is_open()) {
            ROS_ERROR("Cannot open pose file: %s", filepath.c_str());
            return false;
        }
        
        poses_.clear();
        std::string line;
        int line_num = 0;
        
        while (std::getline(file, line)) {
            // 跳过空行和注释
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
                pose.timestamp = line_num;  // 使用行号作为索引
            }
            
            pose.rotation.normalize();
            poses_.push_back(pose);
            line_num++;
        }
        
        ROS_INFO("Loaded %zu poses from %s", poses_.size(), filepath.c_str());
        return !poses_.empty();
    }
    
    /**
     * @brief 根据时间戳查找最近的位姿
     */
    bool findPoseByTimestamp(double timestamp, Pose& pose, double tolerance = 0.1) const {
        if (poses_.empty()) return false;
        
        double min_diff = std::numeric_limits<double>::max();
        int best_idx = -1;
        
        for (size_t i = 0; i < poses_.size(); ++i) {
            double diff = std::abs(poses_[i].timestamp - timestamp);
            if (diff < min_diff) {
                min_diff = diff;
                best_idx = static_cast<int>(i);
            }
        }
        
        if (best_idx >= 0 && min_diff < tolerance) {
            pose = poses_[best_idx];
            return true;
        }
        
        return false;
    }
    
    /**
     * @brief 根据索引获取位姿
     */
    bool getPoseByIndex(size_t index, Pose& pose) const {
        if (index < poses_.size()) {
            pose = poses_[index];
            return true;
        }
        return false;
    }
    
    size_t size() const { return poses_.size(); }
    const std::vector<Pose>& poses() const { return poses_; }

private:
    std::vector<Pose> poses_;
};

//=============================================================================
// PCD写入器（支持VIEWPOINT字段）
//=============================================================================
bool savePCDWithViewpoint(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud,
                          const std::string& filepath,
                          const Pose& pose) {
    // 先保存为标准PCD
    if (pcl::io::savePCDFileBinary(filepath, *cloud) == -1) {
        return false;
    }
    
    // 读取文件并修改VIEWPOINT行
    std::ifstream infile(filepath);
    std::stringstream buffer;
    buffer << infile.rdbuf();
    infile.close();
    
    std::string content = buffer.str();
    
    // 构建新的VIEWPOINT行
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
    
    // 替换VIEWPOINT行
    size_t pos = content.find("VIEWPOINT ");
    if (pos != std::string::npos) {
        size_t end = content.find('\n', pos);
        content.replace(pos, end - pos, viewpoint.str());
    }
    
    // 写回文件
    std::ofstream outfile(filepath);
    outfile << content;
    outfile.close();
    
    return true;
}

//=============================================================================
// 主函数
//=============================================================================
int main(int argc, char** argv) {
    ros::init(argc, argv, "bag_to_pcd_offline");
    ros::NodeHandle nh("~");
    
    // 获取参数
    std::string bag_file, pose_file, output_dir, topic, map_file;
    int pose_format;
    double voxel_size;
    bool save_binary;
    
    nh.param<std::string>("bag_file", bag_file, "");
    nh.param<std::string>("pose_file", pose_file, "");
    nh.param<std::string>("output_dir", output_dir, "./pcd_output");
    nh.param<std::string>("topic", topic, "/rslidar_points");
    nh.param<std::string>("map_file", map_file, "map.pcd");
    nh.param<int>("pose_format", pose_format, 1);
    nh.param<double>("voxel_size", voxel_size, 0.1);
    nh.param<bool>("save_binary", save_binary, true);
    
    // 检查参数
    if (bag_file.empty()) {
        ROS_ERROR("Please specify bag_file parameter!");
        ROS_INFO("Usage: rosrun rosbag2pcd bag_to_pcd_offline _bag_file:=/path/to/bag _pose_file:=/path/to/poses.txt _output_dir:=/path/to/output");
        return 1;
    }
    
    if (pose_file.empty()) {
        ROS_ERROR("Please specify pose_file parameter!");
        return 1;
    }
    
    // 打印配置
    ROS_INFO("===========================================");
    ROS_INFO("       ROS Bag to PCD Converter");
    ROS_INFO("===========================================");
    ROS_INFO("Bag file:    %s", bag_file.c_str());
    ROS_INFO("Pose file:   %s", pose_file.c_str());
    ROS_INFO("Output dir:  %s", output_dir.c_str());
    ROS_INFO("Topic:       %s", topic.c_str());
    ROS_INFO("Map file:    %s", map_file.c_str());
    ROS_INFO("Pose format: %d", pose_format);
    ROS_INFO("Voxel size:  %.3f m", voxel_size);
    ROS_INFO("===========================================");
    
    // 创建输出目录
    fs::create_directories(output_dir);
    
    // 读取位姿文件
    PoseReader pose_reader;
    if (!pose_reader.load(pose_file, pose_format)) {
        ROS_ERROR("Failed to load pose file!");
        return 1;
    }
    
    // 打开rosbag
    rosbag::Bag bag;
    try {
        bag.open(bag_file, rosbag::bagmode::Read);
    } catch (const rosbag::BagException& e) {
        ROS_ERROR("Failed to open bag file: %s", e.what());
        return 1;
    }
    
    // 创建视图
    std::vector<std::string> topics = {topic};
    rosbag::View view(bag, rosbag::TopicQuery(topics));
    
    // 全局地图
    pcl::PointCloud<pcl::PointXYZI>::Ptr global_map(new pcl::PointCloud<pcl::PointXYZI>());
    
    // 处理每一帧
    int frame_idx = 0;
    int saved_count = 0;
    
    ROS_INFO("Processing point clouds...");
    
    for (const rosbag::MessageInstance& m : view) {
        sensor_msgs::PointCloud2::ConstPtr msg = m.instantiate<sensor_msgs::PointCloud2>();
        if (!msg) continue;
        
        // 获取时间戳
        double timestamp = msg->header.stamp.toSec();
        
        // 转换为PCL点云
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>());
        pcl::fromROSMsg(*msg, *cloud);
        
        if (cloud->empty()) {
            frame_idx++;
            continue;
        }
        
        // 获取位姿
        Pose pose;
        bool pose_found = false;
        
        // 首先尝试按索引获取
        if (pose_reader.getPoseByIndex(frame_idx, pose)) {
            pose_found = true;
        }
        // 否则按时间戳匹配
        else if (pose_reader.findPoseByTimestamp(timestamp, pose, 0.5)) {
            pose_found = true;
        }
        
        if (!pose_found) {
            ROS_WARN("No pose found for frame %d (timestamp: %.3f), skipping...", 
                     frame_idx, timestamp);
            frame_idx++;
            continue;
        }
        
        // 生成文件名
        std::ostringstream filename;
        filename << output_dir << "/" << std::setfill('0') << std::setw(6) << saved_count << ".pcd";
        
        // 保存PCD（带VIEWPOINT）
        if (savePCDWithViewpoint(cloud, filename.str(), pose)) {
            // 变换并添加到全局地图
            pcl::PointCloud<pcl::PointXYZI>::Ptr transformed(new pcl::PointCloud<pcl::PointXYZI>());
            pcl::transformPointCloud(*cloud, *transformed, pose.toMatrix());
            *global_map += *transformed;
            
            saved_count++;
            
            if (saved_count % 50 == 0) {
                ROS_INFO("  Processed %d frames, global map size: %zu points", 
                         saved_count, global_map->size());
            }
        } else {
            ROS_WARN("Failed to save frame %d", frame_idx);
        }
        
        frame_idx++;
    }
    
    bag.close();
    
    ROS_INFO("===========================================");
    ROS_INFO("Frame processing complete!");
    ROS_INFO("  Total frames in bag: %d", frame_idx);
    ROS_INFO("  Saved PCD files: %d", saved_count);
    ROS_INFO("  Global map points: %zu", global_map->size());
    
    // 对全局地图进行体素滤波
    if (!global_map->empty()) {
        ROS_INFO("Applying voxel filter to global map...");
        
        pcl::VoxelGrid<pcl::PointXYZI> vg;
        vg.setInputCloud(global_map);
        vg.setLeafSize(voxel_size, voxel_size, voxel_size);
        
        pcl::PointCloud<pcl::PointXYZI>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZI>());
        vg.filter(*filtered);
        
        ROS_INFO("  Filtered map points: %zu", filtered->size());
        
        // 保存地图
        std::string map_path = output_dir + "/" + map_file;
        if (pcl::io::savePCDFileBinary(map_path, *filtered) == 0) {
            ROS_INFO("  Map saved to: %s", map_path.c_str());
        } else {
            ROS_ERROR("  Failed to save map!");
        }
    }
    
    ROS_INFO("===========================================");
    ROS_INFO("All done!");
    ROS_INFO("  Output directory: %s", output_dir.c_str());
    ROS_INFO("  PCD files: %s/000000.pcd ~ %s/%06d.pcd", 
             output_dir.c_str(), output_dir.c_str(), saved_count - 1);
    ROS_INFO("  Map file: %s/%s", output_dir.c_str(), map_file.c_str());
    ROS_INFO("===========================================");
    
    return 0;
}
