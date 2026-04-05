cmake_minimum_required(VERSION 3.0.2)
project(rosbag2pcd)

# C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE Release)

# 查找catkin依赖
find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  pcl_ros
  pcl_conversions
  rosbag
  tf2
  tf2_ros
)

# 查找PCL
find_package(PCL 1.8 REQUIRED)

catkin_package(
  CATKIN_DEPENDS 
    roscpp 
    sensor_msgs 
    pcl_ros 
    pcl_conversions 
    rosbag
    tf2
    tf2_ros
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# 可执行文件1: rosbag离线转换工具
add_executable(bag_to_pcd_offline src/bag_to_pcd_offline.cpp)
target_link_libraries(bag_to_pcd_offline
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

# 可执行文件2: 实时订阅转换节点
add_executable(pointcloud_to_pcd_node src/pointcloud_to_pcd_node.cpp)
target_link_libraries(pointcloud_to_pcd_node
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

# 可执行文件3: 地图生成工具（从已保存的PCD生成）
add_executable(generate_map src/generate_map.cpp)
target_link_libraries(generate_map
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

# 安装
install(TARGETS 
  bag_to_pcd_offline 
  pointcloud_to_pcd_node
  generate_map
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
