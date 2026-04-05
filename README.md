# rosbag2pcd

ROS包：将rosbag中的点云数据转换为PCD文件，并生成拼接地图。

## 功能

1. **逐帧转换**: 将`/rslidar_points`话题的点云逐帧保存为PCD文件
2. **位姿写入**: 每个PCD的`VIEWPOINT`字段存储对应的位姿 (`tx ty tz qw qx qy qz`)
3. **地图拼接**: 根据位姿将所有帧拼接成完整地图

## 文件结构

```
rosbag2pcd/
├── CMakeLists.txt
├── package.xml
├── README.md
├── launch/
│   ├── bag_to_pcd.launch        # 离线转换launch
│   └── realtime_to_pcd.launch   # 实时转换launch
└── src/
    ├── bag_to_pcd_offline.cpp   # 离线转换工具（推荐）
    ├── pointcloud_to_pcd_node.cpp # 实时订阅转换节点
    └── generate_map.cpp         # 从PCD生成地图工具
```

## 依赖

- ROS (Melodic/Noetic)
- PCL
- Eigen3

```bash
sudo apt install ros-${ROS_DISTRO}-pcl-ros ros-${ROS_DISTRO}-pcl-conversions
```

## 编译

```bash
# 进入catkin工作空间
cd ~/catkin_ws/src

# 复制或链接此包
cp -r rosbag2pcd ./
# 或
ln -s /path/to/rosbag2pcd ./

# 编译
cd ~/catkin_ws
catkin_make
# 或
catkin build rosbag2pcd

# 刷新环境
source devel/setup.bash
```

## 输入数据格式

### 位姿文件 (poses.txt)

支持三种格式：

**格式1** (默认，推荐): `timestamp tx ty tz qw qx qy qz`
```
1623456789.123456 0.0 0.0 0.0 1.0 0.0 0.0 0.0
1623456789.223456 0.1 0.0 0.0 0.9999 0.0 0.0 0.01
1623456789.323456 0.2 0.0 0.0 0.9998 0.0 0.0 0.02
...
```

**格式2**: `timestamp tx ty tz qx qy qz qw`
```
1623456789.123456 0.0 0.0 0.0 0.0 0.0 0.0 1.0
...
```

**格式3** (无时间戳，按行号对应帧): `tx ty tz qw qx qy qz`
```
0.0 0.0 0.0 1.0 0.0 0.0 0.0
0.1 0.0 0.0 0.9999 0.0 0.0 0.01
0.2 0.0 0.0 0.9998 0.0 0.0 0.02
...
```

**说明**:
- `tx ty tz`: 平移向量 (米)
- `qw qx qy qz`: 四元数 (w分量在前或在后取决于格式)
- 行数应与bag中的点云帧数一致（格式3）或时间戳匹配（格式1/2）

## 使用方法

### 方法1：离线转换（推荐）

直接从rosbag文件读取，无需播放bag：

```bash
# 使用rosrun
rosrun rosbag2pcd bag_to_pcd_offline \
    _bag_file:=/path/to/data.bag \
    _pose_file:=/path/to/poses.txt \
    _output_dir:=/path/to/output \
    _topic:=/rslidar_points \
    _map_file:=map.pcd \
    _pose_format:=1 \
    _voxel_size:=0.1

# 使用launch
roslaunch rosbag2pcd bag_to_pcd.launch \
    bag_file:=/path/to/data.bag \
    pose_file:=/path/to/poses.txt \
    output_dir:=/path/to/output
```

### 方法2：实时订阅转换

先启动节点，再播放bag：

```bash
# 终端1：启动转换节点
roslaunch rosbag2pcd realtime_to_pcd.launch \
    pose_file:=/path/to/poses.txt \
    output_dir:=/path/to/output

# 终端2：播放rosbag
rosbag play /path/to/data.bag
```

### 方法3：从已有PCD生成地图

如果已有PCD文件（带VIEWPOINT），可以单独生成地图：

```bash
rosrun rosbag2pcd generate_map \
    _input_dir:=/path/to/pcd_folder \
    _output_file:=/path/to/map.pcd \
    _voxel_size:=0.1
```

## 参数说明

### bag_to_pcd_offline

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `bag_file` | string | (必需) | rosbag文件路径 |
| `pose_file` | string | (必需) | 位姿txt文件路径 |
| `output_dir` | string | `./pcd_output` | 输出目录 |
| `topic` | string | `/rslidar_points` | 点云话题名 |
| `map_file` | string | `map.pcd` | 输出地图文件名 |
| `pose_format` | int | `1` | 位姿文件格式 (1/2/3) |
| `voxel_size` | double | `0.1` | 地图体素滤波大小 (米) |

### pointcloud_to_pcd_node

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `pose_file` | string | (必需) | 位姿txt文件路径 |
| `output_dir` | string | `./pcd_output` | 输出目录 |
| `topic` | string | `/rslidar_points` | 点云话题名 |
| `map_file` | string | `map.pcd` | 输出地图文件名 |
| `pose_format` | int | `1` | 位姿文件格式 |
| `voxel_size` | double | `0.1` | 地图体素大小 |
| `build_map` | bool | `true` | 是否生成拼接地图 |

## 输出结果

```
output_dir/
├── 000000.pcd    # 第1帧点云
├── 000001.pcd    # 第2帧点云
├── 000002.pcd    # 第3帧点云
├── ...
└── map.pcd       # 拼接后的完整地图
```

### PCD文件格式

每个PCD文件的头部包含VIEWPOINT字段：

```
# .PCD v0.7 - Point Cloud Data file format
VERSION 0.7
FIELDS x y z intensity
SIZE 4 4 4 4
TYPE F F F F
COUNT 1 1 1 1
WIDTH 12345
HEIGHT 1
VIEWPOINT 1.234 5.678 0.123 0.999 0.001 0.002 0.003
POINTS 12345
DATA binary
```

`VIEWPOINT`格式: `tx ty tz qw qx qy qz`

## 示例

### 完整工作流

```bash
# 1. 假设你有以下文件
#    - /data/sensor.bag (包含/rslidar_points话题)
#    - /data/poses.txt (位姿文件)

# 2. 编译ROS包
cd ~/catkin_ws
catkin_make
source devel/setup.bash

# 3. 运行转换
rosrun rosbag2pcd bag_to_pcd_offline \
    _bag_file:=/data/sensor.bag \
    _pose_file:=/data/poses.txt \
    _output_dir:=/data/pcd_output \
    _topic:=/rslidar_points

# 4. 查看输出
ls /data/pcd_output/
# 000000.pcd  000001.pcd  000002.pcd  ...  map.pcd

# 5. 可视化
pcl_viewer /data/pcd_output/map.pcd
```

### 不同位姿格式

```bash
# TUM格式 (timestamp tx ty tz qx qy qz qw)
rosrun rosbag2pcd bag_to_pcd_offline \
    _bag_file:=/data/sensor.bag \
    _pose_file:=/data/tum_poses.txt \
    _output_dir:=/data/output \
    _pose_format:=2

# 无时间戳格式 (tx ty tz qw qx qy qz)
rosrun rosbag2pcd bag_to_pcd_offline \
    _bag_file:=/data/sensor.bag \
    _pose_file:=/data/simple_poses.txt \
    _output_dir:=/data/output \
    _pose_format:=3
```

## 常见问题

### Q: 位姿和点云数量不匹配怎么办？

A: 
- 格式1/2：使用时间戳匹配，会自动找最近的位姿（容差0.5秒）
- 格式3：按行号对应，需确保位姿行数>=点云帧数

### Q: 输出的PCD可以用于RI-DVP吗？

A: 是的，输出的PCD文件格式完全兼容RI-DVP：
- 点云类型: `pcl::PointXYZI`
- VIEWPOINT字段: `tx ty tz qw qx qy qz`

### Q: 如何查看VIEWPOINT内容？

```bash
head -10 000000.pcd
# 或
grep VIEWPOINT 000000.pcd
```

## License

MIT License
