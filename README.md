# RI-DVP: Range Image-based Dynamic Vehicle/Pedestrian Removal Framework

基于范围图像的动态物体移除框架，用于从稀疏LiDAR序列中重建高保真静态地图。

## 算法概述

RI-DVP框架采用"先激进移除，后保守恢复"的双阶段策略：

### A. 物理预处理：强度校准 (Physics-Based Preprocessing)

基于LiDAR测距方程反演，校准原始强度值以获得视角不变的表面反射率：

$$I_{calib} = I_{raw} \cdot \left( \frac{R}{R_{ref}} \right)^{\eta} \cdot \frac{1}{\cos(\alpha)}$$

- `R_ref`: 参考距离 (默认20m)
- `η`: 距离衰减指数 (默认2.0)
- `α`: 入射角（激光束与法向量夹角）

### B. 时空关联：球面投影 (Spatiotemporal Association)

将3D点云投影到2D范围图像，实现密集的像素级比较：
- 当前帧范围图像 `RI_curr`
- 地图范围图像 `RI_map` (Z-buffer渲染)

### C. 双准则特征决策 (Dual-Criterion Feature Decision) - "激进移除"

同时检测几何变化和材质变化：

```
L_cand = 1, if ΔR > τ_dist_high           (结构突变)
L_cand = 1, if (ΔR < τ_dist_low) AND (ΔI > τ_int)  (材质突变)
L_cand = 0, otherwise
```

**创新点**: 材质突变检测可发现几何模糊但材质不同的物体（如黑色车辆停在黑色道路上）。

### D. 拓扑粘附校正 (Topology-Based Adhesion Correction) - "保守恢复"

通过分析候选动态点簇与静态结构的拓扑关系，恢复误判的静态点：
1. 欧几里得聚类
2. 粘附分析（与静态点的距离）
3. 形状分析（PCA线性度）
4. 恢复逻辑：紧贴静态结构的线性/碎片化小簇 → 静态

### E. 时序概率融合 (Temporal Probabilistic Integration)

滑动窗口贝叶斯滤波，只有持续多帧表现为动态的点才被永久移除。

## 编译

### 依赖项
- CMake >= 3.14
- Eigen3 >= 3.3
- C++17 编译器
- (可选) OpenMP

### 编译步骤

```bash
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j$(nproc)
```

## 使用方法

### 演示模式（合成数据）

```bash
./ri_dvp_demo
```

### 处理真实数据

```bash
./ri_dvp_demo /path/to/data num_frames
```

数据目录结构：
```
data/
├── velodyne/
│   ├── 000000.bin
│   ├── 000001.bin
│   └── ...
└── poses.txt  (可选，每行: x y z qx qy qz qw)
```

### API使用

```cpp
#include "ri_dvp.hpp"
using namespace ri_dvp;

// 配置
RIDVPConfig config;
config.tau_dist_high = 0.5;      // 高距离阈值
config.tau_dist_low = 0.1;       // 低距离阈值
config.tau_intensity = 0.15;     // 强度差阈值
config.sliding_window_size = 5;  // 时序窗口

// 创建处理器
RIDVPProcessor processor(config);

// 处理每一帧
for (int i = 0; i < num_frames; ++i) {
    std::vector<PointXYZI> scan = loadPointCloud(...);
    Eigen::Isometry3d pose = loadPose(...);
    
    processor.processFrame(scan, pose);
    
    // 获取结果
    auto dynamic_pts = processor.getDynamicPoints();
    auto static_map = processor.getStaticMap();
}
```

## 参数调优指南

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `tau_dist_high` | 0.5m | 结构突变阈值，降低可检测更多动态物体 |
| `tau_dist_low` | 0.1m | 材质突变的距离上限 |
| `tau_intensity` | 0.15 | 材质突变强度阈值，降低可检测更微小差异 |
| `adhesion_dist_thresh` | 0.05m | 粘附距离阈值，增大可恢复更多误判点 |
| `cluster_tolerance` | 0.3m | 聚类距离，根据场景密度调整 |
| `sliding_window_size` | 5 | 时序窗口，增大可减少噪声但响应变慢 |

## 输出

- `static_map.pcd`: 最终静态地图
- `dynamic_points.pcd`: 当前帧动态点

## 文件结构

```
ri_dvp/
├── CMakeLists.txt
├── README.md
├── include/
│   └── ri_dvp.hpp          # 头文件（类定义、数据结构）
└── src/
    ├── ri_dvp_components.cpp   # 组件实现（RangeImage, VoxelMap等）
    ├── ri_dvp_processor.cpp    # 主处理器实现
    └── main.cpp                # 示例程序
```

## 引用

如果您使用本代码，请引用：

```bibtex
@article{ri_dvp,
  title={RI-DVP: Range Image-based Dynamic Vehicle/Pedestrian Removal for Static Map Reconstruction},
  author={...},
  journal={...},
  year={2024}
}
```
