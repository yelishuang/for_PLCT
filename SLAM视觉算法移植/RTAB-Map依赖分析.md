# RTABMAP 依赖分析报告（openEuler 24.03 RISC-V + ROS 2 Humble）

## 一、项目概述

RTABMAP（Real-Time Appearance-Based Mapping）项目由两个主要部分组成：rtabmap核心库和rtabmap_ros包集。核心库是一个独立的C++库，版本为0.22.0，提供了完整的SLAM算法实现，不依赖于任何ROS组件。而rtabmap_ros包集则是核心库在ROS 2 Humble环境下的封装，包含了rtabmap_conversions、rtabmap_msgs、rtabmap_sync、rtabmap_util、rtabmap_odom、rtabmap_slam、rtabmap_viz、rtabmap_rviz_plugins等多个功能包，以及用于演示和示例的配置包。

从提供的CMakeLists文件分析可以看出，rtabmap核心库采用了高度模块化的设计，通过大量的编译选项来控制各种可选功能的启用与禁用。这种设计使得该项目可以根据不同的硬件平台和应用需求进行灵活配置。对于RISC-V架构的移植，这种灵活性尤为重要，因为我们可以轻松地排除那些依赖于特定硬件加速（如CUDA）的功能模块。

## 二、核心库依赖树分析

### 基础依赖层级

rtabmap核心库的依赖关系呈现出清晰的层次结构。在最底层是构建工具和基础库，这些是整个系统运行的基石。CMake作为构建系统要求版本不低于3.14，而编译器则需要支持C++11标准，部分模块在检测到相应依赖时会自动启用C++14或C++17支持。

```
rtabmap核心库 (0.22.0)
├── 构建系统
│   ├── CMake (>= 3.14)
│   └── GCC (>= 4.0, 支持C++11/14/17)
│
├── 必需依赖
│   ├── OpenCV (必需模块)
│   │   ├── core
│   │   ├── calib3d
│   │   ├── imgproc
│   │   ├── highgui
│   │   ├── stitching
│   │   ├── photo
│   │   ├── video
│   │   └── videoio
│   │   └── [可选] xfeatures2d, nonfree (SIFT/SURF特征)
│   │
│   ├── PCL (>= 1.7)
│   │   ├── common
│   │   ├── io
│   │   ├── kdtree
│   │   ├── search
│   │   ├── surface
│   │   ├── filters
│   │   ├── registration
│   │   ├── sample_consensus
│   │   ├── segmentation
│   │   └── [WITH_QT时] visualization
│   │
│   ├── ZLIB (压缩支持)
│   ├── SQLite3 (数据库，可用内置版本)
│   └── 间接依赖
│       ├── Eigen3 (通过PCL)
│       ├── FLANN (通过PCL)
│       └── Boost (条件性需要)
│
└── GUI支持 (WITH_QT=ON时)
    ├── Qt (4/5/6自动检测)
    └── VTK (3D可视化，Qt启用时必需)
```

### SLAM优化器依赖

SLAM系统的核心是图优化，rtabmap支持多种图优化后端。这些优化器之间并非互斥关系，但系统要求至少有一个可用的优化器。在RISC-V架构上，所有这些优化器都可以正常编译和运行，因为它们都是纯CPU实现的算法库。

```
图优化后端 (至少需要一个)
├── g2o (推荐，支持多种优化算法)
│   └── 可选: Vertigo (鲁棒优化，需要g2o)
├── GTSAM (佐治亚理工，推荐)
│   ├── Boost (必需)
│   └── 可选: Vertigo (也可配合GTSAM使用)
├── Ceres Solver (Google，大规模非线性优化)
│   └── 注意: OKVIS或FLOAM启用时会自动引入
├── TORO (Tree-based netwORk Optimizer，基础选项)
└── MRPT (移动机器人编程工具包，额外支持)
```

### 传感器驱动依赖

传感器驱动是SLAM系统获取数据的关键接口。在RISC-V架构下，需要特别注意区分哪些驱动依赖CUDA，哪些可以在纯CPU环境下运行。根据CMakeLists文件的分析，大部分相机驱动都提供了CPU模式的支持。

```
相机与传感器驱动
├── 深度相机驱动 (CPU兼容)
│   ├── OpenNI (Kinect v1, Xtion)
│   ├── OpenNI2 (新版本驱动框架)
│   ├── Freenect (Kinect v1开源驱动)
│   ├── Freenect2 (Kinect v2, 需要OpenCL但不需要CUDA)
│   ├── RealSense (Intel D400系列旧版)
│   ├── RealSense2 (Intel D400/L500系列)
│   ├── K4A (Azure Kinect, CPU模式可用)
│   ├── MyntEye (小觅相机)
│   ├── DepthAI (OAK相机，需要depthai-core >= 2.24)
│   ├── ZEDOC (ZED Open Capture, 无CUDA版本)
│   └── XVisio SDK
│
├── 深度相机驱动 (需要CUDA，RISC-V不支持)
│   └── ZED SDK (Stereolabs，强制依赖CUDA)
│
└── 其他相机驱动
    ├── DC1394 (FireWire/IEEE1394相机)
    └── FlyCapture2 (Point Grey相机，需专有SDK)
```

### 点云处理与地图构建依赖

点云处理和地图构建模块提供了多种数据结构和算法实现。这些模块都是纯算法实现，不依赖特定硬件加速，因此在RISC-V平台上都可以正常使用。需要注意的是，某些计算密集型的算法可能在RISC-V上表现出较低的性能。

```
点云与地图处理
├── 点云IO与处理
│   ├── PDAL (点云数据抽象库)
│   ├── libLAS (LAS/LAZ格式支持)
│   └── libpointmatcher (ICP算法库)
│       ├── libnabo (最近邻搜索)
│       └── yaml-cpp (配置文件)
│
├── 地图表示
│   ├── OctoMap (3D概率占用八叉树)
│   ├── GridMap (2D栅格地图)
│   ├── CPUTSDF (截断符号距离场，CPU版本)
│   └── OpenChisel (大规模TSDF融合)
│
└── 3D处理库
    ├── CCCoreLib (CloudCompare核心库)
    ├── Open3D (需要CMake >= 3.19)
    └── AliceVision (3D重建，编译资源需求高)
        ├── Geogram (几何处理)
        └── assimp (3D模型导入)
```

### 视觉里程计与SLAM算法依赖

视觉里程计模块提供了多种前端跟踪算法的实现。这些算法大多是学术研究成果的开源实现，各有其特点和适用场景。在RISC-V平台上，所有列出的视觉里程计算法都可以编译运行，因为它们都提供了CPU实现版本。

```
视觉里程计实现
├── 稀疏特征方法
│   ├── FOVIS (快速视觉里程计，MIT)
│   ├── libviso2 (MATLAB移植，立体视觉)
│   └── ORB_SLAM2/3 (ORB特征，支持单目/双目/RGB-D)
│       └── 注意: 自带g2o版本，需禁用系统g2o
│
├── 直接方法
│   └── DVO (稠密视觉里程计，直接法)
│
├── 视觉惯性融合
│   ├── OKVIS (关键帧优化)
│   │   ├── brisk (特征检测，版本2)
│   │   ├── opengv (几何视觉)
│   │   └── Ceres (优化器，需1.9.0版本)
│   ├── MSCKF_VIO (多状态约束卡尔曼滤波)
│   ├── VINS-Fusion (香港科技大学)
│   └── OpenVINS (特拉华大学)
│
└── 激光雷达里程计
    ├── LOAM (激光里程计与建图)
    └── FLOAM (快速LOAM)
        └── Ceres (优化器依赖)
```

### 特殊功能与加速依赖

某些功能模块设计时考虑了硬件加速，这些模块在RISC-V平台上需要特别处理。CUDA相关的功能必须完全禁用，而一些针对ARM优化的库在RISC-V上可能无法达到预期性能。

```
特殊功能模块
├── GPU加速 (RISC-V不支持)
│   ├── CudaSift (GPU SIFT实现)
│   ├── Torch/libtorch (深度学习，SuperPoint)
│   └── OpenCV CUDA模块
│       ├── opencv::gpu
│       ├── opencv::cudafeatures2d
│       ├── opencv::cudaoptflow
│       └── opencv::cudaimgproc
│
├── 平台优化库 (RISC-V性能未知)
│   └── FastCV (高通移动平台优化)
│
└── 其他功能
    ├── Python支持 (Python3 + pybind11 + NumPy)
    ├── OpenGV (多视图几何)
    ├── OpenMP (并行计算，RISC-V支持)
    ├── Madgwick (IMU滤波算法)
    └── ORB_OCTREE (ORB特征八叉树索引)
```

## 三、ROS 2封装层依赖分析

rtabmap_ros作为ROS 2的封装层，其依赖关系建立在核心库之上，同时引入了ROS 2生态系统的标准组件。每个子包都有其特定的功能定位和依赖需求，它们之间也存在相互依赖关系。

```
rtabmap_ros包集
├── rtabmap_msgs (消息定义)
│   ├── rosidl_default_generators
│   ├── builtin_interfaces
│   ├── geometry_msgs
│   ├── std_msgs
│   ├── sensor_msgs
│   └── std_srvs
│
├── rtabmap_conversions (数据转换)
│   ├── rtabmap_msgs
│   ├── RTABMap核心库 (>= 0.22.0)
│   ├── cv_bridge
│   ├── image_geometry
│   ├── laser_geometry
│   ├── pcl_conversions
│   ├── tf2_eigen
│   └── tf2_geometry_msgs
│
├── rtabmap_sync (数据同步)
│   ├── rtabmap_conversions
│   ├── rtabmap_msgs
│   ├── image_transport
│   ├── message_filters
│   ├── diagnostic_updater
│   └── 可选: 多相机支持 (RTABMAP_SYNC_MULTI_RGBD)
│
├── rtabmap_util (工具节点)
│   ├── rtabmap_conversions
│   ├── rtabmap_msgs
│   ├── RTABMap核心库
│   ├── pcl_ros
│   ├── laser_geometry
│   └── 可选扩展
│       ├── octomap_msgs (OctoMap集成)
│       └── grid_map_ros (GridMap集成)
│
├── rtabmap_odom (里程计)
│   ├── rtabmap_sync
│   ├── rtabmap_util
│   ├── rtabmap_conversions
│   ├── pluginlib
│   └── rclcpp_components
│
├── rtabmap_slam (SLAM核心)
│   ├── rtabmap_sync
│   ├── rtabmap_util
│   ├── visualization_msgs
│   └── 可选标记检测
│       ├── apriltag_msgs
│       ├── aruco系列消息
│       └── nav2_msgs (导航集成)
│
├── rtabmap_viz (可视化界面)
│   ├── rtabmap_sync
│   └── RTABMap::gui (需要核心库GUI支持)
│
└── rtabmap_rviz_plugins (RViz插件)
    ├── rtabmap_conversions
    ├── rviz_common
    ├── rviz_rendering
    └── rviz_default_plugins
```

各个ROS 2包之间形成了清晰的依赖链条：消息定义包(rtabmap_msgs)位于最底层，不依赖其他rtabmap包；转换包(rtabmap_conversions)依赖消息定义并连接核心库；同步和工具包提供中间层服务；而里程计、SLAM和可视化包则构建在这些基础设施之上。这种分层设计使得系统具有良好的模块性，便于在不同场景下选择性地使用所需功能。