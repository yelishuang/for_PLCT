# ROS视觉SLAM算法技术调研与openEuler移植实践

## 目录

- [引言](#引言)
- [主流开源V-SLAM框架分析](#主流开源v-slam框架分析)
  - [ORB-SLAM3：特征点法](#orb-slam3特征点法)
  - [VINS-Mono：资源受限平台的轻量化算法](#vins-mono资源受限平台的轻量化算法)
  - [RTAB-Map：长期大规模建图的算法](#rtab-map长期大规模建图的算法)
  - [DSO：高精度稀疏直接法里程计](#dso高精度稀疏直接法里程计)
- [V-SLAM前沿技术发展趋势](#v-slam前沿技术发展趋势)
- [ORB-SLAM 2移植可行性评估](#orb-slam-2--orb-slam2-lite-移植至-openeuler-2403-可行性评估报告)
- [Pangolin依赖编译实践](#社区实测pangolin-09-在-openeuler-2403-ltsx86-64--aarch64--risc-v64源码构建可行)
- [总结与展望](#总结与展望)

## 引言

随着机器人技术和自动驾驶的快速发展，视觉SLAM（Simultaneous Localization and Mapping）作为核心感知技术愈发重要。本报告旨在全面调研当前主流的开源视觉SLAM算法框架，分析其技术特点和应用场景，并重点探讨将轻量化的ORB-SLAM 2/ORB-SLAM2-Lite移植到openEuler操作系统的可行性方案。

通过深入分析各算法的架构设计、性能表现和依赖关系，我们期望为开源机器人社区提供一份实用的技术参考，推动SLAM技术在国产操作系统上的应用落地。

## 主流开源V-SLAM框架分析

当前视觉SLAM领域呈现百花齐放的态势，不同算法在精度、效率、鲁棒性等方面各有千秋。本节将深入剖析四个代表性的开源框架，从算法原理到工程实现全方位解读其技术特色。

### ORB-SLAM3：特征点法

ORB-SLAM3是第一个能够同时支持视觉、视觉-惯性以及多地图SLAM的实时系统，支持单目、双目和RGB-D相机，并兼容针孔和鱼眼相机模型。作为ORB-SLAM系列的最新版本。

在算法架构上，ORB-SLAM3采用了三线程并行处理模式。跟踪线程负责实时处理每一帧图像，通过ORB特征提取和匹配估计相机位姿；局部建图线程管理局部地图，包括关键帧和地图点的创建、剔除和优化；闭环检测线程则通过DBoW2词袋模型检测回环，并执行全局优化。这种并行架构充分利用多核处理器的计算能力。

ORB-SLAM3的核心创新体现在以下几个方面：

**Atlas多地图系统**。该系统维护一个由多个子地图组成的地图集，包括一个活跃地图和多个非活跃地图。当跟踪失败时，系统会自动创建新的子地图，而不是简单地重新初始化。这种设计使得系统能够在长时间运行中保持稳定，即使在特征稀疏或光照剧烈变化的环境中也能持续工作。

**紧耦合的视觉-惯性融合**。ORB-SLAM3实现了基于最大后验（MAP）估计的紧耦合视觉-惯性SLAM系统，即使在IMU初始化阶段也完全依赖MAP估计。系统采用IMU预积分技术，在关键帧之间高效地融合IMU测量数据。这种紧耦合方案相比松耦合方案具有更高的精度和鲁棒性。在[EuRoC数据集](https://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets)上，ORB-SLAM3的双目-惯性配置达到了3.6厘米的平均精度，在TUM-VI数据集的快速手持运动场景中精度达到9毫米。

**改进的初始化策略**。大幅缩短了系统启动时间，对于纯视觉模式，系统并行计算单应矩阵和基础矩阵，根据模型得分自动选择最优模型。对于视觉-惯性模式，新的快速IMU初始化方法能够在2-3秒内完成，相比之前的方法快了一个数量级。

**增强的回环检测和地图融合**。提高了系统的长期运行能力，改进的DBoW3词袋模型具有更高的召回率，能够更可靠地检测回环。当检测到与非活跃地图的关联时，系统会执行地图融合，将多个子地图合并成一个全局一致的地图。

在特征提取方面，ORB-SLAM3延续了ORB特征的使用。ORB特征结合了FAST角点检测和旋转不变的BRIEF描述子，具有计算效率高、旋转不变性好的特点。为了实现特征点的均匀分布，系统采用四叉树结构对图像进行自适应分割，确保每个区域都有足够的特征点。

> **相关资源**：
> - [ORB-SLAM3 GitHub仓库](https://github.com/UZ-SLAMLab/ORB_SLAM3)
> - [ORB-SLAM3论文](https://arxiv.org/abs/2007.11898)

### VINS-Mono：资源受限平台的轻量化算法

VINS-Mono是香港科技大学开发的单目视觉-惯性里程计系统，专为无人机等资源受限平台设计。该系统采用基于滑动窗口的优化框架，在保证精度的同时大幅降低了计算复杂度。

系统的核心是紧耦合的非线性优化框架。通过在滑动窗口内联合优化视觉重投影误差和IMU预积分误差，VINS-Mono能够准确估计相机位姿、速度、IMU偏置等状态量。滑动窗口的大小通常设置为10-20个关键帧，这在计算效率和估计精度之间取得了良好平衡。

VINS-Mono的鲁棒初始化机制是其一大特色。系统首先通过纯视觉方法建立初始地图，然后逐步引入IMU信息进行视觉-惯性对齐。这种渐进式初始化策略提高了系统在各种运动条件下的成功率。此外，系统还包含在线外参标定功能，能够自动估计相机与IMU之间的相对位姿。

在实际应用中，VINS-Mono展现出优异的性能：在EuRoC数据集上，系统能够以20-30Hz的频率稳定运行，定位精度与ORB-SLAM3相当。特别是在快速运动和剧烈旋转的场景中，IMU的引入显著提升了系统的稳定性。但是在低纹理环境中，由于缺乏足够的视觉特征，系统性能会明显下降。

> **相关资源**：
> - [VINS-Mono GitHub仓库](https://github.com/HKUST-Aerial-Robotics/VINS-Mono)
> - [VINS-Fusion（多传感器扩展版）](https://github.com/HKUST-Aerial-Robotics/VINS-Fusion)

### RTAB-Map：长期大规模建图的算法

RTAB-Map（Real-Time Appearance-Based Mapping）是专为长期、大规模在线操作设计的图优化SLAM框架。其核心在于独特的内存管理机制，通过将地图数据动态划分为工作内存（WM）、短期内存（STM）和长期内存（LTM），实现了在大规模环境下的实时处理能力。

系统的内存管理策略基于贝叶斯滤波框架。当新的位置被访问时，系统会根据其与当前位置的相关性决定保留在工作内存还是转移到长期内存。这种机制确保工作内存始终保持较小规模，从而保证实时性能。当重新访问之前的区域时，相关数据会从长期内存调回工作内存参与优化。

RTAB-Map的另一个优势是其灵活的传感器支持。系统可以同时处理RGB-D、双目、2D/3D激光雷达等多种传感器数据。前端可以使用各种视觉里程计算法，如ORB-SLAM2、VINS-Mono等，后端则通过[g2o](https://github.com/RainerKuemmerle/g2o)或[GTSAM](https://github.com/borglab/gtsam)进行图优化。这种模块化设计使得RTAB-Map能够适应各种硬件配置和应用场景。

基于外观的回环检测是RTAB-Map的核心技术。系统使用改进的词袋模型构建视觉词典，通过计算图像间的相似度检测回环。相比传统方法，RTAB-Map的回环检测具有更高的召回率和更低的误检率。检测到回环后，系统会触发图优化过程，消除累积误差。

> **相关资源**：
> - [RTAB-Map官网](http://introlab.github.io/rtabmap/)
> - [RTAB-Map GitHub仓库](https://github.com/introlab/rtabmap)

### DSO：高精度稀疏直接法里程计

从特征点法到直接法的演进，代表了视觉SLAM技术路线的重要分支。DSO作为直接法的杰出代表，为我们展示了另一种技术思路的可能性。

DSO (Direct Sparse Odometry) 同样出自慕尼黑工业大学，是继LSD-SLAM之后，直接法视觉里程计的又一力作。DSO巧妙地结合了直接法和稀疏法的优点，在保持直接法高精度的同时，通过稀疏化处理大幅提升了计算效率，实现了在CPU上也能高效运行的视觉里程计系统。

DSO的算法精髓在于其对模型参数的联合优化。它在一个滑动窗口内，同时优化所有关键帧的相机位姿、相机内参（仿射亮度校正模型）以及选定活动点的逆深度。这种"全光度模型"（Complete Photometric Model）的联合优化是其达到高精度的关键。

DSO的核心创新点包括：

**稀疏但有效的点选择策略**：DSO仅选择图像中梯度变化最明显的像素点进行跟踪和优化。这种稀疏化的处理方式显著降低了后端的计算负担，使得实时精细优化成为可能。

**严谨的联合优化框架**：DSO在一个滑动窗口内对所有变量（包括相机位姿、内参和点的逆深度）进行联合的非线性优化。通过最小化光度误差，系统能够获得高度精确的运动估计，其精度在许多数据集上甚至超越了基于特征点法的系统。

**无需求助于特征提取**：作为纯粹的直接法，DSO完全摒弃了特征提取和匹配的步骤，使其对运动模糊不那么敏感，并且能够在特征点法难以工作的场景（如弱纹理环境）中稳定运行。

尽管DSO本身是一个纯视觉里程计系统，没有内置闭环检测和全局优化功能，但其前端的高精度和高效率使其成为许多完整SLAM系统（如后面的DV-SLAM）的理想前端模块。DSO的出现，证明直接法在精度上不仅能媲美甚至超越传统方法。

> **相关资源**：
> - [DSO GitHub仓库](https://github.com/JakobEngel/dso)
> - [DSO论文](https://vision.in.tum.de/research/vslam/dso)

## V-SLAM前沿技术发展趋势

经过对经典算法的深入分析，我们将目光投向未来。V-SLAM技术正在向着更智能、更鲁棒的方向快速演进，以下几个前沿方向尤为值得关注。

V-SLAM 技术在神经渲染、语义理解和动态环境处理等方向取得显著突破，推动地图构建从单纯的几何建模向智能化场景感知演进。神经渲染技术通过改变地图表示方式，实现了从高计算成本的神经辐射场（NeRF）到高效 3D 高斯溅射（3DGS）的跨越。3DGS 将场景编码为大量可微分的 3D 高斯椭球集合，通过自适应扩展策略和从粗到细的位姿优化技术，在 GPU 上实现了 8.43 FPS 的实时渲染与建图速度，生成的密集、照片级真实地图为 AR/VR、机器人导航等场景提供了高精度基础。尽管仍需依赖深度学习框架（如 PyTorch）和 GPU 并行计算能力，但其渲染效率较 NeRF 提升了数百倍，且通过动态调整高斯密度和剔除噪声点有效平衡了质量与速度。

语义 SLAM 通过融合几何与语义信息，构建包含物体类别、空间关系的结构化地图。例如，vS-Graphs 框架通过检测墙面、地面等建筑构件，推断房间、走廊等高层次结构元素，将其整合到可优化的 3D 场景图中，显著提升了定位精度（轨迹误差降低 9.58%）和地图可解释性。Quadric SLAM 则采用双二次曲面表示物体，结合 2D 对象检测直接约束 3D 参数，实现了物体级语义地图的紧凑建模。更先进的系统如 DDN-SLAM，通过联合语义编码和深度引导静态掩码，在动态场景中实现了 20-30 Hz 的实时密集重建，同时利用多分辨率哈希编码加速空洞填充和噪声抑制。

动态环境处理方面，3DS-SLAM 通过 3D 对象检测和 HDBSCAN 聚类，识别并过滤动态特征点，在 [TUM RGB-D 数据集](https://vision.in.tum.de/data/datasets/rgbd-dataset)上较 ORB-SLAM2 提升了 98.01% 的轨迹准确性。DDN-SLAM 则通过条件概率场和深度信息分割动态区域，结合稀疏特征点验证和全局光束平差，有效抑制动态干扰并支持多传感器输入。部分系统更进一步构建 4D 时空地图，同时跟踪静态结构与动态物体的运动轨迹，为自动驾驶等复杂场景提供了更完整的时空感知能力。

## 从调研到实践：移植方案选择

通过前面的技术调研，我们对当前主流SLAM算法有了全面认识。然而，在将这些先进算法移植到openEuler系统时，我们发现大部分框架存在依赖复杂、架构庞大等问题。经过仔细权衡，我们选择了ORB-SLAM 2作为首个移植目标，以下是详细的可行性评估。

## ORB‑SLAM 2 / ORB‑SLAM2‑Lite 移植至 openEuler 24.03 可行性评估报告

### 1  选择 ORB‑SLAM 2 或 ORB‑SLAM2‑Lite 的理由

| 评估维度          | ORB‑SLAM 2 / 2‑Lite                              | 其他常见视觉‑SLAM 框架                                         |
| ------------- | ------------------------------------------------ | ------------------------------------------------------ |
| **核心代码体量**    | < 30 k 行；架构清晰、模块独立                               | RTAB‑Map ≈ 300 k 行，依赖繁多；Cartographer、VINS‑Fusion 等体量更大 |
| **外部依赖数量**    | 6 个主要第三方库，全部可源码构建                                | 需要 gRPC、Abseil、ceres‑solver、Qt、SQLite 等额外组件            |
| **硬件指令集耦合**   | SIMD 优化可完全关闭；提供通用 C++ 路径                         | Cartographer、ORB‑SLAM 3 等默认启用 AVX/FMA，关闭后需额外补丁         |
| **ROS 耦合度**   | 主仓库仅生成原生 C++ 静/动态库，可后续再包装 ROS2 节点                | RTAB‑Map/Cartographer 与 ROS2 API 耦合紧密，难以分阶段移植          |
| **社区活跃度**     | 仍有新 PR 与轻量衍生（2‑Lite）；Issue 与 Patch 资源丰富          | VINS‑Fusion 停更；Cartographer 官方于 2023 停止维护              |
| **现有跨架构成功案例** | 已在 openWrt、Yocto、Ubuntu‑RISC‑V 等嵌入式/非 x86 平台编译通过 | RISC‑V 上尚未见到 RTAB‑Map、Cartographer 的完整成功案例             |

> **结论：** ORB‑SLAM 2/2‑Lite 具备代码精简、依赖少、指令集可裁剪的优势，是在 openEuler 24.03（x86‑64, AArch64, RISC‑V64）上实现快速可用的首选视觉‑SLAM 框架。

---

### 2  ORB‑SLAM 2/2‑Lite 在 openEuler 24.03 的依赖分析

| 功能模块          | 建议软件包（dnf 名称）                                | 版本与备注                                   |
| ------------- | -------------------------------------------- | --------------------------------------- |
| **构建工具链**     | `gcc-c++`, `cmake`, `make`                   | gcc ≥ 10，CMake ≥ 3.16，支持 C++17          |
| **线性代数**      | `eigen3-devel`                               | openEuler 官方仓库提供                        |
| **优化框架**      | **g2o**（源码编译或 RPM 打包）                        | `-DG2O_USE_OPENGL=OFF` 以精简依赖            |
| **稀疏向量量化**    | **DBoW2 / DBoW2‑Lite**                       | 只依赖 Eigen / OpenCV                      |
| **视觉库**       | `opencv-devel`（或自编译 opencv + opencv‑contrib） | 若需 SIFT/SURF, 建议启用 contrib 模块           |
| **可视化/GUI**   | **Pangolin**                                 | Headless 场景可 `-DBUILD_PANGOLIN_GUI=OFF` |
| **并行加速 (可选)** | `tbb-devel`                                  | `-DUSE_TBB=ON` 启用多线程                    |

> **RISC‑V 64 编译注意**
>
> ```cmake
> -DORBSLAM_USE_SSE=OFF
> -DORBSLAM_VECTOR_ISA=NONE
> ```
>
> 关闭所有 x86 SIMD 分支，避免运行时非法指令。

---

### 3  移植实施思路

#### 3.1  阶段划分

1. **依赖组件本地编译 / 打包**
   ‑ 统一使用 `cmake .. -DCMAKE_INSTALL_PREFIX=/usr`，确保 OBS 可解析库路径。
   ‑ 对 g2o、DBoW2、Pangolin 生成独立 `.spec`，交由 openEuler OBS 构建。
2. **核心库编译**
   ‑ 在纯命令行环境中先关闭 GUI 与 SIMD，保证最小可运行集通过。
   ‑ 启动官方数据集（[TUM RGB‑D](https://vision.in.tum.de/data/datasets/rgbd-dataset)、[EuRoC](https://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets)）离线验证轨迹误差 ≤ 1 × 10<sup>‑2</sup> m。
3. **RPM/HarmonyOS‑Pkg 打包**
   ‑ 使用 `cpack -G RPM` 生成二进制与开发包。
   ‑ 通过 `dnf repoquery ORB-SLAM2-libs` 验证依赖链完整性。
4. **ROS2 Wrapper（可选）**
   ‑ 利用 `rosidl_generate_interfaces` 暴露 `PoseStamped`、`PointCloud2` 等 topic。
   ‑ 在 openEuler ROS SIG 的 `openEuler_ROS_24.03` 项目中开新 sub‑project。
5. **性能优化与架构扩展**
   ‑ 针对 AArch64 开启 NEON，针对 RISC‑V 开启 V-extension(若硬件支持)。
   ‑ 逐步合并 ORB‑SLAM 3 的多传感器 (IMU/Depth) 分支，实现全栈融合。

#### 3.2  参考命令（本地快速验证）

```bash
# 1. 安装基准依赖
sudo dnf install -y gcc-c++ cmake make opencv-devel eigen3-devel tbb-devel git

# 2. 获取源码
mkdir -p ~/src && cd ~/src
for repo in ORB_SLAM2-Lite g2o DBoW2 Pangolin; do
  git clone https://github.com/$( [ "$repo" = "g2o" ] && echo "RainerKuemmerle" || echo "rmsalinas" )/$repo.git
done

# 3. 编译 g2o（示例）
cd g2o && mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release -DG2O_USE_OPENGL=OFF
make -j$(nproc) && sudo make install

# 4. 编译 ORB‑SLAM2‑Lite
cd ~/src/ORB_SLAM2-Lite && mkdir build && cd build
cmake .. \
  -DCMAKE_BUILD_TYPE=Release \
  -DUSE_PANGOLIN=ON \
  -DORBSLAM_USE_SSE=OFF \
  -DUSE_TBB=ON
make -j$(nproc)

# 5. 运行示例
./Examples/Monocular/mono_tum Vocabulary/ORBvoc.txt Examples/Monocular/TUM1.yaml path/to/associate.txt
```

> **相关资源**：
> - [ORB-SLAM2 GitHub仓库](https://github.com/raulmur/ORB_SLAM2)
> - [ORB-SLAM2-Lite GitHub仓库](https://github.com/rmsalinas/ORB_SLAM2-Lite)
> - [g2o GitHub仓库](https://github.com/RainerKuemmerle/g2o)
> - [DBoW2 GitHub仓库](https://github.com/dorian3d/DBoW2)

---

## 关键依赖验证：Pangolin编译实践

作为ORB-SLAM 2的关键可视化依赖，Pangolin的成功编译是整个移植工作的重要前提。以下是社区成员在openEuler多架构平台上的实测经验。

### 社区实测：Pangolin 0.9 在 openEuler 24.03 LTS（x86-64 / aarch64 / risc-v64）源码构建可行

虽然 Pangolin **尚无官方 RPM**，但开源机器人 SIG 成员已在最小服务器版 openEuler 24.03 LTS 上完成多次手工编译（用作 ORB-SLAM 2、3 的可视化后端）。下面汇总一条 **可复现流程**，所需命令均在 root（或 sudo）终端下执行。

---

## 1 准备：一次性安装系统包

```bash
sudo dnf install -y                     \
  git gcc-c++ cmake make ninja-build    \
  eigen3-devel                          \
  libX11-devel libXi-devel libXrandr-devel libXext-devel \
  mesa-libGL-devel mesa-libEGL-devel    \
  glew-devel freeglut-devel             \
  libjpeg-turbo-devel libpng-devel      \
  ffmpeg-devel                          \
  pkgconfig
# 无桌面/纯虚拟机可省 freeglut-devel，并在 CMake 里禁用 GUI
```

> **说明**
>
> * Pangolin 的 CMake 脚本会自动探测并启用可用后端；缺失的依赖只会导致相应功能被编译为 OFF ，不会中断构建。
> * `mesa-libGL-devel` / `mesa-libEGL-devel` 为 openEuler 等 Red Hat 系列对 OpenGL 的通用包名。

---

## 2 克隆源码（含子模块）

```bash
mkdir -p ~/src && cd ~/src
git clone --recursive https://github.com/stevenlovegrove/Pangolin.git
cd Pangolin
```

---

## 3 配置与编译

### 3.1 通用带 GUI 场景

```bash
cmake -B build -G Ninja \
      -DCMAKE_BUILD_TYPE=Release \
      -DPANGOLIN_BUILD_PYTHON=OFF \
      -DPANGOLIN_INSTALL_CMAKE_CONFIG=ON
cmake --build build     # Ninja 将自动并行
sudo cmake --install build
```

> **最快路径**：Pangolin 官方推荐用 Ninja 直接加速构建；其 `install_prerequisites.sh` 也能自动为 `dnf` 生成依赖清单（支持 openEuler，同 Fedora 系） ([Pangolin GitHub](https://github.com/stevenlovegrove/Pangolin))

### 3.2 无图形 / 纯服务器 / CI 环境

```bash
cmake -B build -G Ninja \
      -DCMAKE_BUILD_TYPE=Release \
      -DBUILD_PANGOLIN_GUI=OFF \
      -DBUILD_PANGOLIN_GL=OFF \
      -DBUILD_PANGOLIN_FFMPEG=OFF
cmake --build build
sudo cmake --install build
```

---

## 4 运行自检

```bash
cd ~/src/Pangolin/build/examples
./SimpleDisplay  # 有显卡时应弹出空窗口
```

无 GUI 构建时可改跑 `examples/SharedMemory/hello` 之类纯 TTY 输出的 demo。

---



> **参与贡献**：
> - [openEuler开源机器人SIG](https://gitee.com/openeuler/community/tree/master/sig/sig-ROS)
> - [openEuler OBS构建服务](https://build.openeuler.openatom.cn/)
> - 技术交流：ros@openeuler.org
