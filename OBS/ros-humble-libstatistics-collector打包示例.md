## openEuler + OBS：`ros-humble-libstatistics-collector` 打包-构建-验收全流程

> **环境**
>
> * OBS 实例：[https://build.tarsier-infra.isrc.ac.cn](https://build.tarsier-infra.isrc.ac.cn)
> * 发行版：openEuler **24.03**（riscv64）
> * 示例包：`ros-humble-libstatistics-collector` **v1.3.1**
> * 开发机：Ubuntu / 任意 Linux，目标虚拟机：openEuler 24.03

---

### 1 · 注册并创建个人项目

1. 注册并登录 OBS。
2. Web UI → **Create Project**：

   ```
   home:<your_id>:ros-test
   ```

---

### 2 · 安装 OBS 客户端

| 平台               | 指令                                          |
| ---------------- | ------------------------------------------- |
| openEuler / RHEL | `sudo yum install -y obs-api obs-server`    |
| Ubuntu / Debian  | `sudo apt install pipx && pipx install osc` |

---

### 3 · 配置 `osc`

```bash
osc -A https://build.tarsier-infra.isrc.ac.cn ls home:<your_id>:ros-test
# 首次会询问账号密码并写入 ~/.oscrc
```

---

### 4 · 获取项目工作副本

```bash
osc -A https://build.tarsier-infra.isrc.ac.cn checkout home:<your_id>:ros-test
cd home:<your_id>:ros-test
```

---

### 5 · 创建并填充软件包目录

```bash
osc mkpac ros-humble-libstatistics-collector
cd ros-humble-libstatistics-collector

# 下载源码
wget https://github.com/ros-tooling/libstatistics_collector/archive/refs/tags/1.3.1.tar.gz \
     -O ros-humble-libstatistics-collector_1.3.1.orig.tar.gz

# 复制经过验证的 .spec
cp <path>/ros-humble-libstatistics-collector.spec .
```

---

### 6 · 上传并触发构建

```bash
osc addremove
osc ci -m "Initial import of ros-humble-libstatistics-collector 1.3.1"
```

查看状态：

```bash
osc results
osc buildinfo openEuler_ROS_24.03 riscv64
```
![构建成功示例截图](./截图%202025-06-28%2012-55-04.png)

---

### 7 · 下载二进制 RPM

```bash
osc getbinaries \
  home:<your_id>:ros-test \
  ros-humble-libstatistics-collector \
  openEuler_ROS_24.03 \
  riscv64 -a --debuginfo=0
```

---

### 8 · openEuler 虚拟机安装

```bash
sudo dnf reinstall ./ros-humble-libstatistics-collector-1.3.1-1.oe2403.riscv64.rpm
rpm -ql ros-humble-libstatistics-collector | grep '\.so'
```

---

### 9 · 官方单元测试验证

```bash
# 9.1 建工作区
mkdir -p ~/stats_test_ws/src && cd ~/stats_test_ws/src
ros2 pkg create --build-type ament_cmake libstatistics_collector_tests
cd libstatistics_collector_tests

# 9.2 加入测试源码
amkdir test
cp /path/to/test_moving_average_statistics.cpp test/

# 9.3 编辑 CMakeLists.txt
```

```cmake
cmake_minimum_required(VERSION 3.8)
project(libstatistics_collector_tests)

find_package(ament_cmake REQUIRED)
find_package(libstatistics_collector REQUIRED)
find_package(rcpputils REQUIRED)
find_package(ament_cmake_gtest REQUIRED)

ament_add_gtest(test_moving_average_statistics
  ${PROJECT_SOURCE_DIR}/test/test_moving_average_statistics.cpp)

target_include_directories(test_moving_average_statistics PRIVATE
  /opt/ros/humble/include/libstatistics_collector
  /opt/ros/humble/include/rcpputils)

target_link_libraries(test_moving_average_statistics
  libstatistics_collector::libstatistics_collector
  rcpputils::rcpputils)

ament_package()
```

```bash
# 9.4 编译 & 运行
a cd ~/stats_test_ws
source /opt/ros/humble/setup.sh
colcon build --symlink-install --cmake-force-configure
colcon test --packages-select libstatistics_collector_tests
colcon test-result --verbose
```

输出示例：

![测试成功示例截图](./截图%202025-06-27%2016-41-47.png)

---

**至此，打包 → 构建 → 下载 → 安装 → 单元测试 全流程成功闭环。**

> 下游项目集成示例
>
> ```cmake
> find_package(libstatistics_collector REQUIRED)
> target_link_libraries(my_node
>   libstatistics_collector::libstatistics_collector)
> #include "libstatistics_collector/moving_average_statistics/moving_average.hpp"
> ```

