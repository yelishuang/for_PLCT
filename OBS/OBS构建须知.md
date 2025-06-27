# OBS/OSC ROS包打包与测试完整手册

## 目录
1. [概述](#概述)
2. [环境准备](#环境准备)
3. [OSC打包操作详解](#osc打包操作详解)
4. [虚拟环境测试流程](#虚拟环境测试流程)
5. [常见场景与解决方案](#常见场景与解决方案)
6. [最佳实践](#最佳实践)

---

## 概述

### 什么是OBS？
OBS (Open Build Service) 是一个自动化、透明化的软件包构建和分发系统。它支持跨平台构建，可以为不同的Linux发行版创建软件包。

### 什么是OSC？
OSC (openSUSE Commander) 是OBS的命令行客户端工具，用于管理OBS项目和软件包。

### 工作流程概览
```
源代码 → OBS项目 → 自动构建 → RPM包 → 测试验证
```

---

## 环境准备

### 1. OBS服务器信息
- **服务器地址**: https://build.tarsier-infra.isrc.ac.cn
- **目标发行版**: openEuler 24.03 (riscv64)
- **ROS版本**: ROS 2 Humble

### 2. 开发环境配置

#### 2.1 安装OSC客户端

**openEuler/RHEL系列**:
```bash
sudo yum install -y osc
# 或者
sudo dnf install -y osc
```

**Ubuntu/Debian系列**:
```bash
# 方法1：使用pipx（推荐）
sudo apt install pipx
pipx install osc

# 方法2：使用pip
pip install --user osc
```

**SUSE系列**:
```bash
sudo zypper install osc
```

#### 2.2 配置OSC

首次使用时配置认证信息：
```bash
# 创建配置文件目录
mkdir -p ~/.config/osc

# 首次连接会自动创建配置
osc -A https://build.tarsier-infra.isrc.ac.cn ls
```

配置文件位置：`~/.config/osc/oscrc` 或 `~/.oscrc`

手动配置示例：
```ini
[general]
apiurl = https://build.tarsier-infra.isrc.ac.cn
# 其他全局设置

[https://build.tarsier-infra.isrc.ac.cn]
user = your_username
pass = your_password
# 可选：使用密钥认证
# keyring = 1
```

---

## OSC打包操作详解

### 1. 项目管理

#### 1.1 创建项目

**Web界面创建**（推荐初学者）:
1. 登录OBS Web界面
2. 点击 "Create Project"
3. 项目名称格式：`home:<用户名>:<项目名>`
   - 示例：`home:zhangsan:ros-humble-test`

**命令行创建**:
```bash
# 创建项目元数据文件
cat > project.xml << EOF
<project name="home:zhangsan:ros-humble-test">
  <title>ROS Humble Test Packages</title>
  <description>Personal project for ROS Humble package testing</description>
  <person userid="zhangsan" role="maintainer"/>
  <repository name="openEuler_ROS_24.03">
    <path project="openEuler:24.03" repository="standard"/>
    <arch>riscv64</arch>
    <arch>x86_64</arch>
  </repository>
</project>
EOF

# 创建项目
osc meta prj home:zhangsan:ros-humble-test -F project.xml
```

#### 1.2 检出项目
```bash
# 检出整个项目
osc checkout home:zhangsan:ros-humble-test
cd home:zhangsan:ros-humble-test

# 或使用短命令
osc co home:zhangsan:ros-humble-test
```

### 2. 软件包创建流程

#### 2.1 标准流程（新建软件包）

```bash
# 1. 创建软件包
osc mkpac ros-humble-package-name
cd ros-humble-package-name

# 2. 准备源代码
# 方式A：从GitHub下载
wget https://github.com/ros2/package/archive/refs/tags/1.0.0.tar.gz \
     -O ros-humble-package-name_1.0.0.orig.tar.gz

# 方式B：从本地打包
tar czf ros-humble-package-name_1.0.0.orig.tar.gz \
    --exclude=.git \
    --transform 's,^,package-name-1.0.0/,' \
    /path/to/source/*

# 3. 创建spec文件
vim ros-humble-package-name.spec
```

#### 2.2 Spec文件模板

ROS包的Spec文件模板根据包类型分为两种：

- **CMake包模板**: 适用于C++包和使用ament_cmake构建的包
  - 参见: [ros-cmake-spec-template](ros-cmake-spec-template)
  
- **Python包模板**: 适用于纯Python包和使用ament_python构建的包
  - 参见: [ros-python-spec-template](ros-python-spec-template)

选择合适的模板，将其中的占位符替换为实际值：
- `PACKAGE_NAME`: 包名（不含ros-humble前缀）
- `X.Y.Z`: 版本号
- `PACKAGE_DESCRIPTION`: 简短描述
- `DETAILED_DESCRIPTION`: 详细描述
- `UPSTREAM_PACKAGE_NAME`: 上游源码包名

**如何选择模板**：
- 查看源码中的`package.xml`，找到`<build_type>`标签
- `ament_cmake` → 使用CMake模板
- `ament_python` → 使用Python模板
- 如果包含`CMakeLists.txt` → 通常使用CMake模板
- 如果只有`setup.py` → 使用Python模板

#### 2.3 添加和提交文件

```bash
# 添加所有新文件
osc add *
# 或指定文件
osc add ros-humble-package-name.spec
osc add *.tar.gz

# 查看状态
osc status

# 提交到OBS
osc commit -m "Initial import of ros-humble-package-name 1.0.0"
# 或使用短命令
osc ci -m "Add package ros-humble-package-name"
```

### 3. 构建管理

#### 3.1 查看构建状态

```bash
# 查看所有仓库构建状态
osc results

# 查看特定仓库和架构
osc results -r openEuler_ROS_24.03 -a riscv64

# 实时监控构建日志
osc buildlog openEuler_ROS_24.03 riscv64 -l
```

#### 3.2 本地构建测试

```bash
# 在提交前进行本地构建测试
osc build openEuler_ROS_24.03 riscv64 ros-humble-package-name.spec

# 使用自定义构建根目录
osc build --root=/var/tmp/build-root openEuler_ROS_24.03 riscv64
```

#### 3.3 下载构建产物

```bash
# 下载所有二进制包
osc getbinaries home:zhangsan:ros-humble-test \
    ros-humble-package-name \
    openEuler_ROS_24.03 \
    riscv64

# 排除调试包
osc getbinaries home:zhangsan:ros-humble-test \
    ros-humble-package-name \
    openEuler_ROS_24.03 \
    riscv64 --debuginfo=0

# 下载到指定目录
osc getbinaries ... -d /path/to/download
```

### 4. 版本更新流程

#### 4.1 更新现有软件包

```bash
# 1. 更新工作副本
osc up

# 2. 下载新版本源码
wget https://github.com/ros2/package/archive/refs/tags/1.1.0.tar.gz \
     -O ros-humble-package-name_1.1.0.orig.tar.gz

# 3. 删除旧版本
osc rm ros-humble-package-name_1.0.0.orig.tar.gz

# 4. 添加新版本
osc add ros-humble-package-name_1.1.0.orig.tar.gz

# 5. 更新spec文件
vim ros-humble-package-name.spec
# 修改Version: 1.1.0
# 更新changelog

# 6. 提交更改
osc ci -m "Update to version 1.1.0"
```

#### 4.2 分支和提交请求

```bash
# 从其他项目分支
osc branch openEuler:Factory ros-humble-package-name

# 创建提交请求
osc sr home:zhangsan:branches:openEuler:Factory \
    ros-humble-package-name \
    openEuler:Factory \
    -m "Update ros-humble-package-name to 1.1.0"
```

---

## 虚拟环境测试流程

### 1. 虚拟机环境准备

#### 1.1 创建openEuler虚拟机

请参考以下资源创建虚拟机环境：
- **openEuler RISC-V虚拟机配置**: https://docs.openeuler.org/zh/docs/25.03/server/installation_upgrade/installation/risc-v-qemu.html
- **下载openEuler镜像**: https://repo.openeuler.org/openEuler-25.03/virtual_machine_img/riscv64/

推荐配置：
- 内存: 至少4GB
- 存储: 至少20GB
- 架构: riscv64或x86_64

#### 1.2 配置ROS环境

```bash
# 1. 更新系统
sudo dnf update -y

# 2. 安装ROS基础包
sudo dnf install -y ros-humble-ros-base

# 3. 配置环境变量
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc

# 4. 验证安装
ros2 --version
```

### 2. 安装测试包

#### 2.1 传输RPM包到虚拟机

```bash
# 从主机传输
scp -P 2222 *.rpm user@localhost:~/

# 或使用共享文件夹
# VirtualBox: 设置共享文件夹
# QEMU: 使用9p文件系统
```

#### 2.2 安装RPM包

```bash
# 安装单个包
sudo dnf install ./ros-humble-package-name-1.0.0-1.oe2403.riscv64.rpm

# 安装多个包（含依赖）
sudo dnf install ./*.rpm

# 强制重新安装
sudo dnf reinstall ./ros-humble-package-name-*.rpm

# 查看安装文件
rpm -ql ros-humble-package-name
```

### 3. 功能测试

#### 3.1 基础功能测试

```bash
# 1. 检查库文件
ldd /opt/ros/humble/lib/libpackage_name.so

# 2. 运行示例程序（如果有）
ros2 run package_name example_node

# 3. 检查ROS接口
ros2 interface list | grep package_name
```

#### 3.2 创建测试工作空间

```bash
# 1. 创建工作空间
mkdir -p ~/test_ws/src
cd ~/test_ws/src

# 2. 创建测试包
ros2 pkg create --build-type ament_cmake test_package_name \
    --dependencies ros-humble-package-name

# 3. 编写测试代码
cd test_package_name
mkdir test
```

#### 3.3 单元测试示例

**CMakeLists.txt**:
```cmake
cmake_minimum_required(VERSION 3.8)
project(test_package_name)

find_package(ament_cmake REQUIRED)
find_package(ros-humble-package-name REQUIRED)
find_package(ament_cmake_gtest REQUIRED)

if(BUILD_TESTING)
  ament_add_gtest(test_basic
    test/test_basic.cpp
  )
  target_link_libraries(test_basic
    ${ros-humble-package-name_LIBRARIES}
  )
endif()

ament_package()
```

**test/test_basic.cpp**:
```cpp
#include <gtest/gtest.h>
#include "package_name/some_header.hpp"

TEST(PackageNameTest, BasicFunctionality) {
    // 测试代码
    EXPECT_TRUE(true);
}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
```

#### 3.4 运行测试

```bash
# 编译测试包
cd ~/test_ws
colcon build --symlink-install

# 运行所有测试
colcon test

# 查看测试结果
colcon test-result --verbose

# 运行特定测试
./build/test_package_name/test_basic
```

### 4. 集成测试

#### 4.1 多节点测试

```bash
# 终端1：启动第一个节点
ros2 run package_name node1

# 终端2：启动第二个节点
ros2 run package_name node2

# 终端3：监控话题
ros2 topic list
ros2 topic echo /topic_name
```

#### 4.2 性能测试

```bash
# 使用ros2_tracing进行性能分析
ros2 run tracetools trace \
    --path ~/traces \
    -- ros2 run package_name performance_test

# 分析结果
babeltrace ~/traces | grep package_name
```

---

## 常见场景与解决方案

### 1. 依赖管理

#### 1.1 处理复杂依赖

在spec文件中正确声明依赖关系是确保包能够正常构建和运行的关键。依赖分为：

- **BuildRequires**: 构建时需要的包
- **Requires**: 运行时需要的包

具体示例请参考spec模板中的依赖声明部分：
- [CMake包依赖配置](ros-cmake-spec-template)
- [Python包依赖配置](ros-python-spec-template)

常见依赖版本控制：
```spec
# 指定最低版本
Requires: ros-humble-rclcpp >= 16.0.0

# 指定版本范围
Requires: ros-humble-geometry-msgs >= 4.2.0, ros-humble-geometry-msgs < 5.0.0

# 架构相关依赖
Requires: %{name}%{?_isa} = %{version}-%{release}
```

#### 1.2 循环依赖解决

```bash
# 使用bootstrap标志
%bcond_with bootstrap

%if %{with bootstrap}
# 最小化构建
%else
# 完整构建
%endif
```

### 2. 构建问题排查

#### 2.1 查看详细构建日志

```bash
# 实时查看
osc bl openEuler_ROS_24.03 riscv64 -l

# 下载完整日志
osc bl openEuler_ROS_24.03 riscv64 > build.log
```

#### 2.2 常见构建错误

**CMake找不到包**:
```spec
# 添加CMAKE_PREFIX_PATH
%build
export CMAKE_PREFIX_PATH=/opt/ros/humble
%cmake ...
```

**Python模块路径问题**:
```spec
# 设置PYTHONPATH
export PYTHONPATH=/opt/ros/humble/lib/python3.10/site-packages:$PYTHONPATH
```

### 3. 打包特殊情况

#### 3.1 Python包打包

Python包需要使用专门的spec模板，主要区别：
- 设置 `BuildArch: noarch`（纯Python包）
- 使用 `python3 setup.py` 而非 CMake
- 正确设置Python模块路径

详见: [ros-python-spec-template](ros-python-spec-template)

#### 3.2 包含启动文件

对于包含launch文件、配置文件的包，需要在spec文件中添加：

```spec
%files
/opt/ros/humble/share/%{name}/launch/
/opt/ros/humble/share/%{name}/config/
/opt/ros/humble/share/%{name}/params/
```

#### 3.3 多架构支持

如果包包含架构相关的二进制文件：

```spec
# 在spec头部
ExclusiveArch: x86_64 aarch64 riscv64

# 使用架构条件
%ifarch x86_64
# x86_64特定配置
%endif

%ifarch riscv64
# riscv64特定配置
%endif
```

### 4. 测试环境问题

#### 4.1 显示相关测试

```bash
# 使用虚拟显示
sudo dnf install xvfb
xvfb-run -a colcon test
```

#### 4.2 网络隔离环境

```bash
# 配置ROS使用本地主机
export ROS_LOCALHOST_ONLY=1
```

---

## 最佳实践

### 1. 项目组织

```
home:username:ros-humble/
├── ros-humble-core-packages/     # 核心包
├── ros-humble-drivers/           # 驱动包
├── ros-humble-tools/             # 工具包
└── ros-humble-experimental/      # 实验性包
```

### 2. 版本管理策略

- **主版本号**: 跟随上游ROS包版本
- **次版本号**: 打包修订版本
- **发布号**: 包含发行版标识 (如 `.oe2403`)

### 3. 自动化脚本

**批量更新脚本**:
```bash
#!/bin/bash
# update_packages.sh

PACKAGES="package1 package2 package3"
VERSION="1.2.0"

for pkg in $PACKAGES; do
    cd $pkg
    osc up
    # 下载新版本
    wget ... -O ${pkg}_${VERSION}.orig.tar.gz
    # 更新spec
    sed -i "s/Version:.*/Version: $VERSION/" *.spec
    osc ci -m "Update $pkg to $VERSION"
    cd ..
done
```

### 4. 持续集成

```yaml
# .obs/workflows/ci.yml
workflow:
  steps:
    - branch_package:
        source_project: home:user:ros-humble
        source_package: ros-humble-package
    - configure_repositories:
        repositories:
          - name: openEuler_ROS_24.03
            architectures:
              - riscv64
              - x86_64
    - build:
        timeout: 3600
    - tests:
        - unit_tests
        - integration_tests
```

### 5. 文档规范

每个包应包含：
- `README.md`: 包描述和使用说明
- `CHANGELOG.md`: 版本变更记录
- `LICENSE`: 许可证文件
- `.spec`: 打包规范文件（参考[CMake模板](ros-cmake-spec-template)或[Python模板](ros-python-spec-template)）


---

## 附录

### A. 常用OSC命令速查

```bash
# 项目操作
osc ls                          # 列出项目/包
osc co PROJECT                  # 检出项目
osc up                          # 更新工作副本
osc mkpac PACKAGE              # 创建新包

# 文件操作  
osc add FILE                    # 添加文件
osc rm FILE                     # 删除文件
osc mv OLD NEW                  # 重命名文件
osc status                      # 查看状态

# 构建操作
osc build REPO ARCH             # 本地构建
osc results                     # 查看构建结果
osc buildlog REPO ARCH          # 查看构建日志
osc rebuil                      # 触发重新构建

# 提交操作
osc diff                        # 查看差异
osc ci -m "MESSAGE"             # 提交更改
osc submitrequest               # 创建提交请求
```

### B. 故障排除检查清单

- [ ] 检查网络连接和OBS服务器状态
- [ ] 验证认证信息是否正确
- [ ] 确认项目和仓库配置
- [ ] 检查源码包命名和版本号
- [ ] 验证spec文件语法
- [ ] 确认依赖包是否可用
- [ ] 查看构建日志中的错误信息
- [ ] 测试本地构建是否成功
- [ ] 检查目标架构兼容性
- [ ] 验证安装路径和文件权限

### C. 相关资源

- [OBS官方文档](https://openbuildservice.org/help/)
- [openEuler ROS SIG](https://gitee.com/openeuler/community/tree/master/sig/sig-ROS)
- [ROS 2文档](https://docs.ros.org/en/humble/)
- [RPM打包指南](https://rpm-packaging-guide.github.io/)

---

**文档版本**: 1.0.0  
**最后更新**: 2025-06-28  
**维护者**: Yelishuang