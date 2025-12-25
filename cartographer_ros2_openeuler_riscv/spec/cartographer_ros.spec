%bcond_without tests
%bcond_without weak_deps

%global debug_package %{nil}
%global __os_install_post %(echo '%{__os_install_post}' | sed -e 's!/usr/lib[^[:space:]]*/brp-python-bytecompile[[:space:]].*$!!g')
%global __provides_exclude_from ^/opt/ros/%{ros_distro}/.*$
%global __requires_exclude_from ^/opt/ros/%{ros_distro}/.*$

%define RosPkgName      cartographer_ros
%define ros_distro      humble

Name:           ros-%{ros_distro}-%{RosPkgName}
Version:        2.0.9003
Release:        1%{?dist}%{?release_suffix}
Summary:        ROS2 integration for Cartographer SLAM

Url:            https://github.com/ros2/cartographer_ros
License:        Apache-2.0
Source0:        cartographer_ros-%{version}.tar.gz
Patch0:         0002-cartographer_ros-abseil-macros.patch
Patch1:         0004-fix-build-resolve-compilation-issues-in-offline_node.patch
Patch2:         0005-fix-oe-ros2-apply-temporary-workaround-for-missing-t.patch

# System dependencies
BuildRequires:  cmake
BuildRequires:  gcc-c++
BuildRequires:  boost-devel
BuildRequires:  eigen3-devel
BuildRequires:  abseil-cpp-devel
BuildRequires:  cairo-devel
BuildRequires:  ceres-solver-devel
BuildRequires:  gflags-devel
BuildRequires:  glog-devel
BuildRequires:  lua-devel
BuildRequires:  protobuf-devel
BuildRequires:  pcl-devel
BuildRequires:  yaml-cpp-devel
BuildRequires:  gmock-devel
BuildRequires:  gtest-devel
BuildRequires:  python3-sphinx

# ROS base build dependencies
BuildRequires:  ros-%{ros_distro}-ros-workspace
BuildRequires:  ros-%{ros_distro}-ament-cmake
BuildRequires:  ros-%{ros_distro}-rosidl-default-generators

# cartographer_ros_msgs dependencies
BuildRequires:  ros-%{ros_distro}-builtin-interfaces
BuildRequires:  ros-%{ros_distro}-geometry-msgs
BuildRequires:  ros-%{ros_distro}-std-msgs

# cartographer_ros dependencies
BuildRequires:  ros-%{ros_distro}-ros-environment
BuildRequires:  ros-%{ros_distro}-cartographer
BuildRequires:  ros-%{ros_distro}-nav-msgs
BuildRequires:  ros-%{ros_distro}-pcl-conversions
BuildRequires:  ros-%{ros_distro}-rclcpp
BuildRequires:  ros-%{ros_distro}-rosbag2-cpp
BuildRequires:  ros-%{ros_distro}-rosbag2-storage
BuildRequires:  ros-%{ros_distro}-sensor-msgs
BuildRequires:  ros-%{ros_distro}-tf2
BuildRequires:  ros-%{ros_distro}-tf2-eigen
BuildRequires:  ros-%{ros_distro}-tf2-msgs
BuildRequires:  ros-%{ros_distro}-tf2-ros
BuildRequires:  ros-%{ros_distro}-urdf
BuildRequires:  ros-%{ros_distro}-visualization-msgs
BuildRequires:  ros-%{ros_distro}-launch
BuildRequires:  ros-%{ros_distro}-robot-state-publisher

# cartographer_rviz dependencies
BuildRequires:  ros-%{ros_distro}-pluginlib
BuildRequires:  ros-%{ros_distro}-rviz-common
BuildRequires:  ros-%{ros_distro}-rviz-ogre-vendor
BuildRequires:  ros-%{ros_distro}-rviz-rendering

# Runtime dependencies
Requires:       ros-%{ros_distro}-ros-workspace
Requires:       ros-%{ros_distro}-rosidl-default-runtime
Requires:       ros-%{ros_distro}-cartographer
Requires:       ros-%{ros_distro}-builtin-interfaces
Requires:       ros-%{ros_distro}-geometry-msgs
Requires:       ros-%{ros_distro}-std-msgs
Requires:       ros-%{ros_distro}-nav-msgs
Requires:       ros-%{ros_distro}-pcl-conversions
Requires:       ros-%{ros_distro}-rclcpp
Requires:       ros-%{ros_distro}-rosbag2-cpp
Requires:       ros-%{ros_distro}-rosbag2-storage
Requires:       ros-%{ros_distro}-sensor-msgs
Requires:       ros-%{ros_distro}-tf2
Requires:       ros-%{ros_distro}-tf2-eigen
Requires:       ros-%{ros_distro}-tf2-msgs
Requires:       ros-%{ros_distro}-tf2-ros
Requires:       ros-%{ros_distro}-urdf
Requires:       ros-%{ros_distro}-visualization-msgs
Requires:       ros-%{ros_distro}-launch
Requires:       ros-%{ros_distro}-robot-state-publisher
Requires:       ros-%{ros_distro}-pluginlib
Requires:       ros-%{ros_distro}-rviz-common
Requires:       ros-%{ros_distro}-rviz-ogre-vendor
Requires:       ros-%{ros_distro}-rviz-rendering

Provides:       %{name}-devel = %{version}-%{release}
Provides:       %{name}-doc = %{version}-%{release}
Provides:       %{name}-runtime = %{version}-%{release}

%description
Cartographer_ros provides ROS2 integration for Cartographer, a real-time
simultaneous localization and mapping (SLAM) library in 2D and 3D.

%prep
%autosetup -p1 -n cartographer_ros-%{version}

%build
export PYTHONPATH=/opt/ros/%{ros_distro}/lib/python%{python3_version}/site-packages

if [ -f "/opt/ros/%{ros_distro}/setup.sh" ]; then . "/opt/ros/%{ros_distro}/setup.sh"; fi

# Create local install directory for intermediate build artifacts
LOCAL_INSTALL_DIR=%{_builddir}/install
mkdir -p ${LOCAL_INSTALL_DIR}

# Build each subpackage separately
for pkg in cartographer_ros_msgs cartographer_ros cartographer_rviz; do
    if [ -d "$pkg" ] && [ -f "$pkg/CMakeLists.txt" ]; then
        echo "Building $pkg..."
        mkdir -p $pkg/.obj-%{_target_platform}
        cd $pkg/.obj-%{_target_platform}
        %cmake3 \
            -UINCLUDE_INSTALL_DIR \
            -ULIB_INSTALL_DIR \
            -USYSCONF_INSTALL_DIR \
            -USHARE_INSTALL_PREFIX \
            -ULIB_SUFFIX \
            -DCMAKE_INSTALL_PREFIX="/opt/ros/%{ros_distro}" \
            -DAMENT_PREFIX_PATH="/opt/ros/%{ros_distro};${LOCAL_INSTALL_DIR}/opt/ros/%{ros_distro}" \
            -DCMAKE_PREFIX_PATH="/opt/ros/%{ros_distro};${LOCAL_INSTALL_DIR}/opt/ros/%{ros_distro}" \
            -DSETUPTOOLS_DEB_LAYOUT=OFF \
%if !0%{?with_tests}
            -DBUILD_TESTING=OFF \
%endif
            ..
        %make_build
        # Install to local directory so subsequent packages can find dependencies
        make install DESTDIR=${LOCAL_INSTALL_DIR}
        cd ../..
    fi
done

%install
export PYTHONPATH=/opt/ros/%{ros_distro}/lib/python%{python3_version}/site-packages

if [ -f "/opt/ros/%{ros_distro}/setup.sh" ]; then . "/opt/ros/%{ros_distro}/setup.sh"; fi

# Install each subpackage
for pkg in cartographer_ros_msgs cartographer_ros cartographer_rviz; do
    if [ -d "$pkg/.obj-%{_target_platform}" ]; then
        %make_install -C $pkg/.obj-%{_target_platform}
    fi
done

%files
/opt/ros/%{ros_distro}

%changelog
* Wed Dec 24 2025 tyns_tarsier-infra <yelishuang23@mails.usas.ac.cn> - 2.0.9003-1
- Initial package for openEuler ROS2 humble
- Add abseil macros patch
- Add build fix patches
