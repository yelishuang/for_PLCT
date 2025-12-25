%bcond_without tests
%bcond_without weak_deps

%global debug_package %{nil}
%global __os_install_post %(echo '%{__os_install_post}' | sed -e 's!/usr/lib[^[:space:]]*/brp-python-bytecompile[[:space:]].*$!!g')
%global __provides_exclude_from ^/opt/ros/%{ros_distro}/.*$
%global __requires_exclude_from ^/opt/ros/%{ros_distro}/.*$

%define RosPkgName      cartographer
%define ros_distro      humble

Name:           ros-%{ros_distro}-%{RosPkgName}
Version:        2.0.9004
Release:        1%{?dist}%{?release_suffix}
Summary:        ROS cartographer package

Url:            https://github.com/ros2/cartographer
License:        Apache 2.0
Source0:        cartographer-%{version}.tar.gz
Patch0:         0001-cartographer-abseil-macros.patch

Requires: boost-devel
Requires: eigen3-devel
Requires: abseil-cpp-devel
Requires: cairo-devel
Requires: ceres-solver-devel
Requires: gflags-devel
Requires: glog-devel
Requires: lua-devel
Requires: protobuf-devel
Requires: ros-%{ros_distro}-ros-workspace

BuildRequires: boost-devel
BuildRequires: eigen3-devel
BuildRequires: abseil-cpp-devel
BuildRequires: cairo-devel
BuildRequires: ceres-solver-devel
BuildRequires: gflags-devel
BuildRequires: glog-devel
BuildRequires: lua-devel
BuildRequires: protobuf-devel
BuildRequires: git
BuildRequires: gmock-devel
BuildRequires: gtest-devel
BuildRequires: python3-sphinx
BuildRequires: cmake
BuildRequires: ros-%{ros_distro}-ros-workspace

%if 0%{?with_tests}
%endif

Provides:       %{name}-devel = %{version}-%{release}
Provides:       %{name}-doc = %{version}-%{release}
Provides:       %{name}-runtime = %{version}-%{release}

%description
Cartographer is a system that provides real-time simultaneous localization
and mapping (SLAM) in 2D and 3D across multiple platforms and sensor
configurations.

%prep
%autosetup -p1 -n cartographer-%{version}

%build
export PYTHONPATH=/opt/ros/%{ros_distro}/lib/python%{python3_version}/site-packages

if [ -f "/opt/ros/%{ros_distro}/setup.sh" ]; then . "/opt/ros/%{ros_distro}/setup.sh"; fi
mkdir -p .obj-%{_target_platform} && cd .obj-%{_target_platform}
%cmake3 \
    -UINCLUDE_INSTALL_DIR \
    -ULIB_INSTALL_DIR \
    -USYSCONF_INSTALL_DIR \
    -USHARE_INSTALL_PREFIX \
    -ULIB_SUFFIX \
    -DCMAKE_INSTALL_PREFIX="/opt/ros/%{ros_distro}" \
    -DAMENT_PREFIX_PATH="/opt/ros/%{ros_distro}" \
    -DCMAKE_PREFIX_PATH="/opt/ros/%{ros_distro}" \
    -DSETUPTOOLS_DEB_LAYOUT=OFF \
%if !0%{?with_tests}
    -DBUILD_TESTING=OFF \
%endif
    ..

%make_build

%install
export PYTHONPATH=/opt/ros/%{ros_distro}/lib/python%{python3_version}/site-packages

if [ -f "/opt/ros/%{ros_distro}/setup.sh" ]; then . "/opt/ros/%{ros_distro}/setup.sh"; fi
%make_install -C .obj-%{_target_platform}

%if 0%{?with_tests}
%check
export PYTHONPATH=/opt/ros/%{ros_distro}/lib/python%{python3_version}/site-packages

TEST_TARGET=$(%__make -qp -C .obj-%{_target_platform} | sed "s/^\(test\|check\):.*/\\1/;t f;d;:f;q0")
if [ -n "$TEST_TARGET" ]; then
if [ -f "/opt/ros/%{ros_distro}/setup.sh" ]; then . "/opt/ros/%{ros_distro}/setup.sh"; fi
CTEST_OUTPUT_ON_FAILURE=1 \
    %make_build -C .obj-%{_target_platform} $TEST_TARGET || echo "RPM TESTS FAILED"
else echo "RPM TESTS SKIPPED"; fi
%endif

%files
/opt/ros/%{ros_distro}

%changelog
* Mon Dec 23 2025 tyns_tarsier-infra <yelishuang23@mails.usas.ac.cn> - 2.0.9004-1
- Update to version 2.0.9004
- Update abseil macros patch for newer abseil-cpp
