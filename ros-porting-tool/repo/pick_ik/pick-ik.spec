%bcond_without tests
%bcond_without weak_deps

%global __os_install_post %(echo '%{__os_install_post}' | sed -e 's!/usr/lib[^[:space:]]*/brp-python-bytecompile[[:space:]].*$!!g')
%global __provides_exclude_from ^/opt/ros/%{ros_distro}/.*$
%global __requires_exclude_from ^/opt/ros/%{ros_distro}/.*$

%define RosPkgName      pick-ik

Name:           ros-%{ros_distro}-%{RosPkgName}
Version:        1
Release:        None%{?dist}%{?release_suffix}
Summary:        Inverse Kinematics solver for MoveIt

License:        BSD-3-Clause
Source0:        %{name}_%{version}.orig.tar.gz

Patch0: pick-ik-fix-Catch2-error.patch

Requires: fmt-devel
Requires: ros-%{ros_distro}-generate-parameter-library
Requires: ros-%{ros_distro}-moveit-core
Requires: ros-%{ros_distro}-pluginlib
Requires: range-v3-devel
Requires: ros-%{ros_distro}-rclcpp
Requires: ros-%{ros_distro}-rsl
Requires: ros-%{ros_distro}-tf2-geometry-msgs
Requires: ros-%{ros_distro}-tf2-kdl
Requires: ros-%{ros_distro}-tl-expected
Requires: ros-%{ros_distro}-ros-workspace

BuildRequires: fmt-devel
BuildRequires: ros-%{ros_distro}-generate-parameter-library
BuildRequires: ros-%{ros_distro}-moveit-core
BuildRequires: ros-%{ros_distro}-pluginlib
BuildRequires: range-v3-devel
BuildRequires: ros-%{ros_distro}-rclcpp
BuildRequires: ros-%{ros_distro}-rsl
BuildRequires: ros-%{ros_distro}-tf2-geometry-msgs
BuildRequires: ros-%{ros_distro}-tf2-kdl
BuildRequires: ros-%{ros_distro}-tl-expected
BuildRequires: ros-%{ros_distro}-ament-cmake-ros
BuildRequires: Catch2-devel
BuildRequires: ros-%{ros_distro}-ros-workspace

%if 0%{?with_tests}
BuildRequires: ros-%{ros_distro}-moveit-resources-panda-moveit-config
%endif

Provides:       %{name}-devel = %{version}-%{release}
Provides:       %{name}-doc = %{version}-%{release}
Provides:       %{name}-runtime = %{version}-%{release}

%description
Inverse Kinematics solver for MoveIt

%prep
%autosetup -p1

%build
# Needed to bootstrap since the ros_workspace package does not yet exist.
export PYTHONPATH=/opt/ros/%{ros_distro}/lib/python%{python3_version}/site-packages

export ROS_DISTRO=%{ros_distro}
export ROS_PYTHON_VERSION=%{python3_version}

# In case we're installing to a non-standard location, look for a setup.sh
# in the install tree and source it.  It will set things like
# CMAKE_PREFIX_PATH, PKG_CONFIG_PATH, and PYTHONPATH.
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
    -DCMAKE_INSTALL_LIBDIR="/opt/ros/%{ros_distro}/lib" \
    -DSETUPTOOLS_DEB_LAYOUT=OFF \
%if !0%{?with_tests}
    -DBUILD_TESTING=OFF \
%endif
    ..

%make_build

%install
# Needed to bootstrap since the ros_workspace package does not yet exist.
export PYTHONPATH=/opt/ros/%{ros_distro}/lib/python%{python3_version}/site-packages

# In case we're installing to a non-standard location, look for a setup.sh
# in the install tree and source it.  It will set things like
# CMAKE_PREFIX_PATH, PKG_CONFIG_PATH, and PYTHONPATH.
if [ -f "/opt/ros/%{ros_distro}/setup.sh" ]; then . "/opt/ros/%{ros_distro}/setup.sh"; fi
%make_install -C .obj-%{_target_platform}

%if 0%{?with_tests}
%check
# Needed to bootstrap since the ros_workspace package does not yet exist.
export PYTHONPATH=/opt/ros/%{ros_distro}/lib/python%{python3_version}/site-packages

# Look for a Makefile target with a name indicating that it runs tests
TEST_TARGET=$(%__make -qp -C .obj-%{_target_platform} | sed "s/^\(test\|check\):.*/\\1/;t f;d;:f;q0")
if [ -n "$TEST_TARGET" ]; then
# In case we're installing to a non-standard location, look for a setup.sh
# in the install tree and source it.  It will set things like
# CMAKE_PREFIX_PATH, PKG_CONFIG_PATH, and PYTHONPATH.
if [ -f "/opt/ros/%{ros_distro}/setup.sh" ]; then . "/opt/ros/%{ros_distro}/setup.sh"; fi
CTEST_OUTPUT_ON_FAILURE=1 \
    %make_build -C .obj-%{_target_platform} $TEST_TARGET || echo "RPM TESTS FAILED"
else echo "RPM TESTS SKIPPED"; fi
%endif

%files
/opt/ros/%{ros_distro}

%changelog
* Sun Dec 22 2024 Tyler Weaver maybe@tylerjw.dev - 1-None
- Autogenerated by ros-porting-tools