%bcond_without tests
%bcond_without weak_deps

%global debug_package %{nil}
%global __os_install_post %(echo '%{__os_install_post}' | sed -e 's!/usr/lib[^[:space:]]*/brp-python-bytecompile[[:space:]].*$!!g')
%global __provides_exclude_from ^/opt/ros/%{ros_distro}/.*$
%global __requires_exclude_from ^/opt/ros/%{ros_distro}/.*$

%define RosPkgName      system-fingerprint

Name:           ros-%{ros_distro}-%{RosPkgName}
Version:        1
Release:        None%{?dist}%{?release_suffix}
Summary:        The system_fingerprint package

License:        BSD 2-clause
Source0:        %{name}_%{version}.orig.tar.gz

Requires: ros-%{ros_distro}-rcl-interfaces
Requires: ros-%{ros_distro}-rclpy
Requires: ros-%{ros_distro}-ros2action
Requires: ros-%{ros_distro}-ros2cli
Requires: ros-%{ros_distro}-ros2node
Requires: ros-%{ros_distro}-ros2param
Requires: ros-%{ros_distro}-ros2service
Requires: ros-%{ros_distro}-ros2topic
Requires: ros-%{ros_distro}-tf2-msgs
Requires: python3-git
Requires: ros-%{ros_distro}-ros-workspace

BuildRequires: ros-%{ros_distro}-rcl-interfaces
BuildRequires: ros-%{ros_distro}-rclpy
BuildRequires: ros-%{ros_distro}-ros2action
BuildRequires: ros-%{ros_distro}-ros2cli
BuildRequires: ros-%{ros_distro}-ros2node
BuildRequires: ros-%{ros_distro}-ros2param
BuildRequires: ros-%{ros_distro}-ros2service
BuildRequires: ros-%{ros_distro}-ros2topic
BuildRequires: ros-%{ros_distro}-tf2-msgs
BuildRequires: ros-%{ros_distro}-ros-workspace

%if 0%{?with_tests}
%endif

Provides:       %{name}-devel = %{version}-%{release}
Provides:       %{name}-doc = %{version}-%{release}
Provides:       %{name}-runtime = %{version}-%{release}

%description
The system_fingerprint package

%prep
%autosetup -p1

%build

# In case we're installing to a non-standard location, look for a setup.sh
# in the install tree and source it.  It will set things like
# CMAKE_PREFIX_PATH, PKG_CONFIG_PATH, and PYTHONPATH.
if [ -f "/opt/ros/%{ros_distro}/setup.sh" ]; then . "/opt/ros/%{ros_distro}/setup.sh"; fi
%py3_build

%install
# In case we're installing to a non-standard location, look for a setup.sh
# in the install tree and source it.  It will set things like
# CMAKE_PREFIX_PATH, PKG_CONFIG_PATH, and PYTHONPATH.
if [ -f "/opt/ros/%{ros_distro}/setup.sh" ]; then . "/opt/ros/%{ros_distro}/setup.sh"; fi
%py3_install -- --prefix "/opt/ros/%{ros_distro}"

%if 0%{?with_tests}
%check
# Look for a directory with a name indicating that it contains tests
TEST_TARGET=$(ls -d * | grep -m1 "\(test\|tests\)" ||:)
if [ -n "$TEST_TARGET" ] && %__python3 -m pytest --version; then
# In case we're installing to a non-standard location, look for a setup.sh
# in the install tree and source it.  It will set things like
# CMAKE_PREFIX_PATH, PKG_CONFIG_PATH, and PYTHONPATH.
if [ -f "/opt/ros/%{ros_distro}/setup.sh" ]; then . "/opt/ros/%{ros_distro}/setup.sh"; fi
%__python3 -m pytest $TEST_TARGET || echo "RPM TESTS FAILED"
else echo "RPM TESTS SKIPPED"; fi
%endif

%files
/opt/ros/%{ros_distro}

%changelog
* Sun Dec 22 2024 David V. Lu!! davidvlu@gmail.com - 1-None
- Autogenerated by ros-porting-tools
