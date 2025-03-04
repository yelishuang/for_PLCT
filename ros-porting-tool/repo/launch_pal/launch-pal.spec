%bcond_without tests
%bcond_without weak_deps

%global debug_package %{nil}
%global __os_install_post %(echo '%{__os_install_post}' | sed -e 's!/usr/lib[^[:space:]]*/brp-python-bytecompile[[:space:]].*$!!g')
%global __provides_exclude_from ^/opt/ros/%{ros_distro}/.*$
%global __requires_exclude_from ^/opt/ros/%{ros_distro}/.*$

%define RosPkgName      launch-pal

Name:           ros-%{ros_distro}-%{RosPkgName}
Version:        1
Release:        None%{?dist}%{?release_suffix}
Summary:        Utilities for launch files

Url:            https://github.com/pal-robotics/launch_pal
License:        Apache License 2.0
Source0:        %{name}_%{version}.orig.tar.gz

Requires: ros-%{ros_distro}-ament-index-python
Requires: ros-%{ros_distro}-launch
Requires: ros-%{ros_distro}-launch-ros
Requires: python3-yaml
Requires: ros-%{ros_distro}-ros-workspace

BuildRequires: ros-%{ros_distro}-ament-index-python
BuildRequires: ros-%{ros_distro}-launch
BuildRequires: ros-%{ros_distro}-launch-ros
BuildRequires: python3-yaml
BuildRequires: ros-%{ros_distro}-ros-workspace

%if 0%{?with_tests}
BuildRequires: ros-%{ros_distro}-ament-copyright
BuildRequires: ros-%{ros_distro}-ament-flake8
BuildRequires: ros-%{ros_distro}-ament-pep257
BuildRequires: python3-pytest
%endif

Provides:       %{name}-devel = %{version}-%{release}
Provides:       %{name}-doc = %{version}-%{release}
Provides:       %{name}-runtime = %{version}-%{release}

%description
Utilities for launch files

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
* Sun Dec 22 2024 Jordan Palacios jordan.palacios@pal-robotics.com - 1-None
- Autogenerated by ros-porting-tools
