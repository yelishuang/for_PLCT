from copr.v3 import Client

client = Client.create_from_config_file()
# 定义包的信息
package = {
    "name": "ros-humble-ament-clang-format",  # package name
    "source_type": "scm",
    "source_dict": {
        "clone_url": "https://gitee.com/davidhan008/ament_lint.git",  # gitee link
        "committish": "humble",
        "subdirectory": "",
        "spec": "ament-clang-format.spec",  # The spec name that needs to be built
        "scm_type": "git"
    }
}

# 指定 Copr 项目名称
project_name = "test"

# 提交包到指定的 Copr 项目
try:
    print("start project")
    project = client.project_proxy.get(ownername="listen_rain", projectname=project_name)

    # 提交包
    client.package_proxy.add(
        ownername="listen_rain",
        projectname=project_name,
        packagename=package["name"],
        source_type=package["source_type"],
        source_dict=package["source_dict"]
    )
    print(f"Package {package['name']} submitted successfully.")
except CoprNoResultException as e:
    print(f"Error submitting package {package['name']}: {e}")
