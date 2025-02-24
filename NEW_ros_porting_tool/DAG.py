import json
import os
import xml.etree.ElementTree as ET
import logging
import colorlog

# 配置日志记录
handler = colorlog.StreamHandler()
handler.setFormatter(colorlog.ColoredFormatter(
    '%(log_color)s%(asctime)s - %(levelname)s - %(message)s',
    log_colors={
        'DEBUG': 'cyan',
        'INFO': 'green',
        'WARNING': 'yellow',
        'ERROR': 'red',
        'CRITICAL': 'red,bg_white',
    }
))
logger = colorlog.getLogger()
logger.addHandler(handler)
logger.setLevel(logging.INFO)


class PackageDependency:
    def __init__(self, filename):
        self.filename = filename
        self.dependencies = {}
        self.load_dependencies()
        # 加载依赖后进行递归依赖消除
        self._optimize_all_dependencies()

    def load_dependencies(self):
        try:
            with open(self.filename, 'r') as file:
                self.dependencies = json.load(file)
            logger.info(f"Dependencies loaded successfully from {self.filename}")
        except FileNotFoundError:
            logger.error(f"File {self.filename} not found.")
        except json.JSONDecodeError:
            logger.error(f"Error decoding JSON from file {self.filename}.")
        except Exception as e:
            logger.error(f"An unexpected error occurred: {e}")

    def save_dependencies(self):
        try:
            with open(self.filename, 'w') as file:
                json.dump(self.dependencies, file, indent=4)
            logger.info(f"Dependencies saved successfully to {self.filename}")
        except IOError as e:
            logger.error(f"IO error saving dependencies: {e}")
        except Exception as e:
            logger.error(f"An unexpected error occurred while saving dependencies: {e}")

    def add_package(self, package_name, dependencies=None):
        if dependencies is None:
            dependencies = []

        # 确保所有依赖项存在于依赖字典中
        for dep in dependencies:
            if dep not in self.dependencies:
                self.dependencies[dep] = []
                logger.info(f"Added missing dependency '{dep}' with empty dependencies.")

        if package_name in self.dependencies:
            logger.warning(f"Package '{package_name}' already exists. Overwriting its dependencies.")

        # 优化依赖关系
        optimized_dependencies = self._optimize_dependencies(dependencies)

        # Add the new package temporarily and check for cycles
        temp_dependencies = self.dependencies.copy()
        temp_dependencies[package_name] = optimized_dependencies
        if self._has_cycle(temp_dependencies):
            logger.error(f"Adding package '{package_name}' would introduce a cycle.")
            return False

        self.dependencies[package_name] = optimized_dependencies
        self.save_dependencies()
        logger.info(f"Package '{package_name}' added successfully.")
        return True

    def get_dependencies(self, package_name):
        return self.dependencies.get(package_name, [])

    def remove_package(self, package_name):
        if package_name in self.dependencies:
            # Remove the package temporarily and check for cycles
            temp_dependencies = self.dependencies.copy()
            del temp_dependencies[package_name]
            if self._has_cycle(temp_dependencies):
                logger.error(f"Removing package '{package_name}' would introduce a cycle.")
                return False

            del self.dependencies[package_name]
            self.save_dependencies()
            logger.info(f"Package '{package_name}' removed successfully.")
            return True
        else:
            logger.warning(f"Package '{package_name}' does not exist.")
            return False

    def update_dependencies(self, package_name, new_dependencies):
        if package_name in self.dependencies:
            # 确保所有新依赖项存在于依赖字典中
            for dep in new_dependencies:
                if dep not in self.dependencies:
                    self.dependencies[dep] = []
                    logger.info(f"Added missing dependency '{dep}' with empty dependencies.")

            # 优化依赖关系
            optimized_dependencies = self._optimize_dependencies(new_dependencies)

            # Update the dependencies temporarily and check for cycles
            temp_dependencies = self.dependencies.copy()
            temp_dependencies[package_name] = optimized_dependencies
            if self._has_cycle(temp_dependencies):
                logger.error(f"Updating dependencies of package '{package_name}' would introduce a cycle.")
                return False

            self.dependencies[package_name] = optimized_dependencies
            self.save_dependencies()
            logger.info(f"Dependencies of package '{package_name}' updated successfully.")
            return True
        else:
            logger.warning(f"Package '{package_name}' does not exist.")
            return False

    def _has_cycle(self, dependencies):
        visited = set()
        recursion_stack = set()

        def dfs(node):
            if node in recursion_stack:
                return True
            if node in visited:
                return False

            visited.add(node)
            recursion_stack.add(node)

            for neighbor in dependencies.get(node, []):
                if dfs(neighbor):
                    return True

            recursion_stack.remove(node)
            return False

        for node in dependencies.keys():
            if dfs(node):
                return True
        return False

    def list_all_packages(self):
        return list(self.dependencies.keys())

    def _optimize_dependencies(self, dependencies):
        optimized = []
        for dep in dependencies:
            if not any(dep in self._get_all_dependencies(sub_dep) for sub_dep in dependencies if sub_dep != dep):
                optimized.append(dep)
        return optimized

    def _get_all_dependencies(self, package_name):
        all_deps = set()
        stack = [package_name]
        while stack:
            current = stack.pop()
            for dep in self.dependencies.get(current, []):
                if dep not in all_deps:
                    all_deps.add(dep)
                    stack.append(dep)
        return all_deps

    def _optimize_all_dependencies(self):
        for package, deps in self.dependencies.items():
            self.dependencies[package] = self._optimize_dependencies(deps)
        self.save_dependencies()
        logger.info("All dependencies have been optimized.")


def load_json_file(file_path):
    """
    加载 JSON 文件
    :param file_path: JSON 文件路径
    :return: 解析后的 JSON 数据
    """
    try:
        with open(file_path, 'r', encoding='utf-8') as file:
            return json.load(file)
    except FileNotFoundError:
        print(f"Error: JSON file {file_path} not found.")
    except json.JSONDecodeError:
        print(f"Error: Failed to decode JSON file {file_path}.")
    return {}


def get_dependency_tags(root_element, tag_names):
    """
    获取指定标签名的依赖项列表
    :param root_element: XML 根元素
    :param tag_names: 标签名列表
    :return: 依赖项列表
    """
    dependencies = []
    for tag_name in tag_names:
        for tag in root_element.findall(tag_name):
            dependencies.append(tag.text)
    return dependencies


def analyze_package_xml(directory, package_url):
    """
    分析指定目录下的所有 XML 文件，提取依赖信息
    :param directory: 目录路径
    :param package_url: 包的 URL 信息
    :return: 包含更新后的 package_url、BuildRequires 和 Requires 信息的字典
    """
    build_requires_dict = {}
    requires_dict = {}

    for root, _, files in os.walk(directory):
        for file in files:
            if not file.endswith('.xml'):
                continue

            xml_file_path = os.path.join(root, file)
            relative_path = os.path.relpath(xml_file_path, directory)

            try:
                tree = ET.parse(xml_file_path)
                root_element = tree.getroot()
                name_element = root_element.find('name')

                if name_element is None:
                    print(f"Error: <name> tag not found in {relative_path}")
                    continue

                package_name = name_element.text
                parent_dir = xml_file_path.split(os.sep)[-2]
                old_key = xml_file_path.removeprefix("./xml" + os.sep)

                package_url[parent_dir][package_name] = package_url[parent_dir].pop(old_key, None).replace('package.xml', '')

                # 获取构建依赖
                build_tags = ['build_depend', 'buildtool_depend', 'depend']
                build_requires = get_dependency_tags(root_element, build_tags)
                build_requires_dict[package_name] = build_requires

                # 获取运行依赖
                exec_tags = ['depend', 'exec_depend']
                requires = get_dependency_tags(root_element, exec_tags)
                requires_dict[package_name] = requires

                new_file_name = f"{package_name}.xml"
                new_file_path = os.path.join(root, new_file_name)
                os.rename(xml_file_path, new_file_path)
                print(f"Renamed {relative_path} to {os.path.relpath(new_file_path, directory)}")

            except ET.ParseError:
                print(f"Error: Failed to parse {relative_path}")
            except Exception as e:
                print(f"Error: {e} occurred while processing {relative_path}")

    result = {
        'package_url': package_url,
        'BuildRequires': build_requires_dict,
        'Requires': requires_dict
    }
    return result


def find_root_nodes(dependency_dict):
    """
    找出所有不需要任何依赖的根节点
    :param dependency_dict: 依赖字典
    :return: 根节点集合
    """
    root_nodes = set()
    for package, deps in dependency_dict.items():
        if not deps:
            root_nodes.add(package)
    return root_nodes


def count_supported_packages(root_node, dependency_dict):
    """
    统计根节点所支撑的包的数量
    :param root_node: 根节点
    :param dependency_dict: 依赖字典
    :return: 根节点所支撑的包的数量
    """
    supported_packages = set()
    stack = [root_node]
    while stack:
        current = stack.pop()
        for package, deps in dependency_dict.items():
            if current in deps:
                if package not in supported_packages:
                    supported_packages.add(package)
                    stack.append(package)
    return len(supported_packages)


def get_top_root_nodes(dependency_dict, top_n=10):
    """
    获取支撑包最多的前 n 个根节点
    :param dependency_dict: 依赖字典
    :param top_n: 前 n 个根节点
    :return: 前 n 个支撑包最多的根节点及其支撑的包的数量
    """
    root_nodes = find_root_nodes(dependency_dict)
    root_node_counts = []
    for root_node in root_nodes:
        count = count_supported_packages(root_node, dependency_dict)
        root_node_counts.append((root_node, count))
    root_node_counts.sort(key=lambda x: x[1], reverse=True)
    return root_node_counts[:top_n]


if __name__ == "__main__":
    target_directory = './xml'
    json_file_path = 'package_url.json'

    package_url = load_json_file(json_file_path)
    result = analyze_package_xml(target_directory, package_url)

    package_url = result['package_url']
    build_requires_dict = result['BuildRequires']
    requires_dict = result['Requires']

    # 保存更新后的 package_url
    with open(json_file_path, 'w', encoding='utf-8') as f:
        json.dump(package_url, f, indent=4)

    # 为 BuildRequires 构建 DAG
    build_requires_dag = PackageDependency('build_requires_dag.json')
    for package, deps in build_requires_dict.items():
        build_requires_dag.add_package(package, deps)

    # 为 Requires 构建 DAG
    requires_dag = PackageDependency('requires_dag.json')
    for package, deps in requires_dict.items():
        requires_dag.add_package(package, deps)

    # 获取前 10 个支撑包最多的构建根节点
    top_build_root_nodes = get_top_root_nodes(build_requires_dict)
    # 获取前 10 个支撑包最多的运行根节点
    top_requires_root_nodes = get_top_root_nodes(requires_dict)

    # 保存包含统计信息的 build_requires_dag.json
    build_requires_dag_data = {
        "dependencies": build_requires_dag.dependencies,
        "top_10_root_nodes": [{"root_node": node, "supported_count": count} for node, count in top_build_root_nodes]
    }
    with open('build_requires_dag.json', 'w', encoding='utf-8') as f:
        json.dump(build_requires_dag_data, f, indent=4)

    # 保存包含统计信息的 requires_dag.json
    requires_dag_data = {
        "dependencies": requires_dag.dependencies,
        "top_10_root_nodes": [{"root_node": node, "supported_count": count} for node, count in top_requires_root_nodes]
    }
    with open('requires_dag.json', 'w', encoding='utf-8') as f:
        json.dump(requires_dag_data, f, indent=4)
