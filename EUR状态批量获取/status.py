from copr.v3 import config_from_file
from copr.v3 import BuildProxy
import json


ownername = "davidhan008"
projectname = "ROS-SIG-Multi-Version_ros-humble_openEuler-24.03-LTS-TEST1"
    
# 1. 读取 JSON 文件产生字典
def load_json(filename):
    with open(filename, 'r') as file:
        return json.load(file)

# 2. 处理包名并查找对应仓库
def process_build_info(build_info_list, package_dict):
    processed_data = []
    for build in build_info_list:
        build_id = build.id
        source_package = build.get('source_package', {})
        package_name = source_package.get('name', '')
        repo = ""
        state = build.get('state', "N/A")
        if state in ['forked', 'succeeded', 'skipped']:
        	url = source_package.get('url', "N/A")
        else:
        	url = 'https://eur.openeuler.openatom.cn/coprs/'+ownername+'/'+projectname+'/build/'+str(build_id)+'/'
        if package_name is None:
            print(f"存在一个包名无法获取，值为none，ID为{build_id}")
            continue
        # 去除包名前缀 'ros-humble-'
        package_name_clean = package_name.replace('ros-humble-', '', 1) if 'ros-humble-' in package_name else package_name
        # 查找对应仓库
        for key, value in package_dict.items():
            if package_name_clean in value:
                repo = key
        if repo == "":
            repo = "N/A"
        processed_data.append({
            'build_id': build_id,
            'package_name': package_name_clean,
            'repo': repo,
            'url': url,
            'state': state
        })
    return processed_data

# 3. 统计成功和失败的构建状态
def calculate_statistics(processed_data):
    success_count = 0
    failed_count = 0
    success_list = []
    failed_list = []

    for item in processed_data:
        state = item['state']
        if state in ['forked', 'succeeded', 'skipped']:
            success_count += 1
            success_list.append(item)
        elif state == 'failed':
            failed_count += 1
            failed_list.append(item)

    return success_count, success_list, failed_count, failed_list

# 4. 输出结果到 TXT 文件
def generate_output_file(success_count, success_list, failed_count, failed_list, output_file):
    with open(output_file, 'w') as file:
        # 输出成功构建的统计和信息
        file.write(f"共成功构建 {success_count} 包\n")
        for item in success_list:
            file.write(f"包名: {item['package_name']} 仓库: {item['repo']} url: {item['url']} 状态: {item['state']}\n")

        # 输出失败构建的统计和信息
        file.write(f"\n共失败构建 {failed_count} 包\n")
        for item in failed_list:
            file.write(f"包名: {item['package_name']} 仓库: {item['repo']} url: {item['url']} 状态: {item['state']} id:{item['build_id']}\n")

# 主函数
def main():
    # 配置信息
    config = config_from_file()
    build_proxy = BuildProxy(config)
    package_dict_filename = "package.json"  # 假设 JSON 文件名为 package_dict.json
    output_filename = "output.txt"

    # 从 Copr 获取构建信息
    build_info_list = build_proxy.get_list(
        ownername=ownername,
        projectname=projectname,
        packagename=None
    )

    # 1. 读取 JSON 文件
    package_dict = load_json(package_dict_filename)

    # 2. 处理构建信息
    processed_data = process_build_info(build_info_list, package_dict)

    # 3. 统计信息
    success_count, success_list, failed_count, failed_list = calculate_statistics(processed_data)

    # 4. 输出结果
    generate_output_file(success_count, success_list, failed_count, failed_list, output_filename)

    print(f"结果已保存到 {output_filename}")

if __name__ == "__main__":
    main()
