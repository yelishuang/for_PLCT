import json
import random
import time
import requests
from bs4 import BeautifulSoup
from urllib.parse import urlparse
import  os


# 常见的浏览器 User - Agent
user_agents = [
    "Mozilla/5.0 (Windows NT 10.0; Win64; x64) AppleWebKit/537.36 (KHTML, like Gecko) Chrome/91.0.4472.124 Safari/537.36",
    "Mozilla/5.0 (Windows NT 10.0; Win64; x64; rv:89.0) Gecko/20100101 Firefox/89.0",
    "Mozilla/5.0 (Macintosh; Intel Mac OS X 10_15_7) AppleWebKit/605.1.15 (KHTML, like Gecko) Version/14.1.1 Safari/605.1.15"
]

# 选择一个 User - Agent
user_agent = random.choice(user_agents)

# 构建请求头
headers = {
    "User-Agent": user_agent,
    "Accept": "text/html,application/xhtml+xml,application/xml;q=0.9,image/avif,image/webp,image/apng,*/*;q=0.8,application/signed-exchange;v=b3;q=0.9",
    "Accept-Language": "zh-CN,zh;q=0.9",
    "Accept-Encoding": "gzip, deflate, br",
    "Connection": "keep-alive"
}


def get_with_retries(url, headers=None, max_retries=10):
    attempt = 0
    while attempt < max_retries:
        time.sleep(random.uniform(0, 1))
        try:
            response = requests.get(url, headers=headers)
            response.raise_for_status()  # 如果响应状态码不是200，抛出异常
            return response
        except Exception as e:
            attempt += 1
    print_color(f"连接时出错：{url}",'31')


def get_unique_file_path(folder, file_name):
    base_name, ext = os.path.splitext(file_name)
    counter = 0

    while True:
        if folder:
            full_path = os.path.join(folder, f"{base_name}_{counter}{ext}" if counter > 0 else file_name)
        else:
            full_path = f"{base_name}_{counter}{ext}" if counter > 0 else file_name

        if not os.path.exists(full_path):
            os.makedirs(os.path.dirname(full_path), exist_ok=True)
            return full_path

        counter += 1

def print_color(text, color_code):
    print(f"\033[{color_code}m{text}\033[0m")

def identify_source(url):
    try:
        # 解析网址
        parsed_url = urlparse(url)
        domain = parsed_url.netloc  # 获取域名部分
        if "bitbucket.org" in domain:
            return 1
        elif "github.com" in domain:
            return 2
        elif "gitlab.com" in domain:
            return 3
        else:
            return "Unknown"
    except Exception as e:
        print(f"解析网址时出错：{e}")
        return "Invalid URL"


# 对指定输入网站获取其目录下所有package.xml文件,搜索到即终止递归搜索
def get_repo_package_github(repo_url, parent_has_package_xml=False):
    # 初始化存储文件路径的列表
    file_paths = []
    directory_paths = []
    found_package_xml = False

    # 发送HTTP请求获取页面内容
    response=get_with_retries(repo_url)
    if(response is None):
        return []

    # 使用BeautifulSoup解析HTML内容
    soup = BeautifulSoup(response.content, 'html.parser')

    # 找到所有文件和目录的链接
    links = soup.find_all('td', class_='react-directory-row-name-cell-large-screen', colspan ="1")

    for link in links:
        for child in link.descendants:
            if hasattr(child, 'name') and child.name and child.get('href'):
                href = child.get('href')
                aria_label = child.get('aria-label')
                if "Directory" in aria_label:
                    directory_paths.append(f"https://github.com{href}/")
                if child.text == "package.xml":
                    found_package_xml = True
                    file_paths.append(f"https://github.com{href}")

    if not found_package_xml :
        for repo_url in directory_paths:
            file_paths.extend(get_repo_package_github(repo_url))

    return file_paths

from urllib.parse import urlparse, unquote


def download_github_file(github_url, folder=None):
    """
    通过 GitHub 文件链接下载文件，并将文件名设置为 URL 的倒数第二个字段加 `.xml`。

    参数:
        github_url (str): GitHub 文件链接（以 /blob/ 开头）。
        folder (str, 可选): 保存文件的文件夹路径。如果未指定，则保存到当前目录。

    返回:
        str: 下载的文件路径，如果失败则返回 None。
    """
    time.sleep(random.uniform(0, 1))
    try:
        # 解析 URL
        parsed_url = urlparse(github_url)
        path_parts = parsed_url.path.split('/')

        # 提取倒数第二个字段作为文件名
        file_name_part = path_parts[-2]
        file_name = f"{file_name_part}.xml"

        file_path=get_unique_file_path(folder,file_name)

        # 将 GitHub 文件 URL 转换为 Raw URL
        raw_url = github_url.replace("github.com", "raw.githubusercontent.com").replace("/blob/", "/")

        # 请求文件内容
        with requests.get(raw_url, stream=True) as response:
            response.raise_for_status()  # 确保请求成功

            # 保存文件
            with open(file_path, 'wb') as file:
                for chunk in response.iter_content(chunk_size=8192):
                    file.write(chunk)

        print_color(f"文件已成功下载并保存为: {file_path}",'33')
        return file_path

    except requests.exceptions.RequestException as e:
        print_color(f"下载失败: {e}",'31')
        return None
    except Exception as e:
        print_color(f"发生错误: {e}",'31')
        return None


def get_repo_url(url = 'http://repo.ros2.org/status_page/ros_humble_default.html'):
    dict={}
    # 模拟浏览器访问
    response=get_with_retries(url)
    soup = BeautifulSoup(response.text, 'html.parser')
    # 查找所有的 <tr> 标签
    tr_elements = soup.find_all('tr')
    del tr_elements[0]
    # 遍历每个 <tr> 标签
    for tr in tr_elements:
        # 提取 项目名
        name = tr.select_one('td:first-child div').text
        # 提取 GitHub 链接
        url = tr.select_one('td:nth-child(2) .repo a')['href']
        # 提取 rope 名称
        repo = tr.select_one('td:nth-child(2) .repo a').text
        # 提取 项目状态
        status = tr.select_one('td:nth-child(4) span')['class'][0]
        if (repo not in dict.keys()):
            dict[repo] = [url, status, [name]]
        else:
            dict[repo][2].append(name)
    return  dict

if __name__ == "__main__":
    start_time = time.time()
    file_path = "repo.json"
    file_path1 = "package_url.json"
    package_dict={}
    repo_dict=get_repo_url()

    for key in repo_dict:
        if(identify_source(repo_dict[key][0])==2):
            package_dict[key]={}
            for url in get_repo_package_github(repo_dict[key][0]):
                package_dict[key][download_github_file(url,key)]=url
            print_color("完成"+key+"仓库分析","34")

    # 打开文件并写入JSON数据
    with open(file_path, 'w', encoding='utf-8') as file:
        # 使用json.dump将字典写入文件
        json.dump(repo_dict, file, ensure_ascii=False, indent=4)
    print(f"字典已成功写入到文件 {file_path}")

    with open(file_path1, 'w', encoding='utf-8') as file:
        # 使用json.dump将字典写入文件
        json.dump(package_dict, file, ensure_ascii=False, indent=4)
    print(f"字典已成功写入到文件 {file_path1}")

    end_time = time.time()  # 记录结束时间
    elapsed_time = end_time - start_time  # 计算耗时
    print(f"Execution time: {elapsed_time:.2f} seconds")  # 输出耗时
