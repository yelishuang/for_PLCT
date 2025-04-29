#!/bin/bash

# 定义变量
NGINX_CONFIG_SRC="./nginx.conf"
HTML_DIR_SRC="./html"
NGINX_CONFIG_DST="/etc/nginx/nginx.conf"
NGINX_HTML_DST="/usr/share/nginx/html" 

# 1. 安装 Nginx
echo "正在检查 Nginx 安装..."
if ! rpm -q nginx > /dev/null 2>&1; then
    echo "Nginx 未安装，开始安装..."
    sudo dnf install -y nginx
    if [ $? -ne 0 ]; then
        echo "安装 Nginx 失败！请检查网络或依赖关系。"
        exit 1
    fi
else
    echo "Nginx 已安装，跳过安装步骤。"
fi

# 2. 替换 Nginx 配置文件和 HTML 目录
echo "正在替换 Nginx 配置文件..."
if [ -f "$NGINX_CONFIG_SRC" ]; then
    sudo cp "$NGINX_CONFIG_SRC" "$NGINX_CONFIG_DST"
    echo "配置文件已替换为当前目录的 nginx.conf"
else
    echo "警告：当前目录下未找到 nginx.conf 文件，跳过配置文件替换！"
fi

echo "正在替换 Nginx HTML 目录..."
if [ -d "$HTML_DIR_SRC" ]; then
    sudo rm -rf "$NGINX_HTML_DST"/*
    sudo cp -r "$HTML_DIR_SRC"/* "$NGINX_HTML_DST"
    echo "HTML 目录已替换为当前目录的 html 文件夹内容"
else
    echo "警告：当前目录下未找到 html 文件夹，跳过 HTML 目录替换！"
fi

# 3. 安装 pam 和 pam-devel
echo "正在检查 pam 和 pam-devel..."
for package in pam pam-devel; do
    if ! rpm -q "$package" > /dev/null 2>&1; then
        echo "$package 未安装，开始安装..."
        sudo dnf install -y "$package"
        if [ $? -ne 0 ]; then
            echo "安装 $package 失败！请检查网络或依赖关系。"
            exit 1
        fi
    else
        echo "$package 已安装，跳过。"
    fi
done

# 4. 安装 Python 依赖
echo "正在检查 Python 依赖..."

# 需要安装的 Python 包列表
PACKAGES=("python-pam" "Flask" "flask-cors" "websockets" "psutil")

for package in "${PACKAGES[@]}"; do
    if ! pip list | grep "$package" &> /dev/null; then
        echo "$package 未安装，开始安装..."
        sudo pip3 install "$package"
        if [ $? -ne 0 ]; then
            echo "安装 $package 失败！请检查 Python 环境。"
            exit 1
        fi
    else
        echo "$package 已安装，跳过。"
    fi
done

# 5. 运行 login.py
echo "正在启动 login.py..."
nohup python3 ./login.py > login.log 2>&1 &
echo "login.py 已在后台运行，日志保存在 login.log"

# 6. 重启 Nginx
echo "正在重启 Nginx..."
sudo systemctl restart nginx
if [ $? -ne 0 ]; then
    echo "重启 Nginx 失败！请检查配置文件或服务状态。"
    exit 1
else
    echo "Nginx 已成功重启"
fi

echo "所有操作完成！"
