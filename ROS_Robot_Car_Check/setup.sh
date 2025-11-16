#!/bin/bash

# 定义变量
NGINX_CONFIG_SRC="./nginx.conf"
HTML_DIR_SRC="./html"
NGINX_CONFIG_DST="/etc/nginx/nginx.conf"
NGINX_HTML_DST="/usr/share/nginx/html"

# 检测系统类型
if [ -f /etc/os-release ]; then
    . /etc/os-release
    OS=$ID
else
    echo "无法检测系统类型"
    exit 1
fi

# 根据系统类型设置包管理器和依赖包名
case $OS in
    "openEuler"|"centos"|"rhel"|"fedora")
        PKG_MANAGER="dnf"
        SYSTEM_DEPS="nginx pam pam-devel python3-pip python3-systemd"
        ;;
    "ubuntu"|"debian"|"raspbian")
        PKG_MANAGER="apt"
        SYSTEM_DEPS="nginx libpam0g libpam0g-dev python3-pip python3-systemd"
        ;;
    *)
        echo "不支持的系统: $OS"
        echo "支持的系统: openeuler, centos, rhel, fedora, ubuntu, debian, raspbian"
        exit 1
        ;;
esac

# 1. 安装系统依赖
echo "正在安装系统依赖 (使用 $PKG_MANAGER)..."
sudo $PKG_MANAGER install -y $SYSTEM_DEPS
if [ $? -ne 0 ]; then
    echo "系统依赖安装失败！请检查网络或软件源配置。"
    exit 1
fi
echo "系统依赖安装完成。"

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

# 3. 安装 Python 依赖
echo "正在安装 Python 依赖..."
if [ -f "requirements.txt" ]; then
    pip3 install -r requirements.txt --quiet
    if [ $? -ne 0 ]; then
        echo "Python 依赖安装失败！请检查 requirements.txt 文件。"
        exit 1
    fi
    echo "Python 依赖安装完成。"
else
    echo "警告：未找到 requirements.txt 文件！"
    echo "请手动安装以下 Python 包："
    echo "  Flask flask-cors psutil python-pam pytz"
fi

# 4. 创建必要的数据文件
echo "检查数据文件..."
if [ ! -f "system_logs.db" ]; then
    touch system_logs.db
    chmod 666 system_logs.db
    echo "创建 system_logs.db"
else
    echo "system_logs.db 已存在，跳过创建"
fi

if [ ! -f "cursor.txt" ]; then
    touch cursor.txt
    chmod 666 cursor.txt
    echo "创建 cursor.txt"
else
    echo "cursor.txt 已存在，跳过创建"
fi

if [ ! -f "login.log" ]; then
    touch login.log
    echo "创建 login.log"
fi

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
