#!/bin/bash

# 修改后的脚本：通过命令行参数匹配Python进程
PROCESS_NAME="login.py"   # 监控的Python脚本名
INTERVAL=1               # 刷新间隔（秒）

# 获取所有运行 test.py 的Python进程PID
get_pids() {
    pgrep -f "python.*${PROCESS_NAME}"
}

# 核心监控命令
while true; do
    clear
    echo "🔄 监控中 (进程名包含: ${PROCESS_NAME})..."
    echo "--------------------------------------------------"
    
    # 获取进程信息（按CPU使用率排序）
    ps aux | grep -E "[p]ython.*${PROCESS_NAME}" | awk '{
        printf "PID: %-8s CPU: %-6s MEM: %-6s 启动时间: %s\n", 
        $2, $3, $4, $9
    }'
    
    sleep $INTERVAL
done
