// 认证状态管理
async function checkAuth() {
    try {
        const response = await fetch('/api/check-auth', {
            credentials: 'include'
        });

        if (!response.ok || !(await response.json()).authenticated) {
            window.location.replace('/login');
        }
    } catch (error) {
        console.error('认证检查失败:', error);
        window.location.replace('/login');
    }
}

// 初始化系统参数页
function initNavigation() {
    loadContent("overview.html", document.querySelector('.submenu-item:first-child'));
}

// 侧边栏点击对应标签 主页面加载对应内容
async function loadContent(page, clickedItem) {
    try {
        const allMenuItems = document.querySelectorAll('.submenu-item');
        allMenuItems.forEach(item => {
            item.classList.remove('active');
        });
        if (clickedItem) {
            clickedItem.classList.add('active');
        }

        // 停止所有数据更新
        stopDataUpdates();
        stopStorage();
        stopNet();

        const response = await fetch(page);
        if (!response.ok) throw new Error(`Error loading ${page}: ${response.statusText}`);
        const text = await response.text();
        document.querySelector('.content-area').innerHTML = text;

        // 启动数据更新（仅在 overview 页面）
        if (page === 'overview.html') {
            startDataUpdates();
        }
        else if(page === 'storage.html')
         {
            loadStorage();
        }
        else if(page === 'network.html')
         {
            loadNetInfo();
        }
    } catch (error) {
        console.error(error);
        document.querySelector('.content-area').innerHTML = '<p>无法加载页面内容。</p>';
    }
}

// 内容更新
async function updatePageContent() {
    const pageElement = document.getElementById("overview");
    try {
        const response = await fetch('/api/overview', {
            credentials: 'include'
        });
        const data = await response.json();
        Object.entries(data).forEach(([key, value]) => {
                pageElement.querySelectorAll(`[data-field="${key}"]`).forEach(el => {
                el.textContent = value || 'N/A';
                el.classList.add('data-loaded');
            });
        });
    } catch (error) {
        showGlobalError(`数据加载失败: ${error.message.replace('pam_', '')}`);
    }
}

// 设备名称编辑
function initDeviceEditor() {
    document.querySelector('.btn-edit').addEventListener('click', (e) => {
        const span = e.target;
        const newName = prompt('编辑设备名称', span.textContent);
        if (newName && newName !== span.textContent) {
            span.textContent = newName;
        }
    });
}

// 实时时间显示
function initClock() {
    function update() {
        document.getElementById('real-time').textContent =
            new Date().toLocaleString('zh-CN', {
                year: 'numeric',
                month: '2-digit',
                day: '2-digit',
                hour: '2-digit',
                minute: '2-digit',
                second: '2-digit',
                hour12: false
            }).replace(/\//g, '-');
    }
    setInterval(update, 1000);
    update();
}

// 主初始化
document.addEventListener('DOMContentLoaded', async () => {
    await checkAuth();
    initClock();
    initDeviceEditor();
    initNavigation();
});

// 数据更新相关变量
let pollingIntervalId = null;
let eventSource = null;

// 启动数据更新
function startDataUpdates() {

    updatePageContent()
    // 启动轮询数据
    pollingIntervalId = setInterval(fetchPollingData, 50000);
    fetchPollingData();

    // 启动 SSE 更新
    initSSE();
}

// 停止数据更新
function stopDataUpdates() {
    // 清除轮询数据定时器
    if (pollingIntervalId) {
        clearInterval(pollingIntervalId);
        pollingIntervalId = null;
    }

    // 关闭 SSE 连接
    if (eventSource) {
        eventSource.close();
        eventSource = null;
    }
}

//定时轮询50s轮询一次
async function fetchPollingData() {
    try {
        const response = await fetch('/api/pollingdata', {
            credentials: 'include'
        });
        if (!response.ok) throw new Error(`HTTP错误: ${response.status}`);
        const data = await response.json();
        updateUI(data);

    } catch (error) {
        console.error('获取数据失败:', error.message);
    }
}

// 数据更新UI函数（低频静态数据）
function updateUI(data) {
    const fieldMap = {
        file: 'file-integrity',
        net: 'Network-Links',
        disk: 'Partition-Usage',
        uptime: 'uptime'
    };

    Object.entries(fieldMap).forEach(([dataKey, domKey]) => {
        let value = data[dataKey];

        // 处理 uptime 时间戳（精确到分钟，格式：X天X小时X分钟 或 X小时X分钟 或 X分钟）
        if (dataKey === 'uptime' && !isNaN(value)) {
            const seconds = Math.floor(value);
            const days = Math.floor(seconds / 86400);       // 24*3600秒=1天
            const hours = Math.floor((seconds % 86400) / 3600); // 剩余小时数
            const minutes = Math.floor((seconds % 3600) / 60); // 剩余分钟数

            // 拼接友好时间字符串
            value = [];
            if (days > 0) value.push(`${days}天`);
            if (days > 0 || hours > 0) value.push(`${hours}小时`);
            if (minutes > 0 || value.length === 0) value.push(`${minutes}分钟`);
            value = value.join('');
        } else if (dataKey === 'disk' && typeof value === 'object') {
            // 动态处理 disk 字典
            const entries = Object.entries(value);
            if (entries.length > 0) {
                value = entries.map(([key, val]) => `${key}: ${val}`).join(', ');
            } else {
                value = 'N/A';
            }
        } else {
            value = value !== undefined ? value : 'N/A';
        }

        // 写入DOM
        if (domKey === 'uptime') {
            document.getElementById(domKey).textContent = value || 'N/A';
        } else {
            document.querySelectorAll(`[data-field="${domKey}"]`).forEach(el => {
                el.textContent = value || 'N/A';
            });
        }
    });
}

// SSE更新动态数据
function initSSE() {
    eventSource = new EventSource('/metrics/stream');

    eventSource.onmessage = function (event) {
        try {
            const metrics = JSON.parse(event.data);
            updateCPUUsage(metrics);
            updateMemoryUsage(metrics);
            updateTemperature(metrics);
            updateLoadAverage(metrics);
            updateTopProcesses(metrics);
        } catch (error) {
            console.error('解析消息时出错:', error);
        }
    };

    eventSource.onerror = function (error) {
        console.error('SSE发生错误:', error);
    };
}

function updateCPUUsage(metrics) {
    const cpuUsageElement = document.getElementById('cpu-usage');
    if (cpuUsageElement) {
        cpuUsageElement.textContent = `${metrics.cpu_usage}%`;
    }
}

function updateMemoryUsage(metrics) {
    const memoryUsageElement = document.getElementById('memory-usage');
    if (memoryUsageElement) {
        memoryUsageElement.textContent = `${metrics.memory_usage}%`;
    }
}

function updateTemperature(metrics) {
    const temperatureElement = document.getElementById('temperature');
    if (temperatureElement) {
        temperatureElement.textContent = `${metrics.temperature}°C`;
    }
}

function updateLoadAverage(metrics) {
    const loadAverageElement = document.getElementById('load-average');
    if (loadAverageElement) {
        loadAverageElement.textContent = metrics.load_avg;
    }
}

function updateTopProcesses(metrics) {
    const topProcessesElement = document.getElementById('top-processes');
    if (topProcessesElement) {
        topProcessesElement.innerHTML = '';
        metrics.top_processes.forEach((line) => {
            const processItem = document.createElement('p');
            processItem.textContent = `PID: ${line.pid}, Name: ${line.name}, Status: ${line.status}, CPU%: ${line.cpu_percent}`;
            topProcessesElement.appendChild(processItem);
        });
    }
}


let currentOffset = 0;
let currentParams = {};

// 时间范围计算（前端处理）
function calcTimeRange(option) {
    const now = new Date();
    const ranges = {
        '24h': [
            new Date(now.getTime() - 86400000),  // 24小时前
            now
        ],
        '3d': [
            new Date(now.getTime() - 259200000), // 3天前
            now
        ],
        '7d': [
            new Date(now.getTime() - 604800000), // 7天前
            now
        ],
        'all': [
            new Date(0),  // 时间起点（1970-01-01）
            now
        ]
    };
    
    return ranges[option].map(d => 
        d.toISOString().slice(0, 19).replace('T', ' ')
    );
}

// 参数预处理
function prepareParams() {
    const timeRange = document.getElementById('timeRange').value;
    const [start, end] = calcTimeRange(timeRange);
    
    return {
        start_time: start,
        end_time: end,
        priority: document.getElementById('logLevel').value,
        identifier: document.getElementById('identifier').value.trim()
    };
}

// 执行查询
async function searchLogs() {
    currentParams = prepareParams();
    currentOffset = 0;
    await fetchData(true);
}

// 数据获取
async function fetchData(clear) {
    const params = new URLSearchParams({
        ...currentParams,
        offset: currentOffset
    });

    const res = await fetch(`/search?${params}`); 
    const data = await res.json();

    renderResults(data.logs, clear);
    document.getElementById('loadMore').style.display = 
        data.has_more ? 'block' : 'none';
}

function convertLogLevel(level) {
    const logLevels = ["EMERG", "ALERT", "CRIT", "ERR", "WARNING", "NOTICE", "INFO", "DEBUG"];
    return logLevels[Number(level)] || "UNKNOWN";
}

// 渲染结果
function renderResults(logs, clear) {
    const container = document.getElementById('results');
    if (clear) container.innerHTML = '';

    logs.forEach(log => {
        const div = document.createElement('div');
        div.className = 'log-item';
        div.innerHTML = `
            <div class="log-time">${log.timestamp}</div>
            <div class="log-header">
                <span class="log-level">日志级别：${convertLogLevel(log.priority)}</span>
                <span class="log-identifier">${log.identifier || ''}</span>
            </div>
            <div>${log.message}</div>
        `;
        container.appendChild(div);
    });
}

// 加载更多
async function loadMore() {
    currentOffset += 50;
    await fetchData(false);
}


let readChart = null;
let writeChart = null;

class NativeChart {
    constructor(canvasId, color) {
        this.canvas = document.getElementById(canvasId);
        this.ctx = this.canvas.getContext('2d');
        this.color = color;
        this.data = [];
        this.maxPoints = 30;
    }

    addData(value) {
        this.data.push(value);
        if (this.data.length > this.maxPoints) {
            this.data.shift();
        }
        this.draw();
    }

    draw() {
        const ctx = this.ctx;
        const width = this.canvas.width;
        const height = this.canvas.height;

        // 清空画布
        ctx.clearRect(0, 0, width, height);

        // 绘制网格
        ctx.strokeStyle = '#eee';

        for (let y = height - 20; y > 0; y -= 40) {
            ctx.beginPath();
            ctx.moveTo(0, y);
            ctx.lineTo(width, y);
            ctx.stroke();
        }

        // 绘制折线
        ctx.beginPath();
        ctx.strokeStyle = this.color;
        ctx.lineWidth = 2;

        const maxValue = Math.max(...this.data) || 1;
        const stepX = width / (this.maxPoints - 1);

        this.data.forEach((val, index) => {
            const x = index * stepX;
            const y = height - 20 - (val / maxValue) * (height - 40);

            if (index === 0) {
                ctx.moveTo(x, y);
            } else {
                ctx.lineTo(x, y);
            }
        });

        ctx.stroke();
    }
}

function convertSpeed(speedBytes) {
    const units = ['B/s', 'KB/s', 'MB/s', 'GB/s'];
    let unitIndex = 0;
    let speed = speedBytes;

    while (speed >= 1024 && unitIndex < units.length - 1) {
        speed /= 1024;
        unitIndex++;
    }

    return {
        speed: speed.toFixed(speed < 10 ? 2 : 1),
        unit: units[unitIndex]
    };
}

async function fetchDiskInfo() {
    try {
        const response = await fetch('/api/disk_info');
        const data = await response.json();
        const tbody = document.getElementById('diskInfoBody');

        tbody.innerHTML = data.map(disk => `
            <tr>
                <td>${disk.device}</td>
                <td>${disk.mountpoint}</td>
                <td>${disk.fstype}</td>
                <td>${disk.total}</td>
                <td>${disk.used}</td>
                <td>${disk.free}</td>
                <td>${disk.percent}</td>
            </tr>
        `).join('');
    } catch (error) {
        console.error('获取磁盘信息失败:', error);
    }
}

function startIOStream() {
    eventSource = new EventSource('/stream/disk');

    eventSource.onmessage = (event) => {
        const data = JSON.parse(event.data);

        // 处理读取速度
        const read = convertSpeed(data.read_speed);
        document.getElementById('readSpeed').textContent = read.speed;
        document.getElementById('readUnit').textContent = read.unit;
        readChart.addData(parseFloat(read.speed));

        // 处理写入速度
        const write = convertSpeed(data.write_speed);
        document.getElementById('writeSpeed').textContent = write.speed;
        document.getElementById('writeUnit').textContent = write.unit;
        writeChart.addData(parseFloat(write.speed));
    };

    eventSource.onerror = (error) => {
        console.error('SSE连接错误:', error);
        eventSource.close();
        eventSource = null;
    };
}

function loadStorage() {
    readChart = new NativeChart('readChart', '#4a90e2');
    writeChart = new NativeChart('writeChart', '#ff6b6b');

    fetchDiskInfo();
    setInterval(fetchDiskInfo, 30000);
    startIOStream();
}

function stopStorage() {
    if (eventSource) {
        eventSource.close();
        eventSource = null;
    }
}

class NetworkChart {
    constructor(canvasId, color) {
        this.canvas = document.getElementById(canvasId);
        this.ctx = this.canvas.getContext('2d');
        this.color = color;
        this.data = [];
        this.maxPoints = 30;
    }

    addData(value) {
        this.data.push(value);
        if (this.data.length > this.maxPoints) {
            this.data.shift();
        }
        this.draw();
    }

    draw() {
        const ctx = this.ctx;
        const width = this.canvas.width;
        const height = this.canvas.height;

        // 清空画布
        ctx.clearRect(0, 0, width, height);

        // 绘制网格横线
        ctx.strokeStyle = '#eee';
        const numTicks = 5;
        for (let i = 0; i <= numTicks; i++) {
            const y = height - 20 - (i / numTicks) * (height - 40);
            ctx.beginPath();
            ctx.moveTo(0, y);
            ctx.lineTo(width, y);
            ctx.stroke();
        }

        // 绘制折线
        ctx.beginPath();
        ctx.strokeStyle = this.color;
        ctx.lineWidth = 2;

        const stepX = width / (this.maxPoints - 1);

        this.data.forEach((val, index) => {
            const x = index * stepX;
            const y = height - 20 - (val / Math.max(...this.data)) * (height - 40);

            if (index === 0) {
                ctx.moveTo(x, y);
            } else {
                ctx.lineTo(x, y);
            }
        });

        ctx.stroke();
    }
}


// 自动单位转换
function convertNetworkSpeed(speedBytes) {
    const units = ['B/s', 'KB/s', 'MB/s', 'GB/s'];
    let unitIndex = 0;
    let speed = speedBytes;

    while (speed >= 1024 && unitIndex < units.length - 1) {
        speed /= 1024;
        unitIndex++;
    }

    return {
        speed: speed.toFixed(speed < 10 ? 2 : 1),
        unit: units[unitIndex]
    };
}

// 获取网络接口信息
async function fetchNetworkInterfaces() {
    try {
        const response = await fetch('/api/network_info');
        const data = await response.json();
        const tbody = document.getElementById('networkInterfaceBody');

        tbody.innerHTML = data.map(interfaceInfo => `
            ${interfaceInfo.addresses.map(addr => `
                <tr>
                    <td>${interfaceInfo.interface}</td>
                    <td>${addr.family}</td>
                    <td>${addr.address}</td>
                    <td>${addr.netmask || '-'}</td>
                    <td>${addr.broadcast || '-'}</td>
                    <td>${addr.ptp || '-'}</td>
                </tr>
            `).join('')}
        `).join('');
    } catch (error) {
        console.error('获取网络接口信息失败:', error);
    }
}

// 实时数据监听
function startNetworkStream() {
    const eventSource = new EventSource('/stream/network');

    eventSource.onmessage = (event) => {
        const data = JSON.parse(event.data);

        // 处理发送速度
        const sent = convertNetworkSpeed(data.sent_speed);
        document.getElementById('networkSentSpeed').textContent = sent.speed;
        document.getElementById('networkSentUnit').textContent = sent.unit;
        networkSentChart.addData(parseFloat(sent.speed));

        // 处理接收速度
        const recv = convertNetworkSpeed(data.recv_speed);
        document.getElementById('networkRecvSpeed').textContent = recv.speed;
        document.getElementById('networkRecvUnit').textContent = recv.unit;
        networkRecvChart.addData(parseFloat(recv.speed));
    };

    eventSource.onerror = (error) => {
        console.error('SSE连接错误:', error);
        eventSource.close();
    };
}


function loadNetInfo() {
    networkSentChart = new NetworkChart('networkSentChart', '#4a90e2');
    networkRecvChart = new NetworkChart('networkRecvChart', '#ff6b6b');

    fetchNetworkInterfaces();
    setInterval(fetchNetworkInterfaces, 30000);
    startNetworkStream();
}

function stopNet() {
    if (eventSource) {
        eventSource.close();
        eventSource = null;
    }
}


