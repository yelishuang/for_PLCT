import socket
import os
import re
import subprocess
import pam
from datetime import datetime, timedelta
import pytz
import time
import psutil
import systemd.journal
import sqlite3
import json
import threading
from flask import Flask, request, jsonify, session, Response
from flask_cors import CORS
from collections import deque


# 数据库配置
DB_NAME = "system_logs.db"
MAX_RECORDS = 800000  # 表记录上限


_cpu_stats_queue = deque(maxlen=2)


app = Flask(__name__)
app.secret_key = os.urandom(24)
app.config.update(
    SESSION_COOKIE_HTTPONLY=True,
    SESSION_COOKIE_SAMESITE='Lax',
    PERMANENT_SESSION_LIFETIME=3600,
    SESSION_REFRESH_EACH_REQUEST=True
)
CORS(app, supports_credentials=True)

# 系统监控核心代码
_thermal_zones = [f'/sys/class/thermal/{zone}/temp'
                  for zone in os.listdir('/sys/class/thermal/')
                  if zone.startswith('thermal_zone')]



def init_db():
    """初始化数据库表结构"""
    conn = sqlite3.connect(DB_NAME)
    cur = conn.cursor()
    
    cur.execute('''
        CREATE TABLE IF NOT EXISTS logs (
            id INTEGER PRIMARY KEY AUTOINCREMENT,
            timestamp INTEGER NOT NULL,
            priority INTEGER NOT NULL,
            identifier TEXT NOT NULL,
            message TEXT NOT NULL
        )
    ''')
    
    # 创建时间索引（加速时间范围查询）
    cur.execute('''
        CREATE INDEX IF NOT EXISTS idx_timestamp 
        ON logs (timestamp,priority,identifier)
    ''')
    
    conn.commit()
    conn.close()

def save_to_db(entries):
    """保存日志到数据库并维护表大小"""
    if not entries:
        return
    
    conn = sqlite3.connect(DB_NAME)
    cur = conn.cursor()
    
    try:
        # 批量插入数据
        cur.executemany('''
            INSERT INTO logs (timestamp, priority, identifier, message)
            VALUES (:timestamp, :priority, :identifier, :message)
        ''', [{
            "timestamp": entry["__REALTIME_TIMESTAMP"],
            "priority": entry["PRIORITY"],
            "identifier": entry["SYSLOG_IDENTIFIER"],
            "message": entry["MESSAGE"]
        } for entry in entries])
        
        # 维护表记录数量
        cur.execute("SELECT COUNT(*) FROM logs")
        count = cur.fetchone()[0]
        if count > MAX_RECORDS:
            # 计算需要删除的旧记录数量
            delete_count = count - MAX_RECORDS
            # 删除最早的记录
            cur.execute('''
                DELETE FROM logs 
                WHERE id IN (
                    SELECT id FROM logs 
                    ORDER BY timestamp ASC 
                    LIMIT ?
                )
            ''', (delete_count,))
            
        conn.commit()
        print(f"成功插入 {len(entries)} 条日志，删除 {delete_count if count > MAX_RECORDS else 0} 条旧记录")
    except Exception as e:
        print(f"数据库操作失败: {str(e)}")
        conn.rollback()
    finally:
        conn.close()

def get_systemd_log_entries(last_cursor=None):
    """获取系统日志条目，根据游标增量或全量获取"""
    j = systemd.journal.Reader()
    j.return_immediately = True  # 非阻塞模式

    try:
        if last_cursor:
            try:
                j.seek_cursor(last_cursor)
                entry = j.get_next()
                if entry and entry.get('__CURSOR') == last_cursor:
                    pass
                else:
                    raise ValueError("无效的游标")
            except Exception as e:
                print(f"游标错误: {e}, 回退到7天范围")
                last_cursor = None

        if not last_cursor:
            seven_days_ago = datetime.now(pytz.utc) - timedelta(days=7)
            j.seek_realtime(seven_days_ago)

        entries = []
        new_last_cursor = last_cursor

        while True:
            entry = j.get_next()
            if not entry:
                break
            log_entry = {
                "PRIORITY": entry.get("PRIORITY", 0),
                "SYSLOG_IDENTIFIER": entry.get("SYSLOG_IDENTIFIER", ""),
                "MESSAGE": entry.get("MESSAGE", ""),
                "__REALTIME_TIMESTAMP": entry.get("__REALTIME_TIMESTAMP", 0),
            }
            entries.append(log_entry)
            new_last_cursor = j._get_cursor()

        print(f"获取到 {len(entries)} 条日志")
        return entries, new_last_cursor

    finally:
        j.close()

def load_last_cursor():
    cursor_file = 'cursor.txt'
    if not os.path.exists(cursor_file):
        return None
    try:
        with open(cursor_file, 'r') as f:
            return f.read().strip()
    except FileNotFoundError:
        return None

def save_last_cursor(cursor):
    cursor_file = 'cursor.txt'
    with open(cursor_file, 'w') as f:
        f.write(cursor)

@app.route('/search', methods=['GET'])
def search_logs():
    try:
        params = {
            'start_time': request.args.get('start_time'),
            'end_time': request.args.get('end_time'),
            'priority': int(request.args.get('priority', 7)),
            'identifier': request.args.get('identifier'),
            'offset': int(request.args.get('offset', 0))
        }

        # 构建SQL查询
        conditions = [
            "timestamp BETWEEN ? AND ?",
            "priority = ?"
        ]
        query_params = [
            params['start_time'],
            params['end_time'],
            params['priority']
        ]

        if params['identifier']:
            print("params:" + params['identifier'])
            if params['identifier'].strip() == '':
                conditions.append("(identifier IS NULL OR identifier = '')")
            else:
                conditions.append("identifier = ?")
                query_params.append(params['identifier'])

        base_query = f"""
            SELECT 
                timestamp, priority, identifier, message 
            FROM logs 
            WHERE {' AND '.join(conditions)}
            ORDER BY timestamp DESC 
            LIMIT 51 
            OFFSET ?
        """
        query_params.append(params['offset'])

        # 执行查询
        conn = sqlite3.connect('system_logs.db')
        conn.row_factory = sqlite3.Row  # 方便调试
        cur = conn.cursor()
        
        cur.execute(base_query, query_params)
        rows = cur.fetchall()

        # 转换结果
        logs = [dict(row) for row in rows[:50]]
        for log in logs:
            log['timestamp'] = log['timestamp'][:19]  # 简化时间格式
            
        return jsonify({
            "logs": logs,
            "has_more": len(rows) > 50
        })

    except sqlite3.Error as e:
        return jsonify({"error": f"数据库错误: {str(e)}"}), 500
        
    except Exception as e:
        return jsonify({"error": f"系统异常: {str(e)}"}), 500


def log_sync_thread():
    """日志同步线程函数"""
    while True:
        print(f"\n{datetime.now()}: 开始日志同步...")
        entries, new_cursor = get_systemd_log_entries(load_last_cursor())
        if entries:
            save_to_db(entries)
            save_last_cursor(new_cursor)
        time.sleep(60)
    
@app.after_request
def add_cors_headers(response):
    response.headers['Access-Control-Allow-Origin'] = '*'
    return response


def get_hardware_info():
    def _safe_read(path):
        try:
            with open(path, 'r') as f:
                return f.read().strip() or "N/A"
        except Exception:
            return "N/A"

    return {
        "product_name": _safe_read('/sys/class/dmi/id/product_name'),
        "product_serial": _safe_read('/sys/class/dmi/id/product_serial'),
        "bios_version": _safe_read('/sys/class/dmi/id/bios_version')
    }


def get_cpu_arch():
    return os.uname().machine


def get_cpu_model():
    try:
        with open('/proc/cpuinfo', 'r') as f:
            for line in f:
                if line.startswith('model name'):
                    return re.split(r':\s+', line, 1)[1].strip()
        return "Unknown CPU"
    except Exception:
        return "Unknown CPU"


def get_kernel_version():
    return os.uname().release


def get_kernel_patches():
    try:
        result = subprocess.run(
            ['rpm', '-q', 'kernel', '--last'],
            capture_output=True,
            text=True,
            timeout=1
        )
        return result.stdout.split('\n')[0][:100]
    except Exception:
        return "N/A"


def get_last_login():
    try:
        result = subprocess.run(
            ['last', '-n', '1', '-i'],
            capture_output=True,
            text=True,
            timeout=2
        )
        return result.stdout.split('\n')[0][:80]
    except Exception:
        return "N/A"


def get_system_version():
    try:
        with open('/etc/os-release', 'r') as f:
            for line in f:
                if line.startswith('PRETTY_NAME'):
                    return line.split('=')[1].strip().strip('"')
    except Exception:
        return "未知系统主机"


def get_device_name():
    try:
        return socket.gethostname()
    except Exception:
        return "未知"


def is_port_in_use(port):
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        return s.connect_ex(('localhost', port)) == 0


def check_file_integrity():
    try:
        # 使用 fsck 检查文件系统完整性
        result = subprocess.run(['fsck', '-N'], capture_output=True, text=True, timeout=5)
        if result.returncode != 0:
            return "异常"
        return "正常"
    except Exception as e:
        return "异常"


def check_network_connectivity():
    try:
        # 使用 ping 命令检查环回接口的连通性
        result = subprocess.run(['ping', '-c', '1', '127.0.0.1'], capture_output=True, text=True, timeout=5)
        if result.returncode != 0:
            return "异常"
        return "正常"
    except Exception as e:
        return "异常"


def get_disk_usage():
    try:
        # 使用 df 命令获取磁盘分区使用率
        result = subprocess.run(['df', '-h'], capture_output=True, text=True, timeout=5)
        lines = result.stdout.split('\n')
        usage_info = {}
        for line in lines[1:]:
            parts = line.split()
            if len(parts) >= 5:
                filesystem = parts[0]
                usage_percent = parts[4]
                usage_info[filesystem] = usage_percent
        return usage_info
    except Exception as e:
        return {}


def get_uptime():
    try:
        # 使用 uptime 命令获取设备工作时间
        result = subprocess.run(['uptime', '-p'], capture_output=True, text=True, timeout=5)
        return result.stdout.strip()
    except Exception as e:
        return "N/A"


@app.route('/api/login', methods=['POST'])
def login():
    try:
        data = request.get_json()
        username = data.get('username')
        password = data.get('password')

        try:
            auth_result = pam.authenticate(username, password)
        except Exception as e:
            return jsonify({
                "success": False,
                "code": "AUTH_SYSTEM_ERROR",
                "message": "认证系统暂时不可用"
            }), 500

        if auth_result:
            session.clear()
            session['logged_in'] = True
            session.permanent = True
            return jsonify({
                "success": True,
                "code": "AUTH_SUCCESS",
                "user": username,
                "redirect": "/",
                "message": "认证成功"
            }), 200
        else:
            return jsonify({
                "success": False,
                "code": "INVALID_CREDENTIALS",
                "message": "用户名或密码无效"
            }), 401

    except Exception as e:
        return jsonify({
            "success": False,
            "code": "INTERNAL_ERROR",
            "message": "系统服务暂时不可用"
        }), 500


@app.route('/api/check-auth', methods=['GET'])
def check_auth():
    if session.get('logged_in'):
        return jsonify({
            "authenticated": True,
        }), 200
    return jsonify({"authenticated": False}), 401


@app.route('/api/overview', methods=['GET'])
def system_overview():
    if not session.get('logged_in'):
        return jsonify({"error": "未认证"}), 401

    try:
        hardware = get_hardware_info()
        return jsonify({
            "kernel-version": f"{get_kernel_version()} | {get_kernel_patches()}",
            "login-history": get_last_login(),
            "device-info": f"{hardware['product_name']} | SN:{hardware['product_serial']}",
            "cpu-info": f"{get_cpu_model()} ({get_cpu_arch()})",
            "firmware-info": f"BIOS:{hardware['bios_version']}"
        }), 200
    except Exception as e:
        return jsonify({
            "code": "OVERVIEW_ERROR",
            "message": "获取系统信息失败"
        }), 500


@app.route('/api/system-info', methods=['GET'])
def get_system_info():
    system_version = get_system_version()
    device_name = get_device_name()
    system_info = {
        "system_version": system_version,
        "device_name": device_name
    }
    return jsonify(system_info)


@app.route('/api/pollingdata', methods=['GET'])
def get_polling_data():
    if not session.get('logged_in'):
        return jsonify({"error": "未认证"}), 401

    try:
        file_integrity = check_file_integrity()
        network_status = check_network_connectivity()
        disk_usage = get_disk_usage()
        uptime = get_uptime()

        return jsonify({
            "file": file_integrity,
            "net": network_status,
            "disk": disk_usage,
            "uptime": uptime
        }), 200
    except Exception as e:
        return jsonify({
            "code": "POLLING_DATA_ERROR",
            "message": "获取轮询数据失败"
        }), 500

def _read_cpu_usage():
    with open('/proc/stat') as f:
        line = f.readline().strip()
    parts = line.split()[1:]
    total = sum(map(int, parts))
    idle = int(parts[3]) + int(parts[4])
    return total, idle


def _get_cpu_percent():
    current = _read_cpu_usage()
    _cpu_stats_queue.append(current)

    if len(_cpu_stats_queue) < 2:
        return 0.0

    prev_total, prev_idle = _cpu_stats_queue[0]
    curr_total, curr_idle = _cpu_stats_queue[1]

    total_diff = curr_total - prev_total
    idle_diff = curr_idle - prev_idle

    return 100.0 * (total_diff - idle_diff) / total_diff if total_diff else 0.0


def _get_memory_percent():
    mem_info = {}
    with open('/proc/meminfo') as f:
        for line in f:
            key = line.split(':')[0]
            value = line.split(':')[1].strip().split()[0]
            mem_info[key] = int(value) * 1024

    available = mem_info.get('MemAvailable', mem_info['MemFree'])
    used = mem_info['MemTotal'] - available
    return 100.0 * used / mem_info['MemTotal']


def _get_load_avg():
    with open('/proc/loadavg') as f:
        return float(f.read().split()[0])


def _get_temperature():
    temps = []
    for path in _thermal_zones:
        try:
            with open(path, 'r') as f:
                temp = int(f.read()) / 1000.0
                temps.append(temp)
        except Exception:
            continue
    return sum(temps) / len(temps) if temps else None

def get_top_processes():
    # Get a list of all running processes
    processes = []
    for proc in psutil.process_iter(['pid', 'status', 'cpu_percent', 'name']):
        try:
            pinfo = proc.info
            processes.append(pinfo)
        except (psutil.NoSuchProcess, psutil.AccessDenied, psutil.ZombieProcess):
            pass

    # Sort processes by CPU usage
    sorted_processes = sorted(processes, key=lambda x: x['cpu_percent'], reverse=True)

    return sorted_processes[:5]

def get_system_metrics():
    top_processes = get_top_processes()
    return {
        'cpu_usage': round(_get_cpu_percent(), 1),
        'memory_usage': round(_get_memory_percent(), 1),
        'temperature': round(_get_temperature(), 1) if _get_temperature() else None,
        'load_avg': round(_get_load_avg(), 2),
        'top_processes': top_processes
    }


# SSE 路由
@app.route('/metrics/stream')
def sse_stream():
    def event_stream():
        last_top_update = 0
        cached_top_processes = []
        try:
            while True:
                current_time = time.time()
                
                # 每3秒更新一次top_processes
                if current_time - last_top_update >= 3:
                    cached_top_processes = get_top_processes()
                    last_top_update = current_time
                
                # 实时获取其他指标
                metrics = {
                    'cpu_usage': round(_get_cpu_percent(), 1),
                    'memory_usage': round(_get_memory_percent(), 1),
                    'temperature': round(_get_temperature(), 1) if _get_temperature() else None,
                    'load_avg': round(_get_load_avg(), 2),
                    'top_processes': cached_top_processes
                }
                
                data = f"data: {json.dumps(metrics)}\n\n"
                yield data
                time.sleep(1)
        except GeneratorExit:
            print("Client disconnected")

    return Response(event_stream(), mimetype="text/event-stream")


@app.route('/api/disk_info', methods=['GET'])
def get_disk_info():
    """获取所有磁盘分区的详细信息"""
    disks = []
    for part in psutil.disk_partitions():
        if 'cdrom' in part.opts or not part.device:
            continue
        try:
            usage = psutil.disk_usage(part.mountpoint)
            disks.append({
                "device": part.device,
                "mountpoint": part.mountpoint,
                "fstype": part.fstype,
                "total": f"{usage.total // (1024**3)}GB",
                "used": f"{usage.used // (1024**3)}GB",
                "free": f"{usage.free // (1024**3)}GB",
                "percent": f"{usage.percent}%",
                "flags": part.opts.split(',')
            })
        except (PermissionError, FileNotFoundError):
            continue
    return jsonify(disks)

def get_disk_io():
    # 获取所有物理磁盘的独立 IO 统计（排除虚拟设备）
    io_counters = psutil.disk_io_counters(perdisk=True)
    total_read = 0
    total_write = 0
    for disk, counters in io_counters.items():
        # 过滤虚拟设备（根据系统不同可能需要调整）
        if not disk.startswith(('loop', 'sr', 'md', 'dm-')):
            total_read += counters.read_bytes
            total_write += counters.write_bytes
    return total_read, total_write

def monitor_disk_io():
    prev_read, prev_write = get_disk_io()
    while True:
        time.sleep(1)
        current_read, current_write = get_disk_io()
        
        read_speed = current_read - prev_read
        write_speed = current_write - prev_write
        
        # 正确写法：将字典构造与字符串分离
        data = {
            'read_speed': int(read_speed),
            'write_speed': int(write_speed)
        }
        yield f"data: {json.dumps(data)}\n\n"  # 单行字符串
        
        prev_read, prev_write = current_read, current_write
        
@app.route('/stream/disk')
def stream():
    return Response(monitor_disk_io(), mimetype='text/event-stream')

def get_network_io():
    # 获取所有网络接口的 I/O 统计
    io_counters = psutil.net_io_counters(pernic=True)
    total_sent = 0
    total_recv = 0
    for interface, counters in io_counters.items():
        # 过滤虚拟设备（根据系统不同可能需要调整）
        if not interface.startswith(('lo', 'veth', 'docker')):
            total_sent += counters.bytes_sent
            total_recv += counters.bytes_recv
    return total_sent, total_recv

def monitor_network_io():
    prev_sent, prev_recv = get_network_io()
    while True:
        time.sleep(1)
        current_sent, current_recv = get_network_io()
        
        sent_speed = current_sent - prev_sent
        recv_speed = current_recv - prev_recv
        
        # 正确写法：将字典构造与字符串分离
        data = {
            'sent_speed': int(sent_speed),
            'recv_speed': int(recv_speed)
        }
        yield f"data: {json.dumps(data)}\n\n"  # 单行字符串
        
        prev_sent, prev_recv = current_sent, current_recv

@app.route('/api/network_info', methods=['GET'])
def get_network_info():
    """获取所有网络设备的详细信息"""
    net_interfaces = psutil.net_if_addrs()
    network_info = []

    for interface_name, addresses in net_interfaces.items():
        interface_data = {
            "interface": interface_name,
            "addresses": []
        }
        for address in addresses:
            addr_info = {
                "family": str(address.family),
                "address": address.address,
                "netmask": address.netmask,
                "broadcast": address.broadcast,
                "ptp": address.ptp
            }
            interface_data["addresses"].append(addr_info)
        network_info.append(interface_data)

    return jsonify(network_info)

@app.route('/stream/network')
def stream_network():
    return Response(monitor_network_io(), mimetype='text/event-stream')

if __name__ == '__main__':
    init_db()
    # 启动日志同步线程
    sync_thread = threading.Thread(target=log_sync_thread, daemon=True)
    sync_thread.start()

    http_port = 8000
    if is_port_in_use(http_port):
        print(f"端口 {http_port} 已被占用，请释放端口后再运行。")
    else:
        app.run(host='0.0.0.0', port=http_port, threaded=True)

