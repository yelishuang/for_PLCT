# -*- coding: utf-8 -*-


import os
import re
import json
import time
import socket
import sqlite3
import threading
import subprocess
from collections import deque
from datetime import datetime, timedelta, timezone

import pam
import psutil
import systemd.journal
from flask import Flask, request, jsonify, session, Response
from flask_cors import CORS


# =============================
# Flask 基础配置
# =============================

app = Flask(__name__)
app.secret_key = os.urandom(24)
app.config.update(
    SESSION_COOKIE_HTTPONLY=True,
    SESSION_COOKIE_SAMESITE='Lax',
    PERMANENT_SESSION_LIFETIME=3600,
    SESSION_REFRESH_EACH_REQUEST=True
)
CORS(app, supports_credentials=True)

_cpu_stats_queue = deque(maxlen=2)

# 预读 thermal zones（仅系统信息用）
_thermal_zones = [f'/sys/class/thermal/{z}/temp'
                  for z in os.listdir('/sys/class/thermal/')
                  if z.startswith('thermal_zone')]


# =============================
# 新版日志服务（logs-v2） —— 模块化类
# =============================

class LogServiceV2:
    """
    模块化日志服务：
    - init_db()：初始化表结构与索引
    - start_background_sync()：后台线程增量同步 systemd journal → SQLite
    - http_search()：/search 接口处理（分页检索）
    - http_sse_stream()：/api/logs/stream 接口（SSE 实时流，直接 follow journal）
    约定：
    - 数据库存储时间为 epoch 秒（int）
    - 对外返回 timestamp 为 'YYYY-MM-DD HH:MM:SS'
    - 每页固定 50 条，通过 offset 翻页（0, 50, 100, ...）
    """

    def __init__(self,
                 app: Flask,
                 db_path: str = "system_logs.db",
                 cursor_path: str = "cursor.txt",
                 page_size: int = 50,
                 max_records: int = 800000,
                 sync_interval_sec: int = 15,
                 journal_lookback_days: int = 7):
        self.app = app
        self.db_path = db_path
        self.cursor_path = cursor_path
        self.page_size = page_size
        self.max_records = max_records
        self.sync_interval_sec = sync_interval_sec
        self.journal_lookback_days = journal_lookback_days
        self._stop_flag = threading.Event()

        self.init_db()
        self._register_routes()

    # ---------- 工具 ----------

    @staticmethod
    def _normalize_epoch_seconds(ts_any) -> int:
        """
        将 systemd 的 __REALTIME_TIMESTAMP（微秒）或其他整数统一转换为 '秒' 的 int。
        - 如果 ts>=1e12 认为是微秒，做 //1_000_000
        - 如果 ts>=1e10 认为是毫秒，做 //1_000
        - 否则认为已经是秒
        """
        try:
            ts = int(ts_any)
        except Exception:
            return 0
        if ts >= 1_000_000_000_000:      # microseconds
            return ts // 1_000_000
        elif ts >= 10_000_000_000:       # milliseconds
            return ts // 1_000
        else:
            return ts

    @staticmethod
    def _fmt_ts(epoch_sec: int) -> str:
        return datetime.fromtimestamp(int(epoch_sec), tz=timezone.utc).astimezone().strftime('%Y-%m-%d %H:%M:%S')

    @staticmethod
    def _parse_ts_str(ts_str: str) -> int:
        """
        输入 'YYYY-MM-DD HH:MM:SS'（本地时区）→ 转换为 epoch 秒
        """
        if not ts_str:
            return 0
        dt_local = datetime.strptime(ts_str, '%Y-%m-%d %H:%M:%S')
        # 认为输入是本地时间，转本地 tz 再转 UTC
        dt_local = dt_local.replace(tzinfo=datetime.now().astimezone().tzinfo)
        return int(dt_local.timestamp())

    def _read_cursor(self) -> str | None:
        if not os.path.exists(self.cursor_path):
            return None
        try:
            with open(self.cursor_path, 'r') as f:
                return f.read().strip() or None
        except Exception:
            return None

    def _save_cursor(self, cursor: str) -> None:
        try:
            with open(self.cursor_path, 'w') as f:
                f.write(cursor or '')
        except Exception as e:
            print(f"[LogServiceV2] 保存 cursor 失败: {e}")

    # ---------- 数据库 ----------

    def init_db(self):
        conn = sqlite3.connect(self.db_path)
        cur = conn.cursor()
        # 存 epoch 秒，priority 0-7，identifier/message 文本
        cur.execute("""
            CREATE TABLE IF NOT EXISTS logs (
                id INTEGER PRIMARY KEY AUTOINCREMENT,
                timestamp INTEGER NOT NULL,
                priority  INTEGER NOT NULL,
                identifier TEXT NOT NULL,
                message    TEXT NOT NULL
            )
        """)
        cur.execute("""
            CREATE INDEX IF NOT EXISTS idx_logs_time_prio_ident
            ON logs (timestamp, priority, identifier)
        """)
        conn.commit()
        conn.close()

    def _trim_table_if_needed(self, cur):
        cur.execute("SELECT COUNT(*) FROM logs")
        count = cur.fetchone()[0]
        if count > self.max_records:
            delete_count = count - self.max_records
            cur.execute("""
                DELETE FROM logs
                 WHERE id IN (SELECT id FROM logs ORDER BY timestamp ASC LIMIT ?)
            """, (delete_count,))
            return delete_count
        return 0

    def _bulk_insert(self, entries: list[dict]) -> int:
        if not entries:
            return 0
        conn = sqlite3.connect(self.db_path)
        cur = conn.cursor()
        try:
            cur.executemany("""
                INSERT INTO logs (timestamp, priority, identifier, message)
                VALUES (?, ?, ?, ?)
            """, [
                (
                    self._normalize_epoch_seconds(e.get('__REALTIME_TIMESTAMP', 0)),
                    int(e.get('PRIORITY', 0)),
                    str(e.get('SYSLOG_IDENTIFIER', '')),
                    str(e.get('MESSAGE', '')),
                )
                for e in entries
            ])
            deleted = self._trim_table_if_needed(cur)
            conn.commit()
            return len(entries)
        except Exception as e:
            conn.rollback()
            print(f"[LogServiceV2] 批量写库失败: {e}")
            return 0
        finally:
            conn.close()

    # ---------- Journal 增量同步线程 ----------

    def _pull_from_journal(self, last_cursor: str | None) -> tuple[list[dict], str | None]:
        """
        从 systemd journal 拉取（从游标后开始；无游标则从 lookback 天前开始）
        """
        j = systemd.journal.Reader()
        j.this_boot()          # 仅本次启动的日志，按需可移除
        j.log_level(systemd.journal.LOG_INFO)
        j.lazy_seek = True

        new_cursor = last_cursor
        entries = []

        try:
            if last_cursor:
                try:
                    j.seek_cursor(last_cursor)
                    # get_next() 会返回游标所在条目的“下一条”
                    _ = j.get_next()
                except Exception as e:
                    print(f"[LogServiceV2] 游标无效，回退：{e}")
                    last_cursor = None

            if not last_cursor:
                since = datetime.now(timezone.utc) - timedelta(days=self.journal_lookback_days)
                j.seek_realtime(since)

            while True:
                rec = j.get_next()
                if not rec:
                    break
                entries.append({
                    "PRIORITY": rec.get("PRIORITY", 0),
                    "SYSLOG_IDENTIFIER": rec.get("SYSLOG_IDENTIFIER", ""),
                    "MESSAGE": rec.get("MESSAGE", ""),
                    "__REALTIME_TIMESTAMP": rec.get("__REALTIME_TIMESTAMP", 0),
                })
                new_cursor = j._get_cursor()

            return entries, new_cursor
        finally:
            j.close()

    def _sync_loop(self):
        """
        周期性把 journal 新增条目刷进 SQLite
        """
        print("[LogServiceV2] 后台同步线程启动")
        while not self._stop_flag.is_set():
            try:
                last = self._read_cursor()
                batch, cur = self._pull_from_journal(last)
                if batch:
                    wrote = self._bulk_insert(batch)
                    if cur:
                        self._save_cursor(cur)
                    print(f"[LogServiceV2] 入库 {wrote} 条（cursor updated）")
            except Exception as e:
                print(f"[LogServiceV2] 同步异常: {e}")
            finally:
                # 小间隔提高新鲜度；与 SSE 不冲突（SSE 直接 follow）
                time.sleep(self.sync_interval_sec)

    def start_background_sync(self):
        t = threading.Thread(target=self._sync_loop, daemon=True)
        t.start()

    # ---------- HTTP: /search ----------

    def http_search(self):
        """
        GET /search?start_time=YYYY-MM-DD HH:MM:SS&end_time=...&priority=0..7&identifier=...&offset=0,50,100
        返回：
        {
          "logs":[{"timestamp":"YYYY-MM-DD HH:MM:SS","priority":4,"identifier":"nginx","message":"..."}],
          "has_more": true,
          "total": 1234   # 可选
        }
        """
        try:
            start_time = request.args.get('start_time', '')
            end_time = request.args.get('end_time', '')
            priority = int(request.args.get('priority', 7))
            identifier = request.args.get('identifier', '')
            offset = int(request.args.get('offset', 0))

            ts_start = self._parse_ts_str(start_time)
            ts_end = self._parse_ts_str(end_time)
            if ts_start and ts_end and ts_end < ts_start:
                ts_start, ts_end = ts_end, ts_start

            conditions = ["timestamp BETWEEN ? AND ?", "priority = ?"]
            qparams = [ts_start, ts_end, priority]

            if identifier:
                if identifier.strip() == '':
                    conditions.append("(identifier IS NULL OR identifier = '')")
                else:
                    conditions.append("identifier = ?")
                    qparams.append(identifier)

            base_sql = f"""
                SELECT timestamp, priority, identifier, message
                  FROM logs
                 WHERE {' AND '.join(conditions)}
                 ORDER BY timestamp DESC
                 LIMIT {self.page_size + 1}
                OFFSET ?
            """
            qparams.append(offset)

            conn = sqlite3.connect(self.db_path)
            conn.row_factory = sqlite3.Row
            cur = conn.cursor()

            cur.execute(base_sql, qparams)
            rows = cur.fetchall()
            conn.close()

            items = [dict(r) for r in rows[:self.page_size]]
            for it in items:
                it['timestamp'] = self._fmt_ts(it['timestamp'])

            return jsonify({
                "logs": items,
                "has_more": len(rows) > self.page_size
            })
        except Exception as e:
            return jsonify({"error": f"SEARCH_ERROR: {e}"}), 500

    # ---------- HTTP: /api/logs/stream (SSE) ----------

    def http_sse_stream(self):
        """
        SSE 实时日志流：直接 follow journal；每条推送与 /search 的元素结构一致。
        """
        def gen():
            j = systemd.journal.Reader()
            j.this_boot()
            j.log_level(systemd.journal.LOG_INFO)
            j.seek_tail()
            j.get_previous()   # 保证从“尾部之后”开始
            try:
                heartbeat_ts = time.time()
                while True:
                    # 最长等待 1 秒，避免连接长期无输出
                    if j.wait(1000) == systemd.journal.APPEND:
                        for rec in j:
                            data = {
                                "timestamp": self._fmt_ts(self._normalize_epoch_seconds(
                                    rec.get("__REALTIME_TIMESTAMP", 0))),
                                "priority": int(rec.get("PRIORITY", 0)),
                                "identifier": str(rec.get("SYSLOG_IDENTIFIER", "")),
                                "message": str(rec.get("MESSAGE", "")),
                            }
                            yield f"data: {json.dumps(data, ensure_ascii=False)}\n\n"
                            heartbeat_ts = time.time()
                    # 心跳，防止代理超时
                    if time.time() - heartbeat_ts > 15:
                        yield "data: {\"heartbeat\": true}\n\n"
                        heartbeat_ts = time.time()
            except GeneratorExit:
                # 客户端断开
                pass
            finally:
                j.close()

        headers = {
            "Content-Type": "text/event-stream; charset=utf-8",
            "Cache-Control": "no-cache",
            "X-Accel-Buffering": "no"  # Nginx 侧也需关闭缓冲
        }
        return Response(gen(), headers=headers)

    # ---------- 路由注册 ----------

    def _register_routes(self):
        self.app.add_url_rule('/search', view_func=self.http_search, methods=['GET'])
        self.app.add_url_rule('/api/logs/stream', view_func=self.http_sse_stream, methods=['GET'])


# 实例化并启动后台同步
logsvc = LogServiceV2(app=app)
logsvc.start_background_sync()


# =============================
# 以下为你原有的其他接口（保留）
# =============================

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
            capture_output=True, text=True, timeout=1
        )
        return result.stdout.split('\n')[0][:100]
    except Exception:
        return "N/A"

def get_last_login():
    try:
        result = subprocess.run(
            ['last', '-n', '1', '-i'],
            capture_output=True, text=True, timeout=2
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
    import socket as _sock
    with _sock.socket(_sock.AF_INET, _sock.SOCK_STREAM) as s:
        return s.connect_ex(('localhost', port)) == 0

def check_file_integrity():
    try:
        result = subprocess.run(['fsck', '-N'], capture_output=True, text=True, timeout=5)
        return "正常" if result.returncode == 0 else "异常"
    except Exception:
        return "异常"

def check_network_connectivity():
    try:
        result = subprocess.run(['ping', '-c', '1', '127.0.0.1'], capture_output=True, text=True, timeout=5)
        return "正常" if result.returncode == 0 else "异常"
    except Exception:
        return "异常"

def get_disk_usage():
    try:
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
    except Exception:
        return {}

def get_uptime():
    try:
        result = subprocess.run(['uptime', '-p'], capture_output=True, text=True, timeout=5)
        return result.stdout.strip()
    except Exception:
        return "N/A"

@app.after_request
def add_cors_headers(response):
    response.headers['Access-Control-Allow-Origin'] = '*'
    return response

@app.route('/api/login', methods=['POST'])
def login():
    try:
        data = request.get_json()
        username = data.get('username')
        password = data.get('password')

        try:
            auth_result = pam.authenticate(username, password)
        except Exception:
            return jsonify({"success": False, "code": "AUTH_SYSTEM_ERROR", "message": "认证系统暂时不可用"}), 500

        if auth_result:
            session.clear()
            session['logged_in'] = True
            session.permanent = True
            return jsonify({"success": True, "code": "AUTH_SUCCESS", "user": username, "redirect": "/", "message": "认证成功"}), 200
        else:
            return jsonify({"success": False, "code": "INVALID_CREDENTIALS", "message": "用户名或密码无效"}), 401

    except Exception:
        return jsonify({"success": False, "code": "INTERNAL_ERROR", "message": "系统服务暂时不可用"}), 500

@app.route('/api/check-auth', methods=['GET'])
def check_auth():
    if session.get('logged_in'):
        return jsonify({"authenticated": True}), 200
    return jsonify({"authenticated": False}), 401

@app.route('/api/overview', methods=['GET'])
def system_overview():
    if not session.get('logged_in'):
        return jsonify({"error": "未认证"}), 401
    try:
        hw = get_hardware_info()
        return jsonify({
            "kernel-version": f"{get_kernel_version()} | {get_kernel_patches()}",
            "login-history": get_last_login(),
            "device-info": f"{hw['product_name']} | SN:{hw['product_serial']}",
            "cpu-info": f"{get_cpu_model()} ({get_cpu_arch()})",
            "firmware-info": f"BIOS:{hw['bios_version']}"
        }), 200
    except Exception:
        return jsonify({"code": "OVERVIEW_ERROR", "message": "获取系统信息失败"}), 500

@app.route('/api/system-info', methods=['GET'])
def get_system_info():
    return jsonify({
        "system_version": get_system_version(),
        "device_name": get_device_name()
    })

# ========== 指标与设备信息（你原本就有；保留） ==========

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
    available = mem_info.get('MemAvailable', mem_info.get('MemFree', 0))
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
                temps.append(int(f.read()) / 1000.0)
        except Exception:
            pass
    return sum(temps) / len(temps) if temps else None

def get_top_processes():
    processes = []
    for proc in psutil.process_iter(['pid', 'status', 'cpu_percent', 'name']):
        try:
            processes.append(proc.info)
        except (psutil.NoSuchProcess, psutil.AccessDenied, psutil.ZombieProcess):
            pass
    return sorted(processes, key=lambda x: x['cpu_percent'], reverse=True)[:5]

@app.route('/metrics/stream')
def sse_stream():
    def event_stream():
        last_top_update = 0
        cached_top = []
        try:
            while True:
                now = time.time()
                if now - last_top_update >= 3:
                    cached_top = get_top_processes()
                    last_top_update = now
                metrics = {
                    'cpu_usage': round(_get_cpu_percent(), 1),
                    'memory_usage': round(_get_memory_percent(), 1),
                    'temperature': round(_get_temperature(), 1) if _get_temperature() else None,
                    'load_avg': round(_get_load_avg(), 2),
                    'top_processes': cached_top
                }
                yield f"data: {json.dumps(metrics)}\n\n"
                time.sleep(1)
        except GeneratorExit:
            pass
    return Response(event_stream(), mimetype="text/event-stream")

@app.route('/api/disk_info', methods=['GET'])
def get_disk_info():
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
    io = psutil.disk_io_counters(perdisk=True)
    total_read = 0
    total_write = 0
    for disk, counters in io.items():
        if not disk.startswith(('loop', 'sr', 'md', 'dm-')):
            total_read += counters.read_bytes
            total_write += counters.write_bytes
    return total_read, total_write

def monitor_disk_io():
    prev_r, prev_w = get_disk_io()
    while True:
        time.sleep(1)
        cur_r, cur_w = get_disk_io()
        data = {
            'read_speed': int(cur_r - prev_r),
            'write_speed': int(cur_w - prev_w)
        }
        yield f"data: {json.dumps(data)}\n\n"
        prev_r, prev_w = cur_r, cur_w

@app.route('/stream/disk')
def stream_disk():
    return Response(monitor_disk_io(), mimetype='text/event-stream')

def get_network_io():
    io = psutil.net_io_counters(pernic=True)
    total_sent = 0
    total_recv = 0
    for name, counters in io.items():
        if not name.startswith(('lo', 'veth', 'docker')):
            total_sent += counters.bytes_sent
            total_recv += counters.bytes_recv
    return total_sent, total_recv

def monitor_network_io():
    prev_s, prev_r = get_network_io()
    while True:
        time.sleep(1)
        cur_s, cur_r = get_network_io()
        data = {
            'sent_speed': int(cur_s - prev_s),
            'recv_speed': int(cur_r - prev_r)
        }
        yield f"data: {json.dumps(data)}\n\n"
        prev_s, prev_r = cur_s, cur_r

@app.route('/api/network_info', methods=['GET'])
def get_network_info():
    net_interfaces = psutil.net_if_addrs()
    out = []
    for ifname, addrs in net_interfaces.items():
        row = {"interface": ifname, "addresses": []}
        for a in addrs:
            row["addresses"].append({
                "family": str(a.family),
                "address": a.address,
                "netmask": a.netmask,
                "broadcast": a.broadcast,
                "ptp": a.ptp
            })
        out.append(row)
    return jsonify(out)

@app.route('/stream/network')
def stream_network():
    return Response(monitor_network_io(), mimetype='text/event-stream')


# =============================
# 启动
# =============================
if __name__ == '__main__':
    http_port = 8000
    if is_port_in_use(http_port):
        print(f"端口 {http_port} 已被占用，请释放端口后再运行。")
    else:
        app.run(host='0.0.0.0', port=http_port, threaded=True)
