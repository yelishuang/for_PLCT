// 右侧“实时 I/O 流”面板：SSE 钩子 + UI 渲染
import { StorageAPI } from './storage-api.js';

export class RealtimePanel {
  constructor({ onSample }){
    this.list = document.getElementById('rtList');
    this.placeholder = document.getElementById('rtPlaceholder');

    this.btnPause = document.getElementById('btnRtPause');
    this.btnClose = document.getElementById('btnRtClose');

    this.onSample = onSample;
    this.paused = false;
    this.conn = null;

    this.btnPause?.addEventListener('click', () => this.togglePause());
    this.btnClose?.addEventListener('click', () => this.close());
  }

  start(){
    this.conn = StorageAPI.openRealtime({
      onOpen: () => {},
      onError: () => {},
      onMessage: (data) => this.handle(data)
    });
    this.conn.connect();
  }

  handle({ ts=Date.now(), read_bps=0, write_bps=0 }){
    if (this.paused) return;

    // 折线图/即时数值
    this.onSample?.({ read_bps, write_bps });

    // 列表行（仅保留最近 60 条）
    const row = document.createElement('div');
    row.className = 'rt-item read';
    row.innerHTML = `
      <div class="rt-time">${new Date(ts).toLocaleTimeString()}</div>
      <div>读取 ${fmt(read_bps)}</div>
    `;
    const row2 = document.createElement('div');
    row2.className = 'rt-item write';
    row2.innerHTML = `
      <div class="rt-time">${new Date(ts).toLocaleTimeString()}</div>
      <div>写入 ${fmt(write_bps)}</div>
    `;

    this.placeholder?.classList.add('hidden');
    this.list.appendChild(row);
    this.list.appendChild(row2);

    while (this.list.children.length > 120) { // 60条读 + 60条写
      this.list.removeChild(this.list.firstChild);
    }
  }

  togglePause(){
    this.paused = !this.paused;
    if (this.btnPause) this.btnPause.textContent = this.paused ? '继续' : '暂停';
  }

  close(){
    try { this.conn?.close(); } catch {}
    this.conn = null;
    this.list.innerHTML = '';
    this.placeholder?.classList.remove('hidden');
  }
}

function fmt(n){
  const u = ["B/s","KB/s","MB/s","GB/s","TB/s"]; let i = 0, v = n;
  while (v >= 1024 && i < u.length-1){ v/=1024; i++; }
  return `${v.toFixed(v<10?2:1)} ${u[i]}`;
}
