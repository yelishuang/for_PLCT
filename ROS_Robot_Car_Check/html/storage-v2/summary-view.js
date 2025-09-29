// 左侧“概览 + 表格”视图
import { StorageAPI } from './storage-api.js';
import { MiniLine } from './chart-lite.js';

export class SummaryView {
  constructor(){
    // KPI
    this.elCapTotal = document.getElementById('capTotal');
    this.elCapFree  = document.getElementById('capFree');
    this.elRoot     = document.getElementById('rootUsage');
    this.elTemp     = document.getElementById('maxTemp');

    // 即时速率
    this.elReadNow  = document.getElementById('readNow');
    this.elWriteNow = document.getElementById('writeNow');

    // 状态
    this.stateIdle     = document.getElementById('stateIdle');
    this.stateLoading  = document.getElementById('stateLoading');
    this.stateEmpty    = document.getElementById('stateEmpty');
    this.stateError    = document.getElementById('stateError');

    // 表格
    this.tbody = document.getElementById('diskRows');

    // 按钮
    document.getElementById('btnRefresh')?.addEventListener('click', () => this.refresh());
    document.getElementById('btnExport')?.addEventListener('click', () => this.exportDiag());

    // 图表实例
    this.readChart  = new MiniLine('readChart',  '#4a90e2', 90);
    this.writeChart = new MiniLine('writeChart', '#f59e0b', 90);
  }

  /* 状态机可视切换 */
  setState(which){
    const show = id => id?.classList.remove('hidden');
    const hide = id => id?.classList.add('hidden');
    [this.stateIdle,this.stateLoading,this.stateEmpty,this.stateError].forEach(hide);
    ({idle:this.stateIdle,loading:this.stateLoading,empty:this.stateEmpty,error:this.stateError}[which])?.classList.remove('hidden');
  }

  /* 刷新汇总与表格 */
  async refresh(){
    this.setState('loading');
    try{
      const [sum, devs] = await Promise.all([
        StorageAPI.getSummary(),
        StorageAPI.getDevices()
      ]);
      // KPI
      this.elCapTotal.textContent = StorageAPI.fmtBytes(sum.total);
      this.elCapFree.textContent  = StorageAPI.fmtBytes(sum.free);
      this.elRoot.textContent     = (sum.rootUsage ?? '-') + '%';
      this.elTemp.textContent     = (sum.maxTemp==null)?'N/A':`${sum.maxTemp} ℃`;

      // 表格
      this.renderTable(devs);

      this.setState(devs?.length ? 'idle' : 'empty');
    }catch(e){
      console.error(e);
      this.setState('error');
    }
  }

  renderTable(rows=[]){
    this.tbody.innerHTML = '';
    const frag = document.createDocumentFragment();
    rows.forEach(r=>{
      const tr = document.createElement('tr');
      tr.innerHTML = `
        <td>${escapeHtml(r.device)}</td>
        <td>${escapeHtml(r.mountpoint||'-')}</td>
        <td>${escapeHtml(r.fstype||'-')}</td>
        <td>${escapeHtml(r.total||'-')}</td>
        <td>${escapeHtml(r.used||'-')}</td>
        <td>${escapeHtml(r.free||'-')}</td>
        <td>${escapeHtml(r.percent||'-')}</td>
        <td>${this.healthBadge(r.health)}</td>
      `;
      frag.appendChild(tr);
    });
    this.tbody.appendChild(frag);
  }

  healthBadge(h){
    const map = { ok:'badge-ok', warn:'badge-warn', bad:'badge-bad' };
    const text = { ok:'良好', warn:'警告', bad:'故障' }[h] || '未知';
    const cls = map[h] || '';
    return `<span class="${cls}">${text}</span>`;
  }

  // 被实时面板调用：更新即时速率 + 图表推点
  pushRealtime({ read_bps=0, write_bps=0 }){
    const fmt = StorageAPI.fmtBytes;
    this.elReadNow.textContent  = fmt(read_bps).split(' ')[0];
    this.elWriteNow.textContent = fmt(write_bps).split(' ')[0];
    // 单位文本用 CSS .unit 固定为 B/s，避免频繁改 DOM（如需动态单位可扩展）
    this.readChart.push(read_bps);
    this.writeChart.push(write_bps);
  }

  exportDiag(){
    // 前端占位：实际实现可请求后端 task 打包
    alert('诊断导出：占位实现（前端）。接入后端后可返回任务ID并跟踪进度。');
  }
}

function escapeHtml(s){ return String(s??'').replace(/[&<>"]/g, m=>({'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;'}[m])); }
