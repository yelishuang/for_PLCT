import { SummaryView } from './summary-view.js';
import { RealtimePanel } from './realtime-panel.js';

const summary = new SummaryView();
await summary.refresh(); // 首次加载

const rt = new RealtimePanel({
  onSample: (s) => summary.pushRealtime(s)
});
rt.start();

// 顶部状态灯（占位逻辑）
const status = document.getElementById('storageStatus');
if (status) {
  status.textContent = '● 监控就绪';
  status.style.background = '#e8fff2';
}
