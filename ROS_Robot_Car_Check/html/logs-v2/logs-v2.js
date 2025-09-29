
(() => {
  const { $, $$, PAGE_SIZE = 200, resolveTimeRange, fetchSearch, openRealtime } = LOGS_API;

  /* =========================
   * DOM 引用
   * ========================= */
  const el = {
    // 条件
    timePreset: $('#timePreset'),
    startTime:  $('#startTime'),
    endTime:    $('#endTime'),
    customWrap: $('#customTimeRange'),

    levelChecks: $$('.level-grid input, .level-row input'), // 兼容 2x2 / 横排两种结构
    serviceRoot: $('#service-filter'), // details.select-multi（内含多选 checkbox）

    keyword:   $('#keyword'),
    useRegex:  $('#useRegex'),

    // 操作
    searchBtn: $('#searchBtn'),
    exportBtn: $('#exportBtn'),
    clearBtn:  $('#clearBtn'),

    // 顶部统计
    resultCount: $('#resultCount'),
    queryTime:   $('#queryTime'),

    // 左列结果
    resultCard: $('#resultCard'),
    loading:    $('#loadingIndicator'),
    emptyState: $('#emptyState'),
    logsContent: $('#logsContent'),
    loadMoreBox: $('#loadMoreContainer'),
    loadMoreBtn: $('#loadMoreBtn'),

    // 右列实时
    realtimeBody: $('#realtimeContent'),
    realtimePlaceholder: $('#realtimePlaceholder'),
    pauseBtn:   $('#pauseRealtimeBtn'),
    closeBtn:   $('#closeRealtimeBtn'),
  };

  /* =========================
   * 状态
   * ========================= */
  let state = {
    phase: 'idle', // idle | loading | loaded | empty | error
    nextCursor: null,
    total: 0,
    tookMs: 0,
    aborter: null,

    // 实时
    realtimeConn: null,
    realtimePaused: false,
    realtimeBuffer: [],
  };

  /* =========================
   * 工具
   * ========================= */
  function setPhase(phase) {
    state.phase = phase;

    // 左侧可视控制
    if (el.loading) {
      el.loading.style.display = (phase === 'loading') ? 'flex' : 'none';
    }
    if (el.emptyState) {
      el.emptyState.classList.toggle('hidden', phase !== 'empty' && phase !== 'error');
      if (phase === 'error') {
        const t = el.emptyState.querySelector('.empty-title');
        const d = el.emptyState.querySelector('.empty-desc');
        if (t) t.textContent = '查询失败';
        if (d) d.textContent = '网络异常或服务器错误，请稍后重试。';
      }
    }
    if (el.loadMoreBox) {
      el.loadMoreBox.classList.toggle('hidden', !(phase === 'loaded' && state.nextCursor));
    }
  }

  function escapeHtml(s) {
    return String(s).replace(/[&<>"]/g, c => ({'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;'}[c]));
  }

  function renderLogItem(item) {
    // 兼容字段名
    const ts  = item.timestamp || item.ts || item.time || item._time || '';
    const svc = item.service || item.unit || item.source || item._systemd_unit || '';
    const pri = item.level ?? item.priority ?? item.pri ?? '';
    const msg = item.message ?? item.msg ?? item._message ?? JSON.stringify(item);

    const div = document.createElement('div');
    div.className = `log-entry level-${pri}`;
    div.innerHTML = `
      <div class="log-time">${escapeHtml(ts)}</div>
      <div class="log-service">${escapeHtml(svc)}</div>
      <div class="log-level">${escapeHtml(pri)}</div>
      <div class="log-message">${escapeHtml(msg)}</div>
    `;
    return div;
  }

  function clearResultsView() {
    if (el.logsContent) el.logsContent.innerHTML = '';
    state.total = 0;
    state.tookMs = 0;
    state.nextCursor = null;
    if (el.resultCount) el.resultCount.textContent = '共 0 条结果';
    if (el.queryTime) el.queryTime.textContent = '查询耗时：0ms';
  }

  function headerStats(total, ms) {
    if (el.resultCount) el.resultCount.textContent = `共 ${total} 条结果`;
    if (el.queryTime) el.queryTime.textContent = `查询耗时：${ms}ms`;
  }

  function appendItems(items) {
    if (!el.logsContent || !items?.length) return;
    const frag = document.createDocumentFragment();
    for (const it of items) frag.appendChild(renderLogItem(it));
    el.logsContent.appendChild(frag);
  }

  // 时间预设控制自定义显隐
  el.timePreset?.addEventListener('change', () => {
    if (!el.customWrap) return;
    el.customWrap.style.display = (el.timePreset.value === 'custom') ? 'block' : 'none';
  });

  // 解析级别（支持“合并值”，如 "0,1,2,3"）
  function getSelectedLevels() {
    const picked = [];
    el.levelChecks.forEach(ck => {
      if (!ck.checked) return;
      const parts = String(ck.value).split(',').map(s => s.trim()).filter(Boolean);
      parts.forEach(p => picked.push(Number(p)));
    });
    return Array.from(new Set(picked)).sort((a,b)=>a-b);
  }

  // 解析服务来源（details.select-multi 内的 checkbox）
  function getSelectedServices() {
    const root = el.serviceRoot;
    if (!root) return [];
    const checks = Array.from(root.querySelectorAll('input[type="checkbox"]'));
    if (!checks.length) return [];
    const checked = checks.filter(c => c.checked).map(c => c.value);

    // 规则：选中“全部”（空字符串）则表示不限服务 → 返回 []
    if (checked.includes('')) return [];
    return checked.filter(Boolean);
  }

  /* =========================
   * 查询主流程
   * ========================= */
  async function runSearch({ append = false } = {}) {
    // 取消前一个请求
    if (state.aborter) {
      try { state.aborter.abort(); } catch {}
    }
    state.aborter = new AbortController();

    try {
      const preset = el.timePreset?.value || '24h';
      const [startISO, endISO] = resolveTimeRange(preset, el.startTime, el.endTime);
      const levels   = getSelectedLevels();
      const services = getSelectedServices();
      const keyword  = el.keyword?.value?.trim() || '';
      const useRegex = !!el.useRegex?.checked;
      const cursor   = append ? state.nextCursor : null;

      const { items, next, tookMs, total } = await fetchSearch(
        { startISO, endISO, levels, services, keyword, useRegex, pageSize: PAGE_SIZE, cursor },
        state.aborter.signal
      );

      if (!append) {
        if (!items || items.length === 0) {
          state.total = total || 0;
          state.tookMs = tookMs || 0;
          headerStats(state.total, state.tookMs);
          setPhase('empty');
          return;
        }
        state.total = total || items.length;
        state.tookMs = tookMs || 0;
        headerStats(state.total, state.tookMs);
        setPhase('loaded');
        appendItems(items);
      } else {
        appendItems(items);
      }

      state.nextCursor = next || null;
      if (el.loadMoreBox) el.loadMoreBox.classList.toggle('hidden', !state.nextCursor);
    } catch (e) {
      if (e?.name === 'AbortError') return; // 主动取消
      console.error('[logs] search error:', e);
      setPhase('error');
    } finally {
      // 查询结束隐藏 loading（HTML 外移的 UI 状态也会感知）
      if (el.loading) el.loading.style.display = 'none';
    }
  }

  /* =========================
   * 导出
   * ========================= */
  function exportCurrent() {
    if (!el.logsContent) return;
    const lines = Array.from(el.logsContent.querySelectorAll('.log-entry')).map(div => {
      const t = div.querySelector('.log-time')?.textContent ?? '';
      const s = div.querySelector('.log-service')?.textContent ?? '';
      const l = div.querySelector('.log-level')?.textContent ?? '';
      const m = div.querySelector('.log-message')?.textContent ?? '';
      return `[${t}] (${s}) <${l}> ${m}`;
    });
    const blob = new Blob([lines.join('\n')], { type: 'text/plain;charset=utf-8' });
    const url = URL.createObjectURL(blob);
    const a = document.createElement('a');
    a.href = url; a.download = `logs_${Date.now()}.txt`; a.click();
    URL.revokeObjectURL(url);
  }

  /* =========================
   * 清空
   * ========================= */
  function clearAll() {
    // 取消请求
    if (state.aborter) { try { state.aborter.abort(); } catch {} }
    clearResultsView();
    setPhase('idle');
  }

  /* =========================
   * 实时日志
   * ========================= */
  function startRealtime() {
    if (!el.realtimeBody) return;

    const MAX_ROWS = 800;  // DOM 上限
    const BATCH_MS = 500;  // 批量写入间隔

    function pushBatch(batch) {
      if (!batch.length) return;
      const frag = document.createDocumentFragment();
      for (const d of batch) {
        frag.appendChild(renderLogItem({
          timestamp: d.timestamp || d.ts || new Date().toISOString(),
          service:   d.service || d._systemd_unit || d.unit || 'realtime',
          priority:  d.level ?? d.priority ?? d.pri ?? 6,
          message:   d.message ?? JSON.stringify(d)
        }));
      }
      el.realtimeBody.appendChild(frag);
      // 超出清理
      const children = el.realtimeBody.children;
      while (children.length > MAX_ROWS) el.realtimeBody.removeChild(children[0]);

      // 有内容后隐藏占位
      el.realtimePlaceholder?.classList.add('hidden');
    }

    state.realtimeConn = openRealtime({
      onOpen: () => {},
      onError: () => {},
      onMessage: data => {
        if (state.realtimePaused) return;
        state.realtimeBuffer.push(data);
      }
    });

    state.realtimeConn.connect();

    setInterval(() => {
      if (state.realtimePaused || state.realtimeBuffer.length === 0) return;
      const batch = state.realtimeBuffer.splice(0, state.realtimeBuffer.length);
      pushBatch(batch);
    }, BATCH_MS);
  }

  function pauseRealtime(pause) {
    state.realtimePaused = !!pause;
    if (el.pauseBtn) el.pauseBtn.textContent = pause ? '继续' : '暂停';
  }

  function closeRealtime() {
    try { state.realtimeConn && state.realtimeConn.close(); } catch {}
    state.realtimeConn = null;
    state.realtimePaused = false;
    if (el.realtimeBody) el.realtimeBody.innerHTML = '';
    el.realtimePlaceholder?.classList.remove('hidden');
  }

  /* =========================
   * 事件绑定
   * ========================= */
  el.searchBtn?.addEventListener('click', () => runSearch({ append: false }));
  el.loadMoreBtn?.addEventListener('click', () => {
    if (state.nextCursor) runSearch({ append: true });
  });
  el.exportBtn?.addEventListener('click', exportCurrent);
  el.clearBtn?.addEventListener('click', clearAll);

  el.pauseBtn?.addEventListener('click', () => pauseRealtime(!state.realtimePaused));
  el.closeBtn?.addEventListener('click', closeRealtime);

  // 初始化：首屏 idle，不主动查询；实时流自动连接
  setPhase('idle');
  startRealtime();
})();

/* =======================================================================
 * 以下逻辑原先在 HTML 内联脚本中，现在整合到此文件。用于处理：
 * - 点击“查询/清空”时的页面态（query-running 类）
 * - 右侧实时占位的显示/隐藏
 * ======================================================================= */
(() => {
  if (window.__logsV2InlineInitInstalled) return;
  window.__logsV2InlineInitInstalled = true;

  function setupInlineUiState() {
    const body = document.body;
    const searchBtn = document.getElementById('searchBtn');
    const clearBtn  = document.getElementById('clearBtn');
    const initialHint = document.getElementById('initialHint');
    const loading = document.getElementById('loadingIndicator');
    const realtimeContent = document.getElementById('realtimeContent');
    const realtimePlaceholder = document.getElementById('realtimePlaceholder');

    // 查询 → 进入查询中态
    if (searchBtn) {
      searchBtn.addEventListener('click', () => {
        body.classList.add('query-running');
        if (initialHint) initialHint.classList.add('hidden');
      });
    }

    // 清空 → 回到未查询态
    if (clearBtn) {
      clearBtn.addEventListener('click', () => {
        body.classList.remove('query-running');
        if (initialHint) initialHint.classList.remove('hidden');
      });
    }

    // 监听 loading 显隐：查询完成时移除 query-running
    if (loading) {
      const mo = new MutationObserver(() => {
        const style = window.getComputedStyle(loading);
        if (style.display === 'none') {
          body.classList.remove('query-running');
        }
      });
      mo.observe(loading, { attributes: true, attributeFilter: ['style', 'class'] });
    }

    // 右侧占位：有内容则隐藏占位，无内容显示
    if (realtimeContent && realtimePlaceholder) {
      const toggle = () => {
        const hasContent = realtimeContent.textContent.trim().length > 0 ||
                           realtimeContent.children.length > 0;
        realtimePlaceholder.classList.toggle('hidden', hasContent);
      };
      const ro = new MutationObserver(toggle);
      ro.observe(realtimeContent, { childList: true, subtree: true, characterData: true });
      toggle();
    }
  }

  if (document.readyState === 'loading') {
    document.addEventListener('DOMContentLoaded', setupInlineUiState, { once: true });
  } else {
    setupInlineUiState();
  }
})();
