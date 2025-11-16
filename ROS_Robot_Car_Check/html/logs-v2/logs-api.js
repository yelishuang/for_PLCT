/* logs-api.js
 * 统一封装日志查询与实时流。可配置端点；含超时、取消、健壮返回解析。
 */

const LOGS_API = (() => {
  // ======= 配置区（按需修改） =======
  const SEARCH_URL = "/search";            // 后端查询接口（POST/GET 均可，见 fetchSearch）
  const REALTIME_URL = "/api/logs/stream"; // 日志实时流 SSE 端点（不存在则自动降级）
  const REQUEST_TIMEOUT_MS = 30_000;       // 查询超时
  const PAGE_SIZE = 200;                   // 每页条数（避免一次性巨量渲染）
  // =================================

  // 将“时间预设”解析为 [startISO, endISO]
  function resolveTimeRange(preset, startInput, endInput) {
    const now = new Date();
    const endISO = now.toISOString();
    const start = new Date(now);

    const sets = {
      "15m": 15, "1h": 60, "6h": 360, "24h": 1440, "3d": 4320, "7d": 10080
    };
    if (preset && sets[preset]) {
      start.setMinutes(now.getMinutes() - sets[preset]);
      return [start.toISOString(), endISO];
    }
    if (preset === "today") {
      const s = new Date(now); s.setHours(0,0,0,0);
      return [s.toISOString(), endISO];
    }
    // 自定义
    const sVal = startInput?.value;
    const eVal = endInput?.value;
    const sISO = sVal ? new Date(sVal).toISOString() : null;
    const eISO = eVal ? new Date(eVal).toISOString() : endISO;
    return [sISO, eISO];
  }

  // 统一超时 + 取消
  function fetchWithTimeout(url, opts = {}, timeout = REQUEST_TIMEOUT_MS, externalAbort) {
    const controller = new AbortController();
    const id = setTimeout(() => controller.abort("timeout"), timeout);
    const signal = externalAbort ?? controller.signal;

    return fetch(url, { ...opts, signal })
      .finally(() => clearTimeout(id));
  }

  // 兼容性解析后端返回
  function normalizeResponse(json) {
    // 期望形如：
    // { results: [...], next: "cursor", took_ms: 12, total: 123 }
    // 兼容备用字段：
    const items = json.results || json.items || json.data || [];
    const next = json.next || json.nextCursor || json.cursor || null;
    const tookMs = json.took_ms ?? json.queryTime ?? json.elapsed_ms ?? 0;
    const total = json.total ?? json.count ?? items.length;
    return { items, next, tookMs, total };
  }

  // 查询接口：params = { startISO, endISO, levels[], services[], keyword, useRegex, pageSize, cursor }
  async function fetchSearch(params, externalAbort) {
    // 支持 POST JSON；若你的后端要求 GET，请改这里序列化为 querystring
    const res = await fetchWithTimeout(
      SEARCH_URL,
      {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({
          start: params.startISO,
          end: params.endISO,
          levels: params.levels,
          services: params.services,
          keyword: params.keyword || "",
          regex: !!params.useRegex,
          limit: params.pageSize ?? PAGE_SIZE,
          cursor: params.cursor ?? null
        })
      },
      REQUEST_TIMEOUT_MS,
      externalAbort
    );

    if (!res.ok) throw new Error(`HTTP ${res.status}`);
    const json = await res.json().catch(() => ({ results: [] }));
    return normalizeResponse(json);
  }

  // 实时日志：返回 { connect, close }
  function openRealtime({ onMessage, onOpen, onError }) {
    if (!window.EventSource || !REALTIME_URL) {
      onError && onError(new Error("SSE not supported/or endpoint missing"));
      return { connect: () => {}, close: () => {} };
    }
    let es = null;
    return {
      connect() {
        try {
          es = new EventSource(REALTIME_URL);
          es.onopen = () => onOpen && onOpen();
          es.onerror = (e) => onError && onError(e);
          es.onmessage = (evt) => {
            // 支持两种：纯行文本 / JSON 对象
            try {
              const data = JSON.parse(evt.data);
              onMessage && onMessage(data);
            } catch {
              onMessage && onMessage({ message: evt.data });
            }
          };
        } catch (e) {
          onError && onError(e);
        }
      },
      close() {
        try { es && es.close(); } catch {}
        es = null;
      }
    };
  }

  // DOM 辅助
  const $ = (sel, root = document) => root.querySelector(sel);
  const $$ = (sel, root = document) => Array.from(root.querySelectorAll(sel));

  // 暴露 API
  return {
    PAGE_SIZE,
    resolveTimeRange,
    fetchSearch,
    openRealtime,
    $, $$
  };
})();

