/* network-v2.js  —— 仅前端，自动初始化版本（适配 loadContent 片段注入）
 * 设计目标：轻量、低开销、状态机、原生 Canvas 曲线、SSE 可接入，Mock 回退。
 */
(function(){
  // ========== 工具 ==========
  const $  = (sel, root=document) => root.querySelector(sel);
  const $$ = (sel, root=document) => Array.from(root.querySelectorAll(sel));
  const fmtBps = (n)=>{ if(n==null||isNaN(n)) return "0 B/s"; const u=["B/s","KB/s","MB/s","GB/s"];
    let i=0,v=n; while(v>=1024 && i<u.length-1){v/=1024;i++} return `${(v<10? v.toFixed(2): v.toFixed(1))} ${u[i]}`; };
  const escapeHtml = s => String(s??'').replace(/[&<>"]/g,m=>({'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;'}[m]));

  // ========== 轻量折线图 ==========
  function MiniDualLine(canvasId, colorRx="#3b82f6", colorTx="#f59e0b", max=90){
    const c = document.getElementById(canvasId); const ctx = c.getContext('2d');
    let rx=[], tx=[];
    function resize(){
      const dpr=Math.max(1,window.devicePixelRatio||1);
      const w=c.clientWidth||360, h=c.clientHeight||140;
      c.width=Math.floor(w*dpr); c.height=Math.floor(h*dpr); ctx.setTransform(dpr,0,0,dpr,0,0); draw();
    }
    function push(r,v){ r.push(v); if(r.length>max) r.shift(); }
    function draw(){
      const w=c.clientWidth||360, h=c.clientHeight||140; ctx.clearRect(0,0,w,h);
      // grid
      ctx.strokeStyle="#2a3340"; ctx.lineWidth=1;
      for(let i=0;i<4;i++){ const y=Math.round((h/4)*(i+1)); ctx.beginPath(); ctx.moveTo(0,y); ctx.lineTo(w,y); ctx.stroke(); }
      const maxVal=Math.max(1, ...rx, ...tx);
      const step=w/(max-1); const plot=(arr,color)=>{ ctx.strokeStyle=color; ctx.lineWidth=2; ctx.beginPath();
        for(let i=0;i<arr.length;i++){ const x=i*step; const y=h - (arr[i]/maxVal)*(h-6); if(i===0) ctx.moveTo(x,y); else ctx.lineTo(x,y); } ctx.stroke(); };
      if(rx.length>1) plot(rx,colorRx); if(tx.length>1) plot(tx,colorTx);
    }
    window.addEventListener('resize', resize);
    resize();
    return {
      push(rxVal,txVal){ push(rx,rxVal||0); push(tx,txVal||0); draw(); }
    };
  }

  // ========== API（Mock 回退） ==========
  const API = (() => {
    const LAT = 240;

    async function getSummary(){
      // 真实接口示例：
      // const r = await fetch('/api/net/summary'); return r.json();
      await delay(LAT);
      return { gateway: "192.168.1.1", dns: ["223.5.5.5","8.8.8.8"] };
    }
    async function getInterfaces(){
      // const r = await fetch('/api/net/interfaces'); return r.json();
      await delay(LAT);
      return [
        { name:"eth0", ipv4:["192.168.1.50/24"], ipv6:[], mac:"02:42:ac:11:00:02", mtu:1500, speed:"1000Mbps", duplex:"full",
          rx_ok: 321345, rx_err:2, tx_ok: 289001, tx_err:0, operstate:"UP" },
        { name:"wlan0", ipv4:["192.168.1.60/24"], ipv6:["fe80::1"], mac:"b8:27:eb:12:34:56", mtu:1500, speed:"300Mbps", duplex:"half",
          rx_ok: 19123, rx_err:0, tx_ok: 21034, tx_err:1, operstate:"DORMANT" }
      ];
    }
    async function getRoutes(){
      // const r = await fetch('/api/net/routes'); return r.json();
      await delay(LAT);
      return [
        { dst:"default", via:"192.168.1.1", dev:"eth0" },
        { dst:"192.168.1.0/24", dev:"eth0" }
      ];
    }
    async function getDNS(){
      // const r = await fetch('/api/net/dns'); return r.json();
      await delay(LAT);
      return ["223.5.5.5","8.8.8.8"];
    }
    async function getConnections(){
      // const r = await fetch('/api/net/sockets?top=10'); return r.json();
      await delay(LAT);
      return [
        { local:"192.168.1.50:22", remote:"203.0.113.10:51432", proto:"TCP", proc:"sshd(712)", bytes: 1_200_000 },
        { local:"192.168.1.50:443", remote:"198.51.100.8:58321", proto:"TCP", proc:"nginx(901)", bytes: 950_000 },
      ];
    }
    function openRealtime({ onOpen, onMessage, onError }){
      // 真实 SSE：
      // const es = new EventSource('/stream/net'); es.onmessage = e => onMessage(JSON.parse(e.data))
      let t=null, opened=false;
      return {
        connect(){ opened=true; onOpen && onOpen(); t=setInterval(()=>{
          if(!opened) return; const rx= (500+Math.random()*5000)*1024, tx=(300+Math.random()*3000)*1024;
          onMessage && onMessage({ ts: Date.now(), rx_bps: Math.round(rx), tx_bps: Math.round(tx), kind: (Math.random()>0.5?'rx':'tx'), iface: 'eth0' });
        },1000); },
        close(){ opened=false; if(t) clearInterval(t); }
      };
    }
    function delay(ms){ return new Promise(r=>setTimeout(r,ms)); }
    return { getSummary, getInterfaces, getRoutes, getDNS, getConnections, openRealtime };
  })();

  // ========== 视图层 ==========
  function NetV2(){
    // 根与元素引用
    const root = $('#netv2-root'); if(!root) return;
    const els = {
      kpiTxNow: $('#kpiTxNow', root), kpiRxNow: $('#kpiRxNow', root),
      kpiGateway: $('#kpiGateway', root), kpiDNS: $('#kpiDNS', root),
      rxNow: $('#rxNow', root), txNow: $('#txNow', root),
      sIdle: $('#sIdle', root), sLoading: $('#sLoading', root), sEmpty: $('#sEmpty', root), sError: $('#sError', root),
      ifaceRows: $('#ifaceRows', root),
      btnRefresh: $('#btnNetRefresh', root),
      routeList: $('#routeList', root), dnsList: $('#dnsList', root),
      connRows: $('#connRows', root),
      rtList: $('#rtList', root), rtPlaceholder: $('#rtPlaceholder', root),
      btnRtPause: $('#btnRtPause', root), btnRtClear: $('#btnRtClear', root),
      statusChip: $('#netv2-status', root)
    };

    // 曲线
    const chart = MiniDualLine('netv2Chart', '#3b82f6', '#f59e0b', 90);

    // 状态
    const state = {
      paused: false,
      conn: null,
      rxBps: 0, txBps: 0
    };

    // 公开
    this.destroy = () => { try{ state.conn?.close(); }catch{} };

    // 事件
    els.btnRefresh?.addEventListener('click', () => loadAll());
    els.btnRtPause?.addEventListener('click', () => {
      state.paused = !state.paused;
      els.btnRtPause.textContent = state.paused ? '继续' : '暂停';
    });
    els.btnRtClear?.addEventListener('click', () => {
      els.rtList.innerHTML = ''; els.rtPlaceholder.classList.remove('hidden');
    });

    // 初始加载
    loadAll();
    startRealtime();

    /* ------ 加载整页数据 ------ */
    async function loadAll(){
      setListState('loading');
      try{
        const [sum, ifaces, routes, dns, conns] = await Promise.all([
          API.getSummary(), API.getInterfaces(), API.getRoutes(), API.getDNS(), API.getConnections()
        ]);

        // KPI
        setText(els.kpiGateway, sum?.gateway ?? '-');
        setText(els.kpiDNS, (dns && dns.length) ? dns.join(', ') : '-');

        // 接口表
        renderIfaces(ifaces);
        setListState((ifaces && ifaces.length) ? 'idle' : 'empty');

        // 路由 / DNS
        renderRoutes(routes||[]);
        renderDNS(dns||[]);

        // 连接
        renderConns(conns||[]);
      }catch(e){
        console.error(e);
        setListState('error');
      }
    }

    function setListState(which){
      const show = (el, yes)=> el && el.classList.toggle('hidden', !yes);
      show(els.sIdle, which==='idle');
      show(els.sLoading, which==='loading');
      show(els.sEmpty, which==='empty');
      show(els.sError, which==='error');
    }

    function setText(el, txt){ if(el) el.textContent = txt; }

    function renderIfaces(rows){
      els.ifaceRows.innerHTML = '';
      const frag = document.createDocumentFragment();
      rows.forEach(r=>{
        const tr = document.createElement('tr');
        tr.innerHTML = `
          <td><strong>${escapeHtml(r.name)}</strong><div class="muted">${escapeHtml(r.duplex||'-')} · ${escapeHtml(r.speed||'-')}</div></td>
          <td>${escapeHtml((r.ipv4||[]).join(', ')||'-')}<div class="muted">${escapeHtml((r.ipv6||[]).join(', ')||'')}</div></td>
          <td>${escapeHtml(r.mac||'-')}<div class="muted">MTU ${escapeHtml(r.mtu||'-')}</div></td>
          <td><span class="muted">—</span></td>
          <td>${(r.rx_ok??0).toLocaleString()} / <span class="muted">${r.rx_err??0}</span></td>
          <td>${(r.tx_ok??0).toLocaleString()} / <span class="muted">${r.tx_err??0}</span></td>
          <td>${statusBadge(r.operstate||'DOWN')}</td>
        `;
        frag.appendChild(tr);
      });
      els.ifaceRows.appendChild(frag);
    }

    function statusBadge(s){
      const m = String(s).toUpperCase();
      const cls = (m==='UP')?'badge-up':(m==='DORMANT'?'badge-dormant':'badge-down');
      const txt = (m==='UP')?'UP':(m==='DORMANT'?'DORMANT':'DOWN');
      return `<span class="${cls}">${txt}</span>`;
    }

    function renderRoutes(routes){
      els.routeList.innerHTML = '';
      routes.forEach(r=>{
        const li = document.createElement('li');
        li.innerHTML = `<strong>${escapeHtml(r.dst)}</strong> ${r.via?('via '+escapeHtml(r.via)):""} ${r.dev?('dev '+escapeHtml(r.dev)):""}`;
        els.routeList.appendChild(li);
      });
    }

    function renderDNS(list){
      els.dnsList.innerHTML = '';
      if(!list || !list.length){ els.dnsList.innerHTML = '<li>—</li>'; return; }
      list.forEach(ip=>{
        const li = document.createElement('li'); li.textContent = ip; els.dnsList.appendChild(li);
      });
    }

    function renderConns(rows){
      const tbody = els.connRows; tbody.innerHTML = '';
      const frag = document.createDocumentFragment();
      rows.forEach(r=>{
        const tr = document.createElement('tr');
        tr.innerHTML = `
          <td>${escapeHtml(r.local||'-')}</td>
          <td>${escapeHtml(r.remote||'-')}</td>
          <td>${escapeHtml(r.proto||'-')}</td>
          <td>${escapeHtml(r.proc||'-')}</td>
          <td>${fmtBps(r.bytes||0).replace('/s','')}</td>
        `;
        frag.appendChild(tr);
      });
      tbody.appendChild(frag);
    }

    /* ------ 实时 ------ */
    function startRealtime(){
      state.conn = API.openRealtime({
        onOpen: ()=> setOnline(true),
        onError: ()=> setOnline(false),
        onMessage: (ev)=>{
          if(state.paused) return;
          const rx = ev.rx_bps||0, tx=ev.tx_bps||0;
          state.rxBps = rx; state.txBps = tx;
          // KPI & 顶部数值
          setText(els.kpiRxNow, fmtBps(rx).split(' ')[0]);
          setText(els.kpiTxNow, fmtBps(tx).split(' ')[0]);
          setText(els.rxNow, fmtBps(rx).split(' ')[0]);
          setText(els.txNow, fmtBps(tx).split(' ')[0]);
          // 曲线
          chart.push(rx, tx);
          // 事件流
          pushEvent(ev);
        }
      });
      state.conn.connect();
    }

    function setOnline(ok){
      if(!els.statusChip) return;
      els.statusChip.textContent = ok ? '● 实时在线' : '● 连接中断';
      els.statusChip.classList.toggle('online', ok);
    }

    function pushEvent(ev){
      if(!els.rtList) return;
      els.rtPlaceholder?.classList.add('hidden');
      const row = document.createElement('div');
      const isRx = (ev.kind||'rx')==='rx';
      row.className = `rt-item ${isRx?'rx':'tx'}`;
      row.innerHTML = `
        <div class="rt-time">${new Date(ev.ts||Date.now()).toLocaleTimeString()}</div>
        <div>${isRx?'下行':'上行'} ${fmtBps((isRx?ev.rx_bps:ev.tx_bps)||0)} · ${escapeHtml(ev.iface||'-')}</div>
      `;
      els.rtList.appendChild(row);
      // 限制 120 行
      while(els.rtList.children.length > 120) els.rtList.removeChild(els.rtList.firstChild);
    }
  }

  // ========== 自动初始化：侦听片段注入 ==========
  (function autoMount(){
    const content = document.querySelector('.content-area') || document.querySelector('#content') || document.body;
    let current = null;

    function tryInit(){
      const root = document.getElementById('netv2-root');
      if(root && !root.__mounted){
        current?.destroy?.();           // 卸载旧实例（若有）
        current = new NetV2();          // 初始化
        root.__mounted = true;
      }
    }

    // 首次尝试
    tryInit();
    // 观察内容区 DOM 变更（适配 loadContent 注入）
    const mo = new MutationObserver(tryInit);
    mo.observe(content, { childList:true, subtree:true });
    // 离开页面时清理
    window.addEventListener('beforeunload', ()=> current?.destroy?.());
  })();
})();
