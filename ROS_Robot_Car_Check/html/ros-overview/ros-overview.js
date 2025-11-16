/* ros-overview.js  —— ROS 2 概览与健康（前端）
 * 适配：loadContent 片段注入；脚本预加载后用 MutationObserver 自动初始化。
 * 数据：内置 Mock，便于无后端先跑通。接入后端只需替换 API.fetch。
 */
(function(){
  const $ = (s,r=document)=>r.querySelector(s);
  const $$=(s,r=document)=>Array.from(r.querySelectorAll(s));
  const esc = s => String(s??'').replace(/[&<>"]/g,m=>({'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;'}[m]));
  const fmtKBs = n => { if(!n) return '0'; const v=n/1024; return v<10? v.toFixed(2): v.toFixed(1); };
  const timeStr = ts => ts? new Date(ts).toLocaleString() : '—';

  // ---------- 轻量迷你折线 ----------
  function MiniLine(canvasId, max=60){
    const c = document.getElementById(canvasId); const ctx = c.getContext('2d');
    let data=[];
    function resize(){
      const dpr=Math.max(1,window.devicePixelRatio||1);
      const w=c.clientWidth||220, h=c.clientHeight||46;
      c.width=Math.floor(w*dpr); c.height=Math.floor(h*dpr); ctx.setTransform(dpr,0,0,dpr,0,0); draw();
    }
    function push(v){ data.push(v||0); if(data.length>max) data.shift(); draw(); }
    function draw(){
      const w=c.clientWidth||220, h=c.clientHeight||46; ctx.clearRect(0,0,w,h);
      // grid
      ctx.strokeStyle="#263043"; ctx.lineWidth=1;
      for(let i=0;i<2;i++){ const y=Math.round((h/3)*(i+1)); ctx.beginPath(); ctx.moveTo(0,y); ctx.lineTo(w,y); ctx.stroke(); }
      const maxVal=Math.max(1, ...data);
      const step=w/(max-1); ctx.strokeStyle="#4a90e2"; ctx.lineWidth=2; ctx.beginPath();
      for(let i=0;i<data.length;i++){ const x=i*step; const y=h - (data[i]/maxVal)*(h-6); if(i===0) ctx.moveTo(x,y); else ctx.lineTo(x,y); }
      ctx.stroke();
    }
    window.addEventListener('resize', resize); resize();
    return { push };
  }

  // ---------- Mock API（接入后端：替换成 fetch 即可） ----------
  const API = (()=>{
    async function getSummary(){
      // 示例：const r = await fetch('/api/ros/summary'); return r.json();
      await delay(180);
      return {
        online: true,
        bridge: 'rosbridge',
        domain_id: 42,
        host: location.hostname,
        nodes: 37,
        topics: 58,
        warn: 3,
        error: 1,
        avg_hz: 24.6,
        avg_bw: 180*1024,   // B/s
        bag: { running:false, size:0, since:null }
      };
    }
    async function getDiagnostics(){
      await delay(180);
      const now = Date.now();
      const rows = [
        { lvl:'OK',    src:'/battery_monitor', msg:'电压正常', ts: now-60000 },
        { lvl:'WARN',  src:'/lidar', msg:'丢包率偏高 3.2%', ts: now-45000 },
        { lvl:'OK',    src:'/imu', msg:'温度 42.1℃', ts: now-30000 },
        { lvl:'ERROR', src:'/camera', msg:'相机节点超时', ts: now-12000 },
        { lvl:'OK',    src:'/navigation', msg:'定位稳定', ts: now-8000 },
      ];
      return rows;
    }
    async function getTF(){
      await delay(160);
      return {
        frames: 12,
        root: 'map',
        updated: Date.now(),
        tree: { // 简化树（父 -> 子[]）
          map: ['odom'],
          odom: ['base_link'],
          base_link: ['base_laser','camera_link'],
          camera_link: ['camera_optical'],
        }
      };
    }
    function openStream({ onHzSample }){
      // 模拟每秒一个 hz 样本
      const t = setInterval(()=> onHzSample && onHzSample(15 + Math.random()*20), 1000);
      return { close(){ clearInterval(t); } };
    }
    function delay(ms){ return new Promise(r=>setTimeout(r,ms)); }
    return { getSummary, getDiagnostics, getTF, openStream };
  })();

  // ---------- 主视图 ----------
  function RosOverview(){
    const root = $('#rosov-root'); if(!root) return;

    // 头部与信息
    const els = {
      status: $('#rosov-status', root), reconnect: $('#btnRosReconnect', root),
      bridgeType: $('#bridgeType', root), rosDomain: $('#rosDomain', root),
      rosHost: $('#rosHost', root), rosDiscovery: $('#rosDiscovery', root),
      kpiNodes: $('#kpiNodes', root), kpiTopics: $('#kpiTopics', root),
      kpiWarn: $('#kpiWarn', root), kpiError: $('#kpiError', root),
      kpiHz: $('#kpiHz', root), kpiBw: $('#kpiBw', root),
      bagStatus: $('#bagStatus', root),
      diagOk: $('#diagOk', root), diagWarn: $('#diagWarn', root), diagError: $('#diagError', root),
      diagList: $('#diagList', root), diagFilter: $('#diagFilter', root), diagRefresh: $('#btnDiagRefresh', root),
      tfFrames: $('#tfFrames', root), tfRoot: $('#tfRoot', root), tfUpdated: $('#tfUpdated', root), tfTree: $('#tfTree', root),
      tfRefresh: $('#btnTfRefresh', root),
    };

    const chart = MiniLine('sparkHz', 90);
    const state = { stream:null, diag:[], filter:'ALL' };

    // 连接状态
    function setStatus(mode){
      els.status.classList.remove('online','connecting','error');
      els.status.classList.add(mode);
      els.status.textContent = mode==='online' ? '● 在线' : (mode==='connecting' ? '● 连接中…' : '● 离线');
    }

    async function loadSummary(){
      setStatus('connecting');
      try{
        const s = await API.getSummary();
        setStatus(s.online? 'online':'error');
        els.bridgeType.textContent = s.bridge || '—';
        els.rosDomain.textContent = s.domain_id ?? '—';
        els.rosHost.textContent = s.host || '—';
        els.rosDiscovery.textContent = 'DDS';

        els.kpiNodes.textContent = s.nodes ?? 0;
        els.kpiTopics.textContent = s.topics ?? 0;
        els.kpiWarn.textContent = s.warn ?? 0;
        els.kpiError.textContent = s.error ?? 0;
        els.kpiHz.textContent = (s.avg_hz ?? 0).toFixed(1);
        els.kpiBw.textContent = fmtKBs(s.avg_bw ?? 0);
        els.bagStatus.textContent = s.bag?.running ? `进行中 · ${(s.bag.size/1024/1024).toFixed(1)} MB` : '未开始';
      }catch(e){
        console.error(e); setStatus('error');
      }
    }

    async function loadDiagnostics(){
      try{
        const rows = await API.getDiagnostics();
        state.diag = rows.slice(0,10);
        renderDiag();
      }catch(e){ console.error(e); }
    }

    function renderDiag(){
      const list = els.diagList; list.innerHTML = '';
      const f = state.filter;
      const rows = state.diag.filter(x=> f==='ALL' || x.lvl===f);
      let ok=0,warn=0,err=0;
      state.diag.forEach(x=>{ if(x.lvl==='OK') ok++; else if(x.lvl==='WARN') warn++; else if(x.lvl==='ERROR') err++; });
      els.diagOk.textContent = ok; els.diagWarn.textContent = warn; els.diagError.textContent = err;

      if(!rows.length){ list.innerHTML = '<div class="placeholder slim">暂无诊断消息。</div>'; return; }
      const frag = document.createDocumentFragment();
      rows.forEach(r=>{
        const div = document.createElement('div');
        div.className = `diag-item ${r.lvl==='WARN'?'warn':(r.lvl==='ERROR'?'error':'')}`;
        div.innerHTML = `
          <div class="lvl">${esc(r.lvl)} · ${timeStr(r.ts)}</div>
          <div><b>${esc(r.src)}</b> — ${esc(r.msg)}</div>
        `;
        frag.appendChild(div);
      });
      list.appendChild(frag);
    }

    async function loadTF(){
      try{
        const t = await API.getTF();
        els.tfFrames.textContent = t.frames ?? 0;
        els.tfRoot.textContent = t.root || '—';
        els.tfUpdated.textContent = timeStr(t.updated);
        renderTfTree(t.tree || {});
      }catch(e){ console.error(e); }
    }
    function renderTfTree(tree){
      const ul = els.tfTree; ul.innerHTML = '';
      const root = Object.keys(tree)[0] || 'map';
      function rec(parent, container, depth){
        const li = document.createElement('li');
        li.innerHTML = `${'&nbsp;'.repeat(depth*2)}<span class="muted">↳</span> <b>${esc(parent)}</b>`;
        container.appendChild(li);
        const children = tree[parent] || [];
        children.forEach(ch=>{
          rec(ch, container, depth+1);
        });
      }
      rec(root, ul, 0);
    }

    function startStream(){
      stopStream();
      state.stream = API.openStream({
        onHzSample: (hz)=> { chart.push(hz); }
      });
    }
    function stopStream(){ try{ state.stream?.close?.(); }catch{} state.stream=null; }

    // 事件
    els.reconnect.addEventListener('click', ()=> { stopStream(); loadAll(); });
    els.diagFilter.addEventListener('change', ()=> { state.filter = els.diagFilter.value; renderDiag(); });
    els.diagRefresh.addEventListener('click', ()=> loadDiagnostics());
    els.tfRefresh.addEventListener('click', ()=> loadTF());

    async function loadAll(){
      await loadSummary();
      await loadDiagnostics();
      await loadTF();
      startStream();
    }

    loadAll();

    // 导出销毁（离开页面）
    this.destroy = ()=> { stopStream(); };
  }

  // ---------- 自动挂载：观察片段注入 ----------
  (function autoMount(){
    const content = document.querySelector('.content-area') || document.querySelector('#content') || document.body;
    let current = null;
    function tryInit(){
      const root = document.getElementById('rosov-root');
      if (root && !root.__mounted){
        current?.destroy?.();
        current = new RosOverview();
        root.__mounted = true;
      }
    }
    tryInit();
    const mo = new MutationObserver(tryInit);
    mo.observe(content, { childList:true, subtree:true });
    window.addEventListener('beforeunload', ()=> current?.destroy?.());
  })();
})();
