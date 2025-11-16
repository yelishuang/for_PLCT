/* ros-comm.js —— 通信监控（ROS 2）
 * 特点：话题/节点合页、行内微型折线（Hz/Bw）、消息预览抽屉、邻接小图（简化）
 * 适配：fragment 注入 + 预加载脚本（MutationObserver 自动初始化）
 */
(function(){
  const $ = (s,r=document)=>r.querySelector(s);
  const $$=(s,r=document)=>Array.from(r.querySelectorAll(s));
  const esc = s => String(s??'').replace(/[&<>"]/g,m=>({'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;'}[m]));
  const fmtKBs = n => { const v=(n||0)/1024; return v<10? v.toFixed(2): v.toFixed(1); };
  const fmtPct = n => (n==null? '—' : (Math.round(n*10)/10)+'%');

  // ---------- 行内小图（canvas） ----------
  function Spark(el, max=50, color='#4a90e2'){
    const c = document.createElement('canvas'); c.className='row-chart'; el.appendChild(c);
    const ctx = c.getContext('2d'); const data=[];
    function resize(){
      const dpr=Math.max(1,window.devicePixelRatio||1);
      const w=c.clientWidth||110, h=c.clientHeight||34;
      c.width = Math.floor(w*dpr); c.height=Math.floor(h*dpr); ctx.setTransform(dpr,0,0,dpr,0,0); draw();
    }
    function push(v){ data.push(v||0); if(data.length>max) data.shift(); draw(); }
    function draw(){
      const w=c.clientWidth||110, h=c.clientHeight||34;
      ctx.clearRect(0,0,w,h);
      ctx.strokeStyle="#263043"; ctx.lineWidth=1;
      ctx.beginPath(); ctx.moveTo(0, Math.round(h/2)); ctx.lineTo(w, Math.round(h/2)); ctx.stroke();
      const maxVal=Math.max(1, ...data);
      const step=w/Math.max(1,(max-1));
      ctx.strokeStyle=color; ctx.lineWidth=2; ctx.beginPath();
      for(let i=0;i<data.length;i++){ const x=i*step; const y=h - (data[i]/maxVal)*(h-4); if(i===0) ctx.moveTo(x,y); else ctx.lineTo(x,y); }
      ctx.stroke();
    }
    window.addEventListener('resize', resize); setTimeout(resize,0);
    return { push };
  }

  // ---------- 语法着色（JSON 简易高亮） ----------
  function syntaxJSON(obj){
    const json = typeof obj==='string'? obj : JSON.stringify(obj, null, 2);
    return esc(json)
      .replace(/"(\\u[\da-fA-F]{4}|\\[^u]|[^"\\])*"(?=\s*:)/g, m=>`<span class="key">${m}</span>`)
      .replace(/"(\\u[\da-fA-F]{4}|\\[^u]|[^"\\])*"/g, m=>`<span class="str">${m}</span>`)
      .replace(/\b-?\d+(\.\d+)?\b/g, m=>`<span class="num">${m}</span>`)
      .replace(/\b(true|false)\b/g, m=>`<span class="bool">${m}</span>`)
      .replace(/\b(null)\b/g, m=>`<span class="nil">${m}</span>`);
  }

  // ---------- Mock API（接后端：换成 fetch/WebSocket） ----------
  const API = (()=>{
    async function listTopics(){
      // 示例：const r = await fetch('/api/ros/topics'); return r.json();
      await delay(180);
      const rows = [
        { name:'/camera/image_raw', type:'sensor_msgs/msg/Image', hz:15.0, bw:800*1024, pubs:['/camera_node'], subs:['/image_proc'], qos:{reliability:'reliable', durability:'volatile'} },
        { name:'/scan', type:'sensor_msgs/msg/LaserScan', hz:10.0, bw:220*1024, pubs:['/lidar'], subs:['/slam','/obstacle'], qos:{reliability:'best_effort', durability:'volatile'} },
        { name:'/tf', type:'tf2_msgs/msg/TFMessage', hz:30.0, bw:60*1024, pubs:['/robot_state'], subs:['/nav'], qos:{reliability:'reliable', durability:'transient_local'} },
        { name:'/odom', type:'nav_msgs/msg/Odometry', hz:30.0, bw:48*1024, pubs:['/ekf'], subs:['/nav'], qos:{reliability:'reliable', durability:'volatile'} },
      ];
      return rows;
    }
    function openTopicStream(name, { onHz, onBw, onMsg }){
      // 模拟：每 1s 推一条频率/带宽；每 2s 推一条样本
      const t1 = setInterval(()=>{ onHz && onHz( 5 + Math.random()*30 ); onBw && onBw( (20+Math.random()*900)*1024 ); }, 1000);
      const t2 = setInterval(()=>{ onMsg && onMsg({ ts: Date.now(), data: { topic:name, seq: Math.floor(Math.random()*1e6), value: Math.random().toFixed(3) } }); }, 2000);
      return { close(){ clearInterval(t1); clearInterval(t2); } };
    }

    async function listNodes(){
      await delay(160);
      return [
        { name:'/camera_node', ns:'/', pubs:1, subs:0, cpu:3.2, mem:1.4, errs:0, up:['/camera/image_raw'], down:[] },
        { name:'/image_proc', ns:'/', pubs:0, subs:1, cpu:6.8, mem:2.3, errs:1, up:['/camera/image_raw'], down:['/image_rect'] },
        { name:'/lidar', ns:'/', pubs:1, subs:0, cpu:4.0, mem:1.1, errs:0, up:['/scan'], down:[] },
        { name:'/slam', ns:'/', pubs:1, subs:1, cpu:12.1, mem:6.5, errs:0, up:['/scan','/odom'], down:['/map'] },
        { name:'/nav', ns:'/', pubs:0, subs:2, cpu:9.4, mem:3.2, errs:0, up:['/tf','/odom'], down:[] },
      ];
    }

    function delay(ms){ return new Promise(r=>setTimeout(r,ms)); }
    return { listTopics, openTopicStream, listNodes };
  })();

  // ---------- 主视图 ----------
  function RosComm(){
    const root = $('#roscm-root'); if(!root) return;

    const els = {
      status: $('#roscm-status', root),
      refresh: $('#btnCommRefresh', root),
      tabs: $$('.tab', root),
      search: $('#commSearch', root),
      panelTopics: $('#panelTopics', root), panelNodes: $('#panelNodes', root),
      topicRows: $('#topicRows', root), topicEmpty: $('#topicEmpty', root),
      nodeRows: $('#nodeRows', root), nodeEmpty: $('#nodeEmpty', root),
      drawerTitle: $('#drawerTitle', root), drawerBody: $('#drawerBody', root), drawerActions: $('#drawerActions', root),
    };

    const state = {
      tab: 'topics',
      topics: [],
      nodes: [],
      topicStreams: new Map(),  // topic -> {sparkHz, sparkBw, close()}
      preview: { topic:null, msgs:[], limit:20, stream:null },
      selectedNode: null,
    };

    function setStatus(mode){ els.status.className='status-chip ' + (mode||'error'); els.status.textContent = mode==='online'?'● 在线': (mode==='connecting'?'● 连接中…':'● 离线'); }

    // 切换标签
    els.tabs.forEach(b=>{
      b.addEventListener('click', ()=>{
        els.tabs.forEach(x=>x.classList.remove('active')); b.classList.add('active');
        state.tab = b.dataset.tab;
        renderTab();
      });
    });

    function renderTab(){
      const t = state.tab;
      els.panelTopics.classList.toggle('hidden', t!=='topics');
      els.panelNodes.classList.toggle('hidden', t!=='nodes');
      els.drawerTitle.textContent = '详情';
      els.drawerBody.innerHTML = '<div class="placeholder slim">请选择左侧列表的一项…</div>';
      els.drawerActions.innerHTML = '';
    }

    // ---------- 加载话题 ----------
    async function loadTopics(){
      setStatus('connecting');
      try{
        const all = await API.listTopics();
        state.topics = all;
        renderTopics();
        setStatus('online');
      }catch(e){ console.error(e); setStatus('error'); }
    }

    function renderTopics(){
      // 清理旧行的流
      state.topicStreams.forEach(s=> s.close && s.close()); state.topicStreams.clear();

      const kw = els.search.value.trim().toLowerCase();
      const rows = state.topics.filter(r=>
        r.name.toLowerCase().includes(kw) ||
        r.type.toLowerCase().includes(kw) ||
        (r.pubs||[]).some(x=>x.toLowerCase().includes(kw)) ||
        (r.subs||[]).some(x=>x.toLowerCase().includes(kw))
      );

      els.topicRows.innerHTML = '';
      if(!rows.length){ els.topicEmpty.classList.remove('hidden'); return; }
      els.topicEmpty.classList.add('hidden');

      const frag = document.createDocumentFragment();
      rows.forEach(row=>{
        const tr = document.createElement('tr'); tr.dataset.topic=row.name;
        const hzCell = document.createElement('td'); hzCell.className='t-center';
        const bwCell = document.createElement('td'); bwCell.className='t-center';
        const hzSpark = Spark(hzCell, 60, '#60a5fa');
        const bwSpark = Spark(bwCell, 60, '#a3e635');

        // 开流（Mock：每秒推 hz/bw）
        const stream = API.openTopicStream(row.name, {
          onHz: v=> { hzSpark.push(v); },
          onBw: v=> { bwSpark.push(v/1024); },
          onMsg: m=> { if(state.preview.topic===row.name) pushPreview(m); }
        });
        state.topicStreams.set(row.name, { close: ()=>stream.close() });

        tr.innerHTML = `
          <td><b>${esc(row.name)}</b></td>
          <td>${esc(row.type)}</td>
          <td>${(row.pubs?.length||0)} / ${(row.subs?.length||0)}</td>
        `;
        tr.appendChild(hzCell);
        tr.appendChild(bwCell);
        const qosTd = document.createElement('td'); qosTd.className='t-center';
        qosTd.innerHTML = `<div class="qos">
          <span class="tag">${esc(row.qos?.reliability||'—')}</span>
          <span class="tag">${esc(row.qos?.durability||'—')}</span>
        </div>`;
        const ctrlTd = document.createElement('td'); ctrlTd.className='row-ctrl';
        ctrlTd.innerHTML = `<button class="btn small" data-act="preview">预览</button>`;
        tr.appendChild(qosTd); tr.appendChild(ctrlTd);

        // 事件
        ctrlTd.querySelector('[data-act="preview"]').addEventListener('click', ()=> openPreview(row));
        frag.appendChild(tr);
      });
      els.topicRows.appendChild(frag);
    }

    // ---------- 消息预览抽屉 ----------
    function openPreview(row){
      state.preview.topic = row.name; state.preview.msgs = [];
      els.drawerTitle.textContent = `消息预览 · ${row.name}`;
      els.drawerActions.innerHTML = `
        <span class="preview-meta">类型：${esc(row.type)} · 最近 ${state.preview.limit} 条</span>
        <button id="btnPrevStop" class="btn small danger">停止预览</button>
        <button id="btnPrevClear" class="btn small ghost">清空</button>
      `;
      els.drawerBody.innerHTML = `<div id="msgList"></div>`;
      $('#btnPrevStop', root).addEventListener('click', ()=> stopPreview());
      $('#btnPrevClear', root).addEventListener('click', ()=> renderPreview());
    }
    function stopPreview(){ state.preview.topic=null; els.drawerActions.innerHTML=''; }
    function pushPreview(m){
      state.preview.msgs.push(m);
      if(state.preview.msgs.length > state.preview.limit) state.preview.msgs.shift();
      renderPreview();
    }
    function renderPreview(){
      const box = $('#msgList', root); if(!box){ return; }
      box.innerHTML = '';
      const rows = state.preview.msgs.slice().reverse();
      if(!rows.length){ box.innerHTML = '<div class="placeholder slim">暂无数据…</div>'; return; }
      const frag = document.createDocumentFragment();
      rows.forEach(x=>{
        const div = document.createElement('div'); div.className='msg';
        div.innerHTML = `<div class="ts">${new Date(x.ts).toLocaleTimeString()}</div>
                         <pre class="json">${syntaxJSON(x.data)}</pre>`;
        frag.appendChild(div);
      });
      box.appendChild(frag);
    }

    // ---------- 加载节点 ----------
    async function loadNodes(){
      setStatus('connecting');
      try{
        const all = await API.listNodes();
        state.nodes = all;
        renderNodes();
        setStatus('online');
      }catch(e){ console.error(e); setStatus('error'); }
    }

    function renderNodes(){
      const kw = els.search.value.trim().toLowerCase();
      const rows = state.nodes.filter(n=>
        n.name.toLowerCase().includes(kw) ||
        n.ns.toLowerCase().includes(kw)
      );

      els.nodeRows.innerHTML = '';
      if(!rows.length){ els.nodeEmpty.classList.remove('hidden'); return; }
      els.nodeEmpty.classList.add('hidden');

      const frag = document.createDocumentFragment();
      rows.forEach(n=>{
        const tr = document.createElement('tr');
        tr.innerHTML = `
          <td><b>${esc(n.name)}</b></td>
          <td>${esc(n.ns||'/')}</td>
          <td>${n.pubs||0} / ${n.subs||0}</td>
          <td class="t-center">${fmtPct(n.cpu)}</td>
          <td class="t-center">${fmtPct(n.mem)}</td>
          <td class="t-center">${n.errs? `<span class="badge danger">${n.errs}</span>`:'<span class="badge ok">0</span>'}</td>
          <td class="t-center"><button class="btn small" data-act="adj">查看</button></td>
        `;
        tr.querySelector('[data-act="adj"]').addEventListener('click', ()=> openAdj(n));
        frag.appendChild(tr);
      });
      els.nodeRows.appendChild(frag);
    }

    // ---------- 邻接图（简化：绝对定位 + 直线） ----------
    function openAdj(n){
      state.selectedNode = n.name;
      els.drawerTitle.textContent = `邻接 · ${n.name}`;
      els.drawerActions.innerHTML = `
        <button id="btnNodeLogs" class="btn small">查看相关日志</button>
        <button id="btnNodeOps" class="btn small ghost">运行中心</button>
      `;
      $('#btnNodeLogs', root).addEventListener('click', ()=> alert('跳转日志（集成时通过查询参数过滤该节点相关）'));
      $('#btnNodeOps', root).addEventListener('click', ()=> alert('跳转运行中心定位进程/服务（集成时实现）'));

      els.drawerBody.innerHTML = `
        <div class="graph-tip">展示与该节点直接相连的上下游关系（1 层）。</div>
        <div class="graph-wrap">
          <div id="graph" class="graph"></div>
        </div>
      `;
      drawGraph(n);
    }

    function drawGraph(n){
      const g = $('#graph', root); if(!g) return; g.innerHTML='';
      const center = makeNodeChip(n.name, 280, 130); g.appendChild(center);

      // 上游在左、下游在右，纵向分布
      const up = n.up||[], down = n.down||[];
      up.forEach((t, i)=> {
        const y = 30 + i*50; const chip = makeNodeChip(t, 40, y); g.appendChild(chip);
        g.appendChild(makeEdge(40+ chip.offsetWidth, y+14, 280, 140)); // 画到中心
      });
      down.forEach((t, i)=> {
        const y = 30 + i*50; const chip = makeNodeChip(t, 520, y); g.appendChild(chip);
        g.appendChild(makeEdge(280+center.offsetWidth, 140, 520, y+14));
      });
    }

    function makeNodeChip(text, x, y){
      const d = document.createElement('div');
      d.className='node-chip'; d.style.left=x+'px'; d.style.top=y+'px'; d.textContent=text;
      return d;
    }
    function makeEdge(x1,y1,x2,y2){
      const d = document.createElement('div'); d.className='edge';
      const dx=x2-x1, dy=y2-y1; const len=Math.sqrt(dx*dx+dy*dy);
      const ang=Math.atan2(dy,dx)*180/Math.PI;
      d.style.left = x1+'px'; d.style.top=y1+'px'; d.style.width=len+'px'; d.style.transform=`rotate(${ang}deg)`;
      return d;
    }

    // ---------- 搜索/刷新 ----------
    let searchTimer=null;
    els.search.addEventListener('input', ()=>{
      clearTimeout(searchTimer);
      searchTimer=setTimeout(()=> { state.tab==='topics'? renderTopics(): renderNodes(); }, 200);
    });

    els.refresh.addEventListener('click', ()=> { reloadAll(); });

    async function reloadAll(){
      if(state.tab==='topics'){ await loadTopics(); }
      else { await loadNodes(); }
    }

    // ---------- 首次加载 ----------
    renderTab();
    loadTopics(); // 默认停留在“话题”页

    // 导出销毁
    this.destroy = ()=> {
      state.topicStreams.forEach(s=> s.close && s.close());
      state.topicStreams.clear();
    };
  }

  // ---------- 自动挂载 ----------
  (function autoMount(){
    const content = document.querySelector('.content-area') || document.querySelector('#content') || document.body;
    let current = null;
    function tryInit(){
      const root = document.getElementById('roscm-root');
      if (root && !root.__mounted){
        current?.destroy?.();
        current = new RosComm();
        root.__mounted = true;
      }
    }
    tryInit();
    const mo = new MutationObserver(tryInit);
    mo.observe(content, { childList:true, subtree:true });
    window.addEventListener('beforeunload', ()=> current?.destroy?.());
  })();
})();
