/* ros-ops.js —— 采集与配置（ROS 2）
 * 内容：录包控制 + 任务进度 + 参数快照/对比 + QoS 查看（只读）
 * 方式：fragment 注入 + 预加载脚本；内置 Mock 数据；接入后端只需替换 API。
 */
(function(){
  const $ = (s,r=document)=>r.querySelector(s);
  const $$=(s,r=document)=>Array.from(r.querySelectorAll(s));
  const esc = s => String(s??'').replace(/[&<>"]/g,m=>({'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;'}[m]));
  const fmtMB = b => { const m=(b||0)/1024/1024; return m<10? m.toFixed(2): m.toFixed(1); };

  // ---------------- Mock API（接后端：替换为真实 fetch/WebSocket/SSE） ----------------
  const API = (()=>{
    async function listTopics(){
      await delay(120);
      return [
        { name:'/camera/image_raw', type:'sensor_msgs/msg/Image' },
        { name:'/scan', type:'sensor_msgs/msg/LaserScan' },
        { name:'/odom', type:'nav_msgs/msg/Odometry' },
        { name:'/tf', type:'tf2_msgs/msg/TFMessage' },
        { name:'/cmd_vel', type:'geometry_msgs/msg/Twist' },
      ];
    }
    // 开始录制，返回任务 id
    async function startBag({ topics, limitSec, limitMB, outDir }){
      await delay(200);
      return { id: 'bag_' + Math.random().toString(36).slice(2,8), started: Date.now(), topics, limitSec, limitMB, outDir };
    }
    async function stopBag(id){
      await delay(200);
      return { ok:true, id, file: `/data/bags/${id}.db3`, size: (50+Math.random()*300)*1024*1024, dur: Math.floor(10+Math.random()*90) };
    }
    // 进度流（模拟）：每 500ms 推进度
    function openBagProgress(id, { onTick }){
      let pct=0, size=0; const t = setInterval(()=>{
        pct = Math.min(100, pct + (2+Math.random()*5));
        size += (100+Math.random()*500)*1024;
        onTick && onTick({ pct, size });
        if (pct>=100){ clearInterval(t); }
      }, 500);
      return { close(){ clearInterval(t); } };
    }

    async function listQoS(){
      await delay(100);
      return [
        { name:'/scan', rel:'best_effort', dur:'volatile', depth:10, hist:'keep_last' },
        { name:'/odom', rel:'reliable', dur:'volatile', depth:10, hist:'keep_last' },
        { name:'/tf', rel:'reliable', dur:'transient_local', depth:100, hist:'keep_last' },
      ];
    }

    // 参数快照：返回 {node: {param:value,...}, ...}
    async function getParamsSnapshot(){
      await delay(150);
      return {
        '/camera_node': { 'exposure': 0.01, 'gain': 2.0, 'frame_rate': 30 },
        '/lidar': { 'range_min': 0.15, 'range_max': 12.0, 'rpm': 600 },
        '/nav': { 'max_vel_x': 0.5, 'global_frame': 'map' }
      };
    }

    function delay(ms){ return new Promise(r=>setTimeout(r,ms)); }
    return { listTopics, startBag, stopBag, openBagProgress, listQoS, getParamsSnapshot };
  })();

  // ---------------- 主视图 ----------------
  function RosOps(){
    const root = $('#rosops-root'); if(!root) return;

    const els = {
      status: $('#rosops-status', root), refresh: $('#btnOpsRefresh', root),

      // 录包控制
      topicSearch: $('#topicSearch', root), topicList: $('#topicList', root),
      limitSeconds: $('#limitSeconds', root), limitSize: $('#limitSize', root), outputDir: $('#outputDir', root),
      btnStart: $('#btnBagStart', root), btnStop: $('#btnBagStop', root),
      bagProgress: $('#bagProgress', root), bagMeta: $('#bagMeta', root), bagStateBadge: $('#bagStateBadge', root),

      // 任务
      taskList: $('#taskList', root), btnTaskClear: $('#btnTaskClear', root),

      // 参数
      paramSearch: $('#paramSearch', root), btnSnapNow: $('#btnSnapNow', root), btnSnapDiff: $('#btnSnapDiff', root), btnSnapExport: $('#btnSnapExport', root),
      snapList: $('#snapList', root), paramTree: $('#paramTree', root), diffPanel: $('#diffPanel', root), diffList: $('#diffList', root),

      // QoS
      qosSearch: $('#qosSearch', root), qosList: $('#qosList', root),
    };

    const state = {
      topics: [],
      selectedTopics: new Set(),
      bag: { running:false, id:null, started:null, stream:null },
      tasks: [],
      snaps: [],           // [{id, ts, data}]
      snapActive: null,    // 当前查看快照 id
      diffPair: [],        // 存两次快照 id
      qos: [],
    };

    function setStatus(mode){ els.status.className='status-chip ' + (mode||'error'); els.status.textContent = mode==='online'?'● 在线': (mode==='connecting'?'● 连接中…':'● 离线'); }

    // ---------- 初始化加载 ----------
    async function loadAll(){
      setStatus('connecting');
      try{
        state.topics = await API.listTopics();
        renderTopicList();
        state.qos = await API.listQoS();
        renderQoS();
        setStatus('online');
      }catch(e){ console.error(e); setStatus('error'); }
    }

    // ---------- 录包控制 ----------
    function renderTopicList(){
      const kw = els.topicSearch.value.trim().toLowerCase();
      const rows = state.topics.filter(t=> t.name.toLowerCase().includes(kw) || t.type.toLowerCase().includes(kw));
      els.topicList.innerHTML = '';
      if(!rows.length){ els.topicList.innerHTML = '<div class="placeholder slim">无匹配话题。</div>'; return; }
      const frag = document.createDocumentFragment();
      rows.forEach(t=>{
        const div = document.createElement('div'); div.className='topic-item';
        const id = 'chk_' + btoa(t.name).replace(/=/g,'');
        div.innerHTML = `
          <input id="${id}" type="checkbox" ${state.selectedTopics.has(t.name)?'checked':''}>
          <label for="${id}"><b>${esc(t.name)}</b></label>
          <div class="topic-meta"> · ${esc(t.type)}</div>
        `;
        div.querySelector('input').addEventListener('change', (e)=>{
          if (e.target.checked) state.selectedTopics.add(t.name);
          else state.selectedTopics.delete(t.name);
        });
        frag.appendChild(div);
      });
      els.topicList.appendChild(frag);
    }

    els.topicSearch.addEventListener('input', ()=> renderTopicList());

    els.btnStart.addEventListener('click', async ()=>{
      if(state.bag.running){ return; }
      if(state.selectedTopics.size===0){ alert('请至少选择一个话题'); return; }
      const topics = Array.from(state.selectedTopics);
      const limitSec = Math.max(1, parseInt(els.limitSeconds.value||'60',10));
      const limitMB  = Math.max(1, parseInt(els.limitSize.value||'256',10));
      const outDir   = els.outputDir.value.trim() || '/data/bags';
      try{
        const task = await API.startBag({ topics, limitSec, limitMB, outDir });
        state.bag = { running:true, id:task.id, started:task.started, stream:null };
        els.btnStart.disabled = true; els.btnStop.disabled = false;
        els.bagStateBadge.textContent = '进行中';
        els.bagMeta.textContent = `话题 ${topics.length} · 限时 ${limitSec}s · 限 ${limitMB}MB · 输出 ${outDir}`;
        startProgress(task.id);
        // 记录任务
        addTask({ name:`录包 ${task.id}`, meta:`${topics.length} 个话题 → ${outDir}`, progress:0, id:task.id });
      }catch(e){ alert('启动失败'); }
    });

    els.btnStop.addEventListener('click', async ()=>{
      if(!state.bag.running) return;
      try{
        const r = await API.stopBag(state.bag.id);
        stopProgress();
        els.btnStart.disabled = false; els.btnStop.disabled = true;
        els.bagStateBadge.textContent = '未开始';
        els.bagMeta.textContent = `完成：${r.file} · ${fmtMB(r.size)} MB · ${r.dur}s`;
        // 完成任务
        finishTask(state.bag.id, true, `${fmtMB(r.size)}MB / ${r.dur}s`, r.file);
        state.bag = { running:false, id:null, started:null, stream:null };
      }catch(e){ alert('停止失败'); }
    });

    function startProgress(id){
      stopProgress();
      const start = Date.now();
      state.bag.stream = API.openBagProgress(id, {
        onTick: s=>{
          els.bagProgress.style.width = Math.max(0, Math.min(100, s.pct)) + '%';
          const dur = Math.max(1, Math.floor((Date.now()-start)/1000));
          els.bagMeta.textContent = `进行中：${s.pct.toFixed(0)}% · ${fmtMB(s.size)} MB · ${dur}s`;
          updateTask(id, s.pct);
        }
      });
    }
    function stopProgress(){ try{ state.bag.stream?.close?.(); }catch{} state.bag.stream=null; els.bagProgress.style.width='0%'; }

    // 任务面板
    function addTask({ name, meta, progress, id }){
      const el = document.createElement('div'); el.className='task'; el.dataset.id=id;
      el.innerHTML = `
        <div>
          <div class="name">${esc(name)}</div>
          <div class="meta">${esc(meta||'')}</div>
        </div>
        <div class="progress">
          <div class="progress-bar small"><span style="width:${progress||0}%"></span></div>
        </div>
        <div class="act muted">进行中</div>
      `;
      els.taskList.querySelector('.placeholder')?.remove();
      els.taskList.appendChild(el);
    }
    function updateTask(id, pct){
      const el = els.taskList.querySelector(`.task[data-id="${CSS.escape(id)}"]`); if(!el) return;
      el.querySelector('.progress-bar > span').style.width = Math.max(0,Math.min(100,pct)) + '%';
    }
    function finishTask(id, ok, msg, file){
      const el = els.taskList.querySelector(`.task[data-id="${CSS.escape(id)}"]`); if(!el) return;
      el.classList.add(ok?'ok':'fail');
      const act = el.querySelector('.act');
      act.textContent = ok? '完成' : (msg || '失败');
      if (ok && file){
        const a = document.createElement('a'); a.href='#'; a.textContent='定位到存储页'; a.className='link';
        a.addEventListener('click', ()=> alert(`在“存储管理”中定位：${file}`));
        act.appendChild(document.createTextNode(' · '));
        act.appendChild(a);
      }
    }
    els.btnTaskClear.addEventListener('click', ()=>{
      $$('.task.ok', els.taskList).forEach(x=>x.remove());
      if (!els.taskList.children.length) els.taskList.innerHTML = '<div class="placeholder slim">暂无任务。</div>';
    });

    // ---------- 参数快照 ----------
    els.btnSnapNow.addEventListener('click', async ()=>{
      try{
        const data = await API.getParamsSnapshot();
        const snap = { id: 'snap_' + Math.random().toString(36).slice(2,8), ts: Date.now(), data };
        state.snaps.unshift(snap);
        renderSnapList();
      }catch(e){ alert('获取参数失败'); }
    });

    function renderSnapList(){
      els.snapList.innerHTML = '';
      if(!state.snaps.length){ els.snapList.innerHTML = '<li class="muted">暂无快照</li>'; return; }
      const frag = document.createDocumentFragment();
      state.snaps.forEach(s=>{
        const li = document.createElement('li'); li.dataset.id=s.id; li.innerHTML=`${new Date(s.ts).toLocaleString()} <span class="muted">(${Object.keys(s.data||{}).length} 个节点)</span>`;
        li.addEventListener('click', ()=>{
          state.snapActive = s.id;
          $$('#snapList li', root).forEach(x=>x.classList.toggle('active', x.dataset.id===s.id));
          renderParamTree(s.data);
          // 维护“对比”选择队列（最多两条）
          manageDiffSelection(s.id);
        });
        frag.appendChild(li);
      });
      els.snapList.appendChild(frag);
    }

    function manageDiffSelection(id){
      const arr = state.diffPair;
      if (arr.includes(id)) return;
      arr.push(id);
      if (arr.length>2) arr.shift();
    }

    function renderParamTree(data){
      const kw = els.paramSearch.value.trim().toLowerCase();
      const box = els.paramTree; box.innerHTML='';
      if(!data || !Object.keys(data).length){ box.innerHTML = '<div class="placeholder slim">空快照。</div>'; return; }
      const frag = document.createDocumentFragment();
      Object.entries(data).forEach(([node, params])=>{
        if (kw && !node.toLowerCase().includes(kw)) return;
        const keys = Object.keys(params||{}).filter(k=> !kw || k.toLowerCase().includes(kw)).sort();
        if (!keys.length && kw) return;
        const h = document.createElement('div'); h.className='param-row'; h.innerHTML = `<b>${esc(node)}</b>`;
        frag.appendChild(h);
        keys.forEach(k=>{
          const v = params[k];
          const row = document.createElement('div'); row.className='param-row';
          row.innerHTML = `<span class="param-key">${esc(k)}</span> = <span class="param-val">${esc(JSON.stringify(v))}</span>`;
          frag.appendChild(row);
        });
      });
      if (!frag.children.length) box.innerHTML = '<div class="placeholder slim">无匹配结果。</div>';
      else box.appendChild(frag);
    }
    els.paramSearch.addEventListener('input', ()=> {
      const s = state.snaps.find(x=>x.id===state.snapActive);
      if (s) renderParamTree(s.data);
    });

    // 导出快照 JSON
    els.btnSnapExport.addEventListener('click', ()=>{
      const s = state.snaps.find(x=>x.id===state.snapActive);
      if (!s){ alert('请先选择一个快照'); return; }
      const blob = new Blob([JSON.stringify(s.data, null, 2)], {type:'application/json'});
      const a = document.createElement('a'); a.href = URL.createObjectURL(blob); a.download = `params_${s.id}.json`; document.body.appendChild(a); a.click(); a.remove();
    });

    // 快照对比（两次）
    els.btnSnapDiff.addEventListener('click', ()=>{
      if (state.diffPair.length<2){ alert('请选择两次快照（在列表中点击两条）'); return; }
      const a = state.snaps.find(x=>x.id===state.diffPair[0]);
      const b = state.snaps.find(x=>x.id===state.diffPair[1]);
      const diffs = diffParams(a?.data||{}, b?.data||{});
      renderDiff(diffs, a, b);
    });

    function diffParams(a, b){
      // 结构：node.param -> {type:'add|del|mod', old, cur}
      const out = [];
      const nodes = new Set([...Object.keys(a), ...Object.keys(b)]);
      nodes.forEach(n=>{
        const pa = a[n]||{}, pb = b[n]||{};
        const keys = new Set([...Object.keys(pa), ...Object.keys(pb)]);
        keys.forEach(k=>{
          const va = JSON.stringify(pa[k]); const vb = JSON.stringify(pb[k]);
          if (va === undefined && vb !== undefined) out.push({ path:`${n}:${k}`, type:'add', cur:pb[k] });
          else if (va !== undefined && vb === undefined) out.push({ path:`${n}:${k}`, type:'del', old:pa[k] });
          else if (va !== vb) out.push({ path:`${n}:${k}`, type:'mod', old:pa[k], cur:pb[k] });
        });
      });
      return out;
    }
    function renderDiff(diffs, a, b){
      els.diffPanel.classList.remove('hidden');
      els.diffList.innerHTML = '';
      if (!diffs.length){ els.diffList.innerHTML = '<div class="placeholder slim">两次快照无差异。</div>'; return; }
      const frag = document.createDocumentFragment();
      diffs.forEach(d=>{
        const div = document.createElement('div'); div.className='diff-item ' + (d.type==='add'?'diff-add': d.type==='del'?'diff-del':'diff-mod');
        const body = d.type==='mod'
          ? `<b>${esc(d.path)}</b>：<span class="muted">旧</span> ${esc(JSON.stringify(d.old))} <span class="muted">→ 新</span> ${esc(JSON.stringify(d.cur))}`
          : (d.type==='add'
              ? `<b>${esc(d.path)}</b>：新增 ${esc(JSON.stringify(d.cur))}`
              : `<b>${esc(d.path)}</b>：删除 ${esc(JSON.stringify(d.old))}`);
        div.innerHTML = body;
        frag.appendChild(div);
      });
      els.diffList.appendChild(frag);
    }

    // ---------- QoS 只读 ----------
    function renderQoS(){
      const kw = els.qosSearch.value.trim().toLowerCase();
      const rows = state.qos.filter(q=> q.name.toLowerCase().includes(kw));
      els.qosList.innerHTML = '';
      if(!rows.length){ els.qosList.innerHTML = '<div class="placeholder slim">无匹配话题。</div>'; return; }
      const frag = document.createDocumentFragment();
      rows.forEach(q=>{
        const d = document.createElement('div'); d.className='qos-item';
        d.innerHTML = `
          <div class="name"><b>${esc(q.name)}</b></div>
          <div class="meta">深度：${q.depth} · 历史：${esc(q.hist)}</div>
          <div class="meta">
            <span class="qos-tag">${esc(q.rel)}</span>
            <span class="qos-tag">${esc(q.dur)}</span>
          </div>
        `;
        frag.appendChild(d);
      });
      els.qosList.appendChild(frag);
    }
    els.qosSearch.addEventListener('input', ()=> renderQoS());

    // ---------- 刷新 ----------
    els.refresh.addEventListener('click', ()=> loadAll());

    // ---------- 启动 ----------
    loadAll();

    // 导出销毁
    this.destroy = ()=> { try{ state.bag.stream?.close?.(); }catch{} };
  }

  // ---------------- 工具 ----------------
  function delay(ms){ return new Promise(r=>setTimeout(r,ms)); }

  // ---------------- 自动挂载 ----------------
  (function autoMount(){
    const content = document.querySelector('.content-area') || document.querySelector('#content') || document.body;
    let current = null;
    function tryInit(){
      const root = document.getElementById('rosops-root');
      if (root && !root.__mounted){
        current?.destroy?.();
        current = new RosOps();
        root.__mounted = true;
      }
    }
    tryInit();
    const mo = new MutationObserver(tryInit);
    mo.observe(content, { childList:true, subtree:true });
    window.addEventListener('beforeunload', ()=> current?.destroy?.());
  })();
})();
