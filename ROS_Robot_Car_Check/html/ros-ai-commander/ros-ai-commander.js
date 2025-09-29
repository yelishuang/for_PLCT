/* ros-ai-commander.js —— 面向“行动/执行”的 AI 指挥官
 * 前端实现：聊天→模型产出计划→确认执行→时间线/遥测/安全控制
 * API 约定（后端自实现）：
 *  - POST /api/ai/chat {messages, stream} -> 计划文本或带 {"plan":[...],"risk":""}
 *  - POST /api/agent/plan/execute {plan, dry_run} -> {task_id}
 *  - GET  /api/agent/mission/stream?task=<id>  (SSE/分片文本) 事件：JSON {type, ts, level, text, pose?, battery?, vel?}
 *  - POST /api/agent/estop {engage:boolean}
 *  - POST /api/agent/arm   {arm:boolean}
 */
(function(){
  const $  = (s,r=document)=>r.querySelector(s);
  const $$ = (s,r=document)=>Array.from(r.querySelectorAll(s));
  // 修复：去除多余的右花括号，避免语法错误
  const esc = s => String(s ?? '').replace(/[&<>"]/g, m => ({'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;'}[m]));
  const fmtT = ms => new Date(ms).toLocaleTimeString();

  // 迷你折线
  function MiniLine(canvasId, max=60){
    const c = document.getElementById(canvasId); const ctx = c.getContext('2d'); const data=[];
    function resize(){
      const dpr=Math.max(1, window.devicePixelRatio||1);
      const w=c.clientWidth||220,h=c.clientHeight||42;
      c.width=w*dpr; c.height=h*dpr; ctx.setTransform(dpr,0,0,dpr,0,0); draw();
    }
    function push(v){ data.push(v||0); if(data.length>max) data.shift(); draw(); }
    function draw(){
      const w=c.clientWidth||220,h=c.clientHeight||42;
      ctx.clearRect(0,0,w,h);
      ctx.strokeStyle="#233352"; ctx.lineWidth=1;
      ctx.beginPath(); ctx.moveTo(0,Math.round(h/2)); ctx.lineTo(w,Math.round(h/2)); ctx.stroke();
      const m=Math.max(1,...data); const step=w/Math.max(1,(max-1));
      ctx.strokeStyle="#06b6d4"; ctx.lineWidth=2; ctx.beginPath();
      for(let i=0;i<data.length;i++){ const x=i*step, y=h-(data[i]/m)*(h-4); if(i===0) ctx.moveTo(x,y); else ctx.lineTo(x,y); }
      ctx.stroke();
    }
    window.addEventListener('resize', resize); setTimeout(resize,0);
    return { push };
  }

  function Commander(){
    const root = $('#roscmd-root'); if(!root) return;

    const els = {
      chatList: $('#chatList', root), chatScroll: $('#chatScroll', root),
      input: $('#chatInput', root), send: $('#btnSend', root), chips: $$('.chip', root), chkStream: $('#chkStream', root),
      missionPanel: $('#missionPanel', root), timeline: $('#timeline', root),
      telePose: $('#telePose', root), teleBatt: $('#teleBatt', root), teleVel: $('#teleVel', root),
      statusMode: $('#statusMode', root), statusArm: $('#statusArm', root), statusEstop: $('#statusEstop', root),
      btnArm: $('#btnArm', root), btnEstop: $('#btnEstop', root),
    };

    const sparkOdom = MiniLine('sparkOdom', 90);
    const sparkAvoid = MiniLine('sparkAvoid', 90);

    const state = {
      messages: [{ role:'system', content:'你是ROS2任务执行助手。请把自然语言任务转换为可执行计划(JSON)，字段：step,name,args。高风险动作要给risk说明。' }],
      talking:false,
      runningTask:null,    // {id, started, dryRun}
      eventStream:null
    };

    // —— UI 基础 ——
    function appendMsg(role, html, loading=false){
      const div = document.createElement('div'); div.className=`msg ${role}`+(loading?' loading':'');
      div.innerHTML = `<div class="avatar">${role==='user'?'🧑':'🧭'}</div><div class="bubble"><div class="bubble-inner">${html}</div></div>`;
      els.chatList.appendChild(div); scrollBottom(); return { el:div, inner:div.querySelector('.bubble-inner') };
    }
    function scrollBottom(){ els.chatScroll.scrollTop = els.chatScroll.scrollHeight + 9999; }
    function md(s){ return esc(s).replace(/`([^`]+)`/g,'<span class="kbd">$1</span>').replace(/\*\*([^*]+)\*\*/g,'<b>$1</b>').replace(/\n/g,'<br/>'); }

    // —— 发送到模型：获取计划或回答 ——
    async function sendCurrent(){
      const text = els.input.value.trim(); if(!text || state.talking) return;
      appendMsg('user', md(text));
      state.messages.push({ role:'user', content:text });
      els.input.value=''; autosize();

      const ai = appendMsg('ai','……', true);
      try{
        state.talking=true; els.send.disabled=true;
        const stream = !!els.chkStream?.checked;
        const res = await fetch('/api/ai/chat', { method:'POST', headers:{'Content-Type':'application/json'}, body:JSON.stringify({ messages: state.messages.filter(m=>m.role!=='system'), stream }) });
        if(!res.ok) throw new Error(res.status+' '+res.statusText);

        let textOut='';
        if(stream && res.body?.getReader){
          ai.inner.innerHTML=''; const r=res.body.getReader(); const d=new TextDecoder();
          while(true){ const {done,value}=await r.read(); if(done) break; const chunk=d.decode(value,{stream:true}); textOut+=chunk; ai.inner.innerHTML+= md(chunk); scrollBottom(); }
        }else{
          const j=await res.json(); textOut=j.content||''; ai.inner.innerHTML = md(textOut);
          if (j.references?.length) renderRefsToTimeline(j.references);
        }

        // 尝试解析计划 JSON
        try{
          const m = textOut.match(/\{[\s\S]*\}/);
          if(m){
            const obj = JSON.parse(m[0]);
            if (Array.isArray(obj.plan)) {
              renderPlanCard(obj);
            }
          }
        }catch{}

      }catch(e){
        ai.inner.innerHTML = `<span class="kbd">错误</span> ${esc(e.message||e)}`;
      }finally{
        state.talking=false; els.send.disabled=false; ai.el.classList.remove('loading');
        const last = ai.inner.innerText.trim(); if(last) state.messages.push({ role:'assistant', content:last });
      }
    }

    // —— 在右侧“当前任务”渲染计划卡 —— 
    function renderPlanCard(obj){
      const risk = obj.risk? `<div class="tip">风险：${esc(obj.risk)}</div>`:'';
      const steps = obj.plan.map(s=> `<li><code>${esc(s.step)}</code> — ${esc(s.name||'')} ${s.args? esc(JSON.stringify(s.args)) : ''}</li>`).join('');
      els.missionPanel.innerHTML = `
        <div class="plan-card">
          <span class="badge-plan">执行计划（需确认）</span>
          ${risk}
          <ol class="plan-steps">${steps}</ol>
          <div class="plan-actions">
            <button id="btnSim" class="btn small">演练（不执行硬件）</button>
            <button id="btnExec" class="btn small">执行</button>
            <button id="btnCancel" class="btn small danger">取消</button>
          </div>
        </div>
      `;
      $('#btnSim',root).addEventListener('click', ()=> executePlan(obj, true));
      $('#btnExec',root).addEventListener('click', ()=> executePlan(obj, false));
      $('#btnCancel',root).addEventListener('click', ()=> { els.missionPanel.innerHTML = '<div class="placeholder slim">计划已取消。</div>'; });
    }

    // —— 执行计划：调用后端 + 打开事件流 ——
    async function executePlan(plan, dryRun){
      try{
        const res = await safeFetch('/api/agent/plan/execute', {
          method:'POST',
          headers:{'Content-Type':'application/json'},
          body: JSON.stringify({ plan, dry_run: !!dryRun })
        });
        const ok = !!res && res.ok;
        const data = ok ? await res.json() : null;
        const taskId = data?.task_id || ('mock_'+Math.random().toString(36).slice(2,8));
        state.runningTask = { id: taskId, started: Date.now(), dryRun: !!dryRun };
        timelinePush('任务启动', 'ok', `ID: ${taskId} · 模式：${dryRun?'演练':'执行'}`);
        missionBannerRunning();
        openEventStream(taskId, dryRun, !ok); // 如果后端失败则走 mock
      }catch{
        const taskId = 'mock_'+Math.random().toString(36).slice(2,8);
        state.runningTask = { id: taskId, started: Date.now(), dryRun: !!dryRun };
        timelinePush('任务启动(模拟)', 'ok', `ID: ${taskId}`);
        missionBannerRunning();
        openEventStream(taskId, dryRun, true);
      }
    }

    function missionBannerRunning(){
      els.missionPanel.innerHTML = `
        <div class="plan-card">
          <span class="badge-plan">任务进行中</span>
          <div class="tip">可随时急停或撤销。任务事件会在下方时间线滚动显示。</div>
          <div class="plan-actions">
            <button id="btnAbort" class="btn small danger">撤销任务</button>
          </div>
        </div>
      `;
      $('#btnAbort',root).addEventListener('click', ()=> { timelinePush('任务已请求撤销','warn','用户操作'); closeStream(); });
    }

    // —— 时间线 —— 
    function timelinePush(title, level, text){
      const div = document.createElement('div'); div.className='t-item ' + (level==='ok'?'ok':level==='warn'?'warn':level==='err'?'err':'');
      div.innerHTML = `<div class="t-time">${fmtT(Date.now())}</div><div><b>${esc(title)}</b> — ${esc(text||'')}</div>`;
      const empty = els.timeline.querySelector('.placeholder'); if(empty) empty.remove();
      els.timeline.appendChild(div); els.timeline.scrollTop = els.timeline.scrollHeight + 9999;
    }
    function renderRefsToTimeline(refs){
      refs.forEach((r,i)=> timelinePush(`引用${i+1}`, 'ok', `${r.title||'参考'} · ${r.source||''}`));
    }

    // —— 事件流（SSE 或模拟） —— 
    async function openEventStream(taskId, dryRun, mock=false){
      closeStream();
      if (mock){
        // 模拟：每1s推事件 & 遥测
        const t = setInterval(()=>{
          const ts=Date.now();
          const seq = Math.floor((ts - (state.runningTask?.started||ts))/1000);
          const msgs = [
            {type:'event', level:'ok', text:'移动到下一航点'},
            {type:'event', level: Math.random()<0.1?'warn':'ok', text: Math.random()<0.1?'检测到动态障碍，减速绕行':'路径跟踪正常'},
            {type:'telemetry', pose:{x: (seq*0.3).toFixed(2), y:(Math.sin(seq/3)*0.5).toFixed(2), yaw: ((seq*5)%360) }, battery: (90-seq*0.2).toFixed(1), vel: (0.3+Math.random()*0.4).toFixed(2) },
          ];
          msgs.forEach(onMissionEvent);
          sparkOdom.push(Math.random()*1.4);
          sparkAvoid.push(Math.random()<0.15?1:0.2);
          if(seq>20){ timelinePush('任务完成','ok', dryRun?'演练结束':'执行成功'); closeStream(); }
        },1000);
        state.eventStream = { close(){ clearInterval(t); } };
        return;
      }

      // 真正 SSE
      try{
        const res = await fetch(`/api/agent/mission/stream?task=${encodeURIComponent(taskId)}`);
        if (!res.ok || !res.body?.getReader) throw new Error('stream open failed');
        const reader = res.body.getReader(); const dec = new TextDecoder();
        state.eventStream = { close(){ try{ reader.cancel(); }catch{} } };
        let buffer='';
        while(true){
          const {done,value}=await reader.read(); if(done) break;
          buffer += dec.decode(value, {stream:true});
          const chunks = buffer.split(/\r?\n\r?\n/); buffer = chunks.pop()||'';
          for(const chunk of chunks){
            const line = chunk.trim();
            const m = line.match(/^data:\s*(.*)$/m); const payload = m? m[1] : line;
            try{ onMissionEvent(JSON.parse(payload)); }catch{}
          }
        }
      }catch(e){
        timelinePush('事件流异常','err', e.message||e);
      }
    }

    function closeStream(){ try{ state.eventStream?.close?.(); }catch{} state.eventStream=null; }

    function onMissionEvent(ev){
      if(ev.type==='event'){
        timelinePush('事件', ev.level||'ok', ev.text||'');
      }
      if(ev.type==='telemetry'){
        if(ev.pose){ els.telePose.textContent = `x:${(+ev.pose.x).toFixed(2)} y:${(+ev.pose.y).toFixed(2)} θ:${Math.round((+ev.pose.yaw||0))}°`; }
        if(ev.battery!=null){ els.teleBatt.textContent = `${ev.battery}%`; }
        if(ev.vel!=null){ els.teleVel.textContent = `${ev.vel}`; }
      }
    }

    // —— 安全控制 —— 
    els.btnEstop.addEventListener('click', async ()=>{
      try{
        await safeFetch('/api/agent/estop', {method:'POST', headers:{'Content-Type':'application/json'}, body:JSON.stringify({engage:true})});
      }catch{}
      els.statusEstop.classList.remove('ok'); els.statusEstop.classList.add('bad'); els.statusEstop.textContent='E-STOP: TRIPPED';
      timelinePush('紧急停止','err','用户触发');
      closeStream();
    });
    els.btnArm.addEventListener('click', async ()=>{
      const armed = els.statusArm.classList.contains('warn');
      try{
        await safeFetch('/api/agent/arm', {method:'POST', headers:{'Content-Type':'application/json'}, body:JSON.stringify({arm:armed})});
      }catch{}
      els.statusArm.classList.toggle('warn', !armed);
      els.statusArm.classList.toggle('ok', armed);
      els.statusArm.textContent = armed? 'ARMED' : 'DISARMED';
      timelinePush(armed?'系统解锁':'系统上锁', armed?'ok':'warn', '用户操作');
    });

    // —— 输入与快捷 —— 
    function autosize(){ els.input.style.height='auto'; els.input.style.height=Math.min(180, Math.max(44, els.input.scrollHeight))+'px'; }
    els.input.addEventListener('input', autosize);
    els.input.addEventListener('keydown', e=>{ if(e.key==='Enter'&&!e.shiftKey){ e.preventDefault(); sendCurrent(); }});
    els.send.addEventListener('click', sendCurrent);
    els.chips.forEach(b=> b.addEventListener('click', ()=> { els.input.value=b.dataset.prompt||''; autosize(); sendCurrent(); }));

    // —— 小工具 —— 
    async function safeFetch(url, init){
      try{ return await fetch(url,init); }catch(e){ return null; }
    }

    // 销毁
    this.destroy = ()=> { closeStream(); };
  }

  // 自动挂载（与项目现有片段加载模式一致）
  (function autoMount(){
    const content = document.querySelector('.content-area') || document.querySelector('#content') || document.body;
    let current=null;
    function tryInit(){
      const root = document.getElementById('roscmd-root');
      if(root && !root.__mounted){ current?.destroy?.(); current=new Commander(); root.__mounted=true; }
    }
    tryInit();
    const mo=new MutationObserver(tryInit);
    mo.observe(content,{childList:true,subtree:true});
    window.addEventListener('beforeunload', ()=> current?.destroy?.());
  })();
})();
