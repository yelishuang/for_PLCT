/* runtime-v2.js  —— 运行中心（服务 / 进程 / 用户）
 * 适配 loadContent 片段注入：脚本预加载后通过 MutationObserver 自动初始化。
 * 内置 Mock 数据，便于无后端时演示；接入后端只需替换 API.fetch。
 */
(function(){
  // 简易选择器
  const $ = (s,r=document)=>r.querySelector(s); const $$=(s,r=document)=>Array.from(r.querySelectorAll(s));
  const nowStr = (ts=Date.now()) => new Date(ts).toLocaleTimeString();
  const esc = s => String(s??'').replace(/[&<>"]/g,m=>({'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;'}[m]));
  const fmtPct = n => (n==null?'-':(Math.round(n*10)/10)+'%');
  const fmtBytes = n => { if(n==null||isNaN(n)) return '-'; const u=['B','KB','MB','GB']; let i=0,v=n; while(v>=1024 && i<u.length-1){v/=1024;i++} return `${v<10?v.toFixed(2):v.toFixed(1)} ${u[i]}`; };

  // -------------------- Mock API（接入后端：替换为真实 fetch） --------------------
  const API = (() => {
    const LAT = 220;
    async function getSummary(){
      await delay(LAT);
      return { svcActive: 18, svcFailed: 1, cpu: 12.6, mem: 43.8, users: 3, locked: 1, events5m: 7 };
    }
    async function getServices(){
      await delay(LAT);
      return [
        { name:'nginx', desc:'Web 服务器', active:'active', enabled:true },
        { name:'ssh',   desc:'OpenSSH Server', active:'active', enabled:true },
        { name:'demo-app', desc:'样例应用', active:'failed', enabled:true, lastErr:'退出码 1 (core dumped)' },
        { name:'cron',  desc:'计划任务', active:'inactive', enabled:false }
      ];
    }
    async function getServiceDetail(name){
      await delay(LAT);
      return {
        name, status:`● ${name} - active (running)`, since:'2025-09-29 10:15:02',
        recentLogs:[
          { ts:Date.now()-60000, msg:`${name}: ready` },
          { ts:Date.now()-30000, msg:`${name}: handling request...` },
        ]
      };
    }
    async function serviceAction(name, action){ await delay(200); return { ok:true, action, name }; }

    async function getProcesses(q=''){
      await delay(LAT);
      const rows = [
        { pid:712, cmd:'sshd: root@pts/0', cpu:0.3, mem:0.5, rss:38*1024*1024, start:'10:01:11' },
        { pid:901, cmd:'nginx: worker',    cpu:1.6, mem:0.8, rss:22*1024*1024, start:'09:58:10' },
        { pid:233, cmd:'demo-app',         cpu:23.2, mem:12.0, rss:180*1024*1024, start:'09:31:55' }
      ];
      return rows.filter(r=>r.cmd.toLowerCase().includes(q.toLowerCase()));
    }
    async function procAction(pid, action){ await delay(150); return {ok:true, pid, action}; }
    async function procSnapshot(pid){ await delay(300); return { ok:true, pid, url:'#' }; }

    async function getUsers(){
      await delay(LAT);
      return [
        { user:'root', uid:0,  group:'root', locked:false, last:'2025-09-28 21:33' },
        { user:'dev',  uid:1000, group:'dev', locked:false, last:'2025-09-29 08:12' },
        { user:'guest',uid:1001, group:'users', locked:true, last:'—' }
      ];
    }
    async function userAction(user, action){ await delay(180); return { ok:true, user, action }; }

    function delay(ms){ return new Promise(r=>setTimeout(r,ms)); }
    return { getSummary, getServices, getServiceDetail, serviceAction, getProcesses, procAction, procSnapshot, getUsers, userAction };
  })();

  // -------------------- 主视图 --------------------
  function RuntimeV2(){
    const root = $('#rtv2-root'); if(!root) return;

    // 顶部 KPI
    const kpi = {
      svcActive: $('#kpiSvcActive', root), svcFailed: $('#kpiSvcFailed', root),
      cpu: $('#kpiCPU', root), mem: $('#kpiMEM', root),
      users: $('#kpiUsers', root), locked: $('#kpiLocked', root),
      events: $('#kpiEvents', root)
    };
    const guardBtn = $('#rtv2-guard', root);

    // 列表 & 状态
    const els = {
      tabs: $$('.tab', root), search: $('#rtv2-search', root), refresh: $('#rtv2-refresh', root),
      sIdle: $('#sIdle', root), sLoading: $('#sLoading', root), sEmpty: $('#sEmpty', root), sError: $('#sError', root),
      listSvc: $('#listSvc', root), listProc: $('#listProc', root), listUser: $('#listUser', root),
      svcRows: $('#svcRows', root), procRows: $('#procRows', root), userRows: $('#userRows', root),
      detailTitle: $('#detailTitle', root), detailBody: $('#detailBody', root),
      opsSvc: $('#opsSvc', root), opsProc: $('#opsProc', root), opsUser: $('#opsUser', root),
      btnSvcStart: $('#btnSvcStart', root), btnSvcStop: $('#btnSvcStop', root), btnSvcRestart: $('#btnSvcRestart', root),
      btnSvcEnable: $('#btnSvcEnable', root), btnSvcDisable: $('#btnSvcDisable', root),
      btnProcTerm: $('#btnProcTerm', root), btnProcKill: $('#btnProcKill', root), btnProcSnap: $('#btnProcSnap', root),
      btnUserAdd: $('#btnUserAdd', root), btnUserLock: $('#btnUserLock', root), btnUserUnlock: $('#btnUserUnlock', root), btnUserPwd: $('#btnUserPwd', root), btnUserKey: $('#btnUserKey', root),
      logTitle: $('#logTitle', root), logList: $('#logList', root), btnLogMore: $('#btnLogMore', root), btnLogClear: $('#btnLogClear', root)
    };

    // 状态
    const state = {
      tab: 'svc',        // svc | proc | user
      maintain: false,   // 维护模式
      selected: null,    // 当前选中项（服务名 / pid / 用户名）
      caches: { svc:[], proc:[], user:[] }
    };

    // —— 顶部 KPI —— //
    async function refreshKPI(){
      try{
        const s = await API.getSummary();
        kpi.svcActive.textContent = s.svcActive;
        kpi.svcFailed.textContent = s.svcFailed;
        kpi.cpu.textContent = fmtPct(s.cpu);
        kpi.mem.textContent = fmtPct(s.mem);
        kpi.users.textContent = s.users;
        kpi.locked.textContent = s.locked;
        kpi.events.textContent = s.events5m;
      }catch(e){ console.error(e); }
    }

    // —— 切换 TAB —— //
    els.tabs.forEach(btn=>{
      btn.addEventListener('click', ()=>{
        els.tabs.forEach(b=>b.classList.remove('active'));
        btn.classList.add('active');
        state.tab = btn.dataset.tab;
        renderTab();
      });
    });

    function setListState(which){
      const show = (el, yes)=> el && el.classList.toggle('hidden', !yes);
      show(els.sIdle, which==='idle');
      show(els.sLoading, which==='loading');
      show(els.sEmpty, which==='empty');
      show(els.sError, which==='error');
    }

    function renderTab(){
      // 列表显隐
      els.listSvc.classList.toggle('hidden', state.tab!=='svc');
      els.listProc.classList.toggle('hidden', state.tab!=='proc');
      els.listUser.classList.toggle('hidden', state.tab!=='user');

      // 操作显隐
      els.opsSvc.classList.toggle('hidden', state.tab!=='svc');
      els.opsProc.classList.toggle('hidden', state.tab!=='proc');
      els.opsUser.classList.toggle('hidden', state.tab!=='user');

      // 标题
      const titleMap = { svc:'服务详情', proc:'进程详情', user:'用户详情' };
      els.detailTitle.textContent = titleMap[state.tab] || '详情';
      els.logTitle.textContent    = (state.tab==='svc'?'服务日志': (state.tab==='proc'?'进程事件':'用户事件'));

      // 清空详情/日志
      state.selected = null;
      els.detailBody.textContent = '请选择左侧列表中的一项查看详情。';
      els.logList.innerHTML = '<div class="placeholder slim">尚无日志/事件。</div>';

      // 加载
      void refreshList();
    }

    // —— 列表刷新 —— //
    async function refreshList(){
      setListState('loading');
      try{
        if(state.tab==='svc'){
          const rows = await API.getServices();
          state.caches.svc = rows;
          renderSvc(rows);
        }else if(state.tab==='proc'){
          const rows = await API.getProcesses(els.search.value.trim());
          state.caches.proc = rows;
          renderProc(rows);
        }else{
          const rows = await API.getUsers();
          state.caches.user = rows;
          renderUser(rows);
        }
        setListState((getActiveRows().length)?'idle':'empty');
      }catch(e){
        console.error(e); setListState('error');
      }
    }

    function getActiveRows(){
      if(state.tab==='svc') return state.caches.svc;
      if(state.tab==='proc')return state.caches.proc;
      return state.caches.user;
    }

    function renderSvc(rows){
      els.svcRows.innerHTML = '';
      const frag = document.createDocumentFragment();
      rows.forEach(r=>{
        const tr = document.createElement('tr');
        tr.tabIndex = 0;
        tr.innerHTML = `
          <td><strong>${esc(r.name)}</strong></td>
          <td>${badgeSvc(r.active)}</td>
          <td>${r.enabled?'<span class="badge ok">enabled</span>':'<span class="badge na">disabled</span>'}</td>
          <td class="muted">${esc(r.desc||'')}</td>
        `;
        tr.addEventListener('click', ()=> selectSvc(r));
        frag.appendChild(tr);
      });
      els.svcRows.appendChild(frag);
    }
    function badgeSvc(s){
      const m = String(s).toLowerCase();
      if(m==='active') return '<span class="badge ok">active</span>';
      if(m==='failed') return '<span class="badge fail">failed</span>';
      return '<span class="badge na">inactive</span>';
    }
    async function selectSvc(row){
      state.selected = row.name;
      els.detailTitle.textContent = `服务详情 · ${row.name}`;
      els.detailBody.textContent = '加载中…';
      try{
        const d = await API.getServiceDetail(row.name);
        els.detailBody.innerHTML =
          `状态：${esc(d.status)}\n自：${esc(d.since)}\n${d.recentLogs?.length?'最近日志：':''}`;
        // 日志
        renderLogs((d.recentLogs||[]).map(x=>({ts:x.ts, text:x.msg})));
      }catch(e){ els.detailBody.textContent = '加载失败。'; }
    }

    function renderProc(rows){
      els.procRows.innerHTML = '';
      const frag = document.createDocumentFragment();
      rows.forEach(r=>{
        const tr = document.createElement('tr');
        tr.tabIndex = 0;
        tr.innerHTML = `
          <td>${r.pid}</td>
          <td>${esc(r.cmd)}</td>
          <td>${fmtPct(r.cpu)}</td>
          <td>${fmtPct(r.mem)}</td>
          <td>${fmtBytes(r.rss)}</td>
          <td>${esc(r.start)}</td>
        `;
        tr.addEventListener('click', ()=> selectProc(r));
        frag.appendChild(tr);
      });
      els.procRows.appendChild(frag);
    }
    function selectProc(row){
      state.selected = row.pid;
      els.detailTitle.textContent = `进程详情 · ${row.pid}`;
      els.detailBody.innerHTML =
        `PID：${row.pid}\n命令：${row.cmd}\nCPU：${fmtPct(row.cpu)}  ·  内存：${fmtPct(row.mem)}  ·  RSS：${fmtBytes(row.rss)}\n启动：${row.start}`;
      renderLogs([{ts:Date.now()-20000,text:`监控：${row.pid} 采样就绪`},{ts:Date.now()-10000,text:`RSS ${fmtBytes(row.rss)}`}]);
    }

    function renderUser(rows){
      els.userRows.innerHTML = '';
      const frag = document.createDocumentFragment();
      rows.forEach(r=>{
        const tr = document.createElement('tr');
        tr.tabIndex = 0;
        tr.innerHTML = `
          <td><strong>${esc(r.user)}</strong></td>
          <td>${r.uid}</td>
          <td>${esc(r.group||'-')}</td>
          <td>${r.locked?'<span class="badge fail">locked</span>':'<span class="badge ok">active</span>'}</td>
          <td>${esc(r.last||'—')}</td>
        `;
        tr.addEventListener('click', ()=> selectUser(r));
        frag.appendChild(tr);
      });
      els.userRows.appendChild(frag);
    }
    function selectUser(row){
      state.selected = row.user;
      els.detailTitle.textContent = `用户详情 · ${row.user}`;
      els.detailBody.innerHTML = `用户：${row.user}\nUID：${row.uid}\n主组：${row.group}\n锁定：${row.locked?'是':'否'}\n最近登录：${row.last||'—'}`;
      renderLogs([{ts:Date.now()-15000,text:`${row.user}: 最近一次成功认证` }]);
    }

    // —— 日志 / 事件 —— //
    function renderLogs(rows){
      els.logList.innerHTML = '';
      if(!rows || !rows.length){
        els.logList.innerHTML = '<div class="placeholder slim">尚无日志/事件。</div>'; return;
      }
      const frag = document.createDocumentFragment();
      rows.forEach(x=>{
        const div = document.createElement('div'); div.className='log-item';
        div.innerHTML = `<div class="log-time">${nowStr(x.ts)}</div><div>${esc(x.text)}</div>`;
        frag.appendChild(div);
      });
      els.logList.appendChild(frag);
    }
    els.btnLogMore?.addEventListener('click', ()=> {
      // 演示：追加几条
      const demo = [{ts:Date.now(),text:'更多日志 · 演示'}];
      const frag = document.createDocumentFragment();
      demo.forEach(x=>{
        const div=document.createElement('div');div.className='log-item';
        div.innerHTML=`<div class="log-time">${nowStr(x.ts)}</div><div>${esc(x.text)}</div>`;
        frag.appendChild(div);
      });
      els.logList.appendChild(frag);
    });
    els.btnLogClear?.addEventListener('click', ()=> { els.logList.innerHTML='<div class="placeholder slim">已清空。</div>'; });

    // —— 搜索 / 刷新 —— //
    let searchTimer=null;
    els.search.addEventListener('input', ()=>{
      clearTimeout(searchTimer);
      searchTimer=setTimeout(()=> refreshList(), 250);
    });
    els.refresh.addEventListener('click', ()=> { void refreshAll(); });

    async function refreshAll(){
      await refreshKPI();
      await refreshList();
    }

    // —— 操作：维护模式门闩 —— //
    guardBtn.addEventListener('click', ()=>{
      state.maintain = !state.maintain;
      guardBtn.classList.toggle('on', state.maintain);
      guardBtn.classList.toggle('off', !state.maintain);
      guardBtn.textContent = `维护模式：${state.maintain?'开启':'关闭'}`;
      guardBtn.title = guardBtn.textContent;
      alert(`维护模式已${state.maintain?'开启':'关闭'}。${state.maintain?'危险操作将启用。':'危险操作已禁用。'}`);
    });

    // —— 操作：服务 —— //
    function ensureMaintain(){ if(!state.maintain){ alert('请先开启“维护模式”再执行此操作。'); return false;} return true; }
    els.btnSvcStart?.addEventListener('click', async ()=> {
      if(!state.selected) return alert('请选择一个服务');
      if(!ensureMaintain()) return;
      if(!confirm(`启动服务 ${state.selected} ?`)) return;
      await API.serviceAction(state.selected, 'start'); alert('已执行：start'); refreshList();
    });
    els.btnSvcStop?.addEventListener('click', async ()=> {
      if(!state.selected) return alert('请选择一个服务');
      if(!ensureMaintain()) return;
      if(!confirm(`停止服务 ${state.selected} ?（可能中断功能）`)) return;
      await API.serviceAction(state.selected, 'stop'); alert('已执行：stop'); refreshList();
    });
    els.btnSvcRestart?.addEventListener('click', async ()=> {
      if(!state.selected) return alert('请选择一个服务');
      if(!ensureMaintain()) return;
      if(!confirm(`重启服务 ${state.selected} ?`)) return;
      await API.serviceAction(state.selected, 'restart'); alert('已执行：restart'); refreshList();
    });
    els.btnSvcEnable?.addEventListener('click', async ()=> {
      if(!state.selected) return alert('请选择一个服务');
      if(!ensureMaintain()) return;
      await API.serviceAction(state.selected, 'enable'); alert('已执行：enable'); refreshList();
    });
    els.btnSvcDisable?.addEventListener('click', async ()=> {
      if(!state.selected) return alert('请选择一个服务');
      if(!ensureMaintain()) return;
      await API.serviceAction(state.selected, 'disable'); alert('已执行：disable'); refreshList();
    });

    // —— 操作：进程 —— //
    els.btnProcTerm?.addEventListener('click', async ()=>{
      if(!state.selected) return alert('请选择一个进程');
      if(!ensureMaintain()) return;
      if(!confirm(`向 ${state.selected} 发送 TERM？`)) return;
      await API.procAction(state.selected, 'TERM'); alert('已发送 TERM');
    });
    els.btnProcKill?.addEventListener('click', async ()=>{
      if(!state.selected) return alert('请选择一个进程');
      if(!ensureMaintain()) return;
      if(!confirm(`强制杀死 ${state.selected} (KILL)？该操作不可恢复！`)) return;
      await API.procAction(state.selected, 'KILL'); alert('已发送 KILL');
    });
    els.btnProcSnap?.addEventListener('click', async ()=>{
      if(!state.selected) return alert('请选择一个进程');
      const r = await API.procSnapshot(state.selected);
      alert('已生成诊断（演示）。接入后端后返回下载链接。');
    });

    // —— 操作：用户 —— //
    els.btnUserAdd?.addEventListener('click', async ()=>{
      if(!ensureMaintain()) return;
      const name = prompt('请输入新用户名：');
      if(!name) return;
      await API.userAction(name, 'add'); alert(`已添加用户：${name}`); refreshList();
    });
    els.btnUserLock?.addEventListener('click', async ()=>{
      if(!state.selected) return alert('请选择一个用户'); if(!ensureMaintain()) return;
      await API.userAction(state.selected, 'lock'); alert('已锁定'); refreshList();
    });
    els.btnUserUnlock?.addEventListener('click', async ()=>{
      if(!state.selected) return alert('请选择一个用户'); if(!ensureMaintain()) return;
      await API.userAction(state.selected, 'unlock'); alert('已解锁'); refreshList();
    });
    els.btnUserPwd?.addEventListener('click', async ()=>{
      if(!state.selected) return alert('请选择一个用户'); if(!ensureMaintain()) return;
      const p = prompt('请输入新密码（演示）'); if(!p) return;
      await API.userAction(state.selected, 'passwd'); alert('已重置密码（演示）');
    });
    els.btnUserKey?.addEventListener('click', async ()=>{
      if(!state.selected) return alert('请选择一个用户'); if(!ensureMaintain()) return;
      alert('SSH Key 管理（演示）：接入后端后提供列表与增删。');
    });

    // 初次渲染
    refreshAll();
  }

  // -------------------- 自动初始化：监听片段注入 --------------------
  (function autoMount(){
    const content = document.querySelector('.content-area') || document.querySelector('#content') || document.body;
    let current = null;

    function tryInit(){
      const root = document.getElementById('rtv2-root');
      if(root && !root.__mounted){
        current?.destroy?.();
        current = new RuntimeV2();
        root.__mounted = true;
      }
    }
    tryInit();
    const mo = new MutationObserver(tryInit);
    mo.observe(content, { childList:true, subtree:true });
    window.addEventListener('beforeunload', ()=> current?.destroy?.());
  })();
})();
