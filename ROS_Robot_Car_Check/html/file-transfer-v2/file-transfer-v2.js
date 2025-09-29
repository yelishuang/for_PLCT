/* file-transfer-v2.js
 * 双栏文件管理（左=本机/工作区, 右=开发板）
 * - 左栏：通过 File System Access API（优先）或 <input webkitdirectory> 挂载“工作区”
 * - 右栏：通过后端 API 列目录/上传/下载/操作（此处以 Mock 占位，方便前端先跑起来）
 * - 支持拖拽互传、选择/双击进入、命令栏（cd/cp/mv/rm/mkdir）
 * - 任务队列展示进度（前端模拟），对接后端可走 SSE / 轮询进度
 * 适配：页面通过 loadContent 注入 fragment，脚本预加载后自动初始化（MutationObserver）
 */

(function(){
  const $ = (s,r=document)=>r.querySelector(s);
  const $$=(s,r=document)=>Array.from(r.querySelectorAll(s));
  const esc = s => String(s??'').replace(/[&<>"]/g,m=>({'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;'}[m]));
  const fmtSize = n => { if(n==null||isNaN(n)) return '-'; const u=['B','KB','MB','GB','TB']; let i=0,v=n; while(v>=1024&&i<u.length-1){v/=1024;i++} return `${v<10?v.toFixed(2):v.toFixed(1)} ${u[i]}`; };
  const fmtTime = ts => ts? new Date(ts).toLocaleString() : '-';

  // ---------------- Mock 后端（右侧）与任务 ----------------
  const API = (()=> {
    // 右侧接口（替换成真实 fetch 即可）
    const mockFS = {
      '/': { type:'dir', mtime:Date.now(), children:{
        'home': { type:'dir', mtime:Date.now()-86400000, children:{
          'root': { type:'dir', mtime:Date.now()-3600000, children:{
            'readme.txt': { type:'file', size: 1234, mtime:Date.now()-500000 },
          }},
        }},
        'var':  { type:'dir', mtime:Date.now()-7200000, children:{}},
        'tmp':  { type:'dir', mtime:Date.now()-120000, children:{}},
      }},
    };
    function getNode(p){
      const segs = p.split('/').filter(Boolean);
      let node = mockFS['/'];
      for(const s of segs){
        if(!node.children || !node.children[s]) return null;
        node = node.children[s];
      }
      return node;
    }
    async function listRight(path='/'){
      await delay(80);
      const node = (path==='/'? mockFS['/'] : getNode(path));
      if(!node || node.type!=='dir') throw new Error('Not found');
      const rows = Object.entries(node.children||{}).map(([name,meta])=> ({
        name, type: meta.type, size: meta.size||0, mtime: meta.mtime||Date.now()
      }));
      rows.sort((a,b)=> (a.type===b.type? a.name.localeCompare(b.name) : (a.type==='dir'?-1:1)));
      return { path, rows };
    }
    async function mkdirRight(path){ await delay(60); // mock
      // 简化：仅支持在现有目录下建一层
      const parent = path.replace(/\/+$/,'').split('/').slice(0,-1).join('/') || '/';
      const base = path.replace(/\/+$/,'').split('/').pop();
      const pnode = (parent==='/'? mockFS['/'] : getNode(parent));
      if(!pnode || pnode.type!=='dir') throw new Error('Parent missing');
      if(!pnode.children) pnode.children = {};
      pnode.children[base] = { type:'dir', mtime: Date.now(), children:{} };
      return { ok:true };
    }
    async function deleteRight(path){ await delay(60);
      const parent = path.replace(/\/+$/,'').split('/').slice(0,-1).join('/') || '/';
      const base = path.replace(/\/+$/,'').split('/').pop();
      const pnode = (parent==='/'? mockFS['/'] : getNode(parent));
      if(!pnode || !pnode.children || !pnode.children[base]) throw new Error('Not found');
      delete pnode.children[base]; return {ok:true};
    }
    async function renameRight(path, newName){ await delay(60);
      const parent = path.replace(/\/+$/,'').split('/').slice(0,-1).join('/') || '/';
      const base = path.replace(/\/+$/,'').split('/').pop();
      const pnode = (parent==='/'? mockFS['/'] : getNode(parent));
      if(!pnode || !pnode.children || !pnode.children[base]) throw new Error('Not found');
      pnode.children[newName] = pnode.children[base]; delete pnode.children[base]; return {ok:true};
    }

    // 上传（右侧落地）：files 为 FileList/数组
    async function uploadRight(dir, files, onProgress){
      for (let i=0;i<files.length;i++){
        const f = files[i];
        // progress 模拟
        for (let p=0; p<=100; p+=10){ await delay(30); onProgress && onProgress(i, files.length, p); }
        // 写入 mockFS
        const node = (dir==='/'? mockFS['/'] : getNode(dir)); if(!node||node.type!=='dir') throw new Error('dir missing');
        node.children = node.children||{};
        node.children[f.name] = { type:'file', size:f.size||0, mtime:Date.now() };
      }
      return { ok:true };
    }

    // 打包下载（右→左）：后端应把选中文件/目录打包为 zip 并返回可下载 URL
    async function packRightToZip(paths){
      await delay(120);
      // mock：返回一个假的 URL（前端不会实际下载）
      return { ok:true, url: '#download.zip', name:'download.zip', size: 1024*128 };
    }

    function delay(ms){ return new Promise(r=>setTimeout(r,ms)); }
    return { listRight, mkdirRight, deleteRight, renameRight, uploadRight, packRightToZip };
  })();

  // ---------------- 左侧：本机/工作区（浏览器） ----------------
  function LocalWorkspace(){
    // 采用两种策略：
    // 1) File System Access API: showDirectoryPicker() → 可读目录结构（推荐，Chrome/Edge）
    // 2) 回退：<input webkitdirectory multiple> 选择，扁平文件清单（目录结构有限）
    const state = { rootName:'工作区', cwd:'/', entries: {} /* path -> {type,size,mtime} */ };

    async function mountWithFSAccess(){
      if (!window.showDirectoryPicker) return false;
      const dirHandle = await window.showDirectoryPicker({id:'ftv2-local'});
      state.handle = dirHandle; state.rootName = dirHandle.name || '工作区';
      state.entries = {}; state.cwd = '/';
      await walkDir(dirHandle, '/');
      return true;
    }

    async function walkDir(handle, base){
      for await (const [name,entry] of handle.entries()){
        const path = base + name;
        if (entry.kind === 'directory'){
          state.entries[path] = { type:'dir', mtime: Date.now() };
          await walkDir(entry, path + '/');
        } else {
          const f = await entry.getFile();
          state.entries[path] = { type:'file', size:f.size, mtime:f.lastModified };
        }
      }
    }

    function mountWithInput(files){
      state.entries = {};
      for (const f of files){
        const rel = f.webkitRelativePath || f.name;
        const segs = rel.split('/'); // 构造“目录项”
        let cur = '/';
        for (let i=0;i<segs.length-1;i++){
          const p = cur + segs[i];
          state.entries[p] = state.entries[p] || { type:'dir', mtime: f.lastModified };
          cur = p + '/';
        }
        state.entries[cur + segs[segs.length-1]] = { type:'file', size:f.size, mtime:f.lastModified, file:f };
      }
      state.rootName = '本地选择';
      state.cwd = '/';
    }

    function list(path){
      // 列出某目录下的第一层
      const rows = [];
      const prefix = (path==='/'?'':path);
      const seen = new Set();
      for (const p of Object.keys(state.entries)){
        if (!p.startsWith(prefix)) continue;
        const rest = p.slice(prefix.length);
        const seg = rest.split('/')[0];
        if (!seg) continue;
        if (seen.has(seg)) continue;
        seen.add(seg);
        const childPath = prefix + seg;
        const meta = state.entries[childPath] || state.entries[childPath + '/'];
        if (!meta) continue;
        rows.push({ name:seg, type: meta.type, size: meta.size, mtime: meta.mtime });
      }
      rows.sort((a,b)=> (a.type===b.type? a.name.localeCompare(b.name) : (a.type==='dir'?-1:1)));
      return { path, rows };
    }

    function readSelectedFiles(path, names){
      // 从 entries 中拿到 File 对象（仅对 input 方式有效；FS Access 方式需再次通过 handle.getFile() 拉）
      const out = [];
      for (const n of names){
        const pFile = (path==='/'? '' : path) + n;
        const meta = state.entries[pFile];
        if (meta?.file) out.push(meta.file);
      }
      return out;
    }

    // 导出 API
    return { state, mountWithFSAccess, mountWithInput, list, readSelectedFiles };
  }

  // ---------------- 主视图 ----------------
  function FTV2(){
    const root = $('#ftv2-root'); if(!root) return;

    // 左
    const left = {
      rows: $('#leftRows', root),
      crumbs: $('#leftCrumbs', root),
      empty: $('#leftEmpty', root),
      up: $('#leftUp', root),
      home: $('#leftHome', root),
      refresh: $('#btnLeftRefresh', root),
      mount: $('#btnMountLocal', root),
      inputDir: $('#inputLocalDir', root),
      tableWrap: $('#leftTableWrap', root),
      selected: new Set(),
      cwd: '/',
    };
    // 右
    const right = {
      rows: $('#rightRows', root),
      crumbs: $('#rightCrumbs', root),
      empty: $('#rightEmpty', root),
      up: $('#rightUp', root),
      home: $('#rightHome', root),
      refresh: $('#btnRightRefresh', root),
      uploadBtn: $('#btnUpload', root),
      uploadInput: $('#inputUpload', root),
      newFolder: $('#btnNewFolder', root),
      tableWrap: $('#rightTableWrap', root),
      selected: new Set(),
      cwd: '/',
    };
    const status = $('#ftv2-status', root);
    const cmdInput = $('#cmdInput', root), cmdRun = $('#cmdRun', root);
    const taskList = $('#taskList', root), taskClearDone = $('#taskClearDone', root);

    const local = LocalWorkspace();

    // 初始化：右侧列根，左侧空
    void refreshRight('/');
    renderLeftEmpty();

    // ---------- 左侧：挂载/刷新/导航 ----------
    left.mount.addEventListener('click', async ()=>{
      // 优先 FS Access
      try {
        const ok = await local.mountWithFSAccess();
        if (ok){ left.cwd = '/'; renderLeft(); return; }
      }catch(e){ /* fallback */ }
      // 回退 input 方式
      left.inputDir.click();
    });
    left.inputDir.addEventListener('change', (e)=>{
      const files = e.target.files || [];
      if (!files.length) return;
      local.mountWithInput(files);
      left.cwd = '/';
      renderLeft();
    });
    left.refresh.addEventListener('click', ()=> renderLeft());
    left.up.addEventListener('click', ()=> { left.cwd = parentDir(left.cwd); renderLeft(); });
    left.home.addEventListener('click', ()=> { left.cwd = '/'; renderLeft(); });

    function renderLeft(){
      const { rows } = local.list(left.cwd);
      renderTable(left, rows);
      renderCrumbs(left, 'left');
      left.empty.classList.toggle('hidden', rows.length>0);
    }
    function renderLeftEmpty(){
      left.rows.innerHTML = ''; left.empty.classList.remove('hidden');
      left.crumbs.innerHTML = ''; addCrumb(left.crumbs, '工作区', '/', 'left');
    }

    // ---------- 右侧：列目录/上传/新建/导航 ----------
    right.refresh.addEventListener('click', ()=> refreshRight(right.cwd));
    right.up.addEventListener('click', ()=> { const p = parentDir(right.cwd); refreshRight(p); });
    right.home.addEventListener('click', ()=> { refreshRight('/'); });

    async function refreshRight(path){
      setStatus('loading','加载中…');
      try{
        const { rows } = await API.listRight(path);
        right.cwd = path; renderTable(right, rows); renderCrumbs(right, 'right');
        right.empty.classList.toggle('hidden', rows.length>0);
        setStatus('ok','就绪');
      }catch(e){
        setStatus('error','加载失败'); console.error(e);
      }
    }

    right.newFolder.addEventListener('click', async ()=>{
      const name = prompt('新建文件夹名：'); if(!name) return;
      const target = joinPath(right.cwd, name);
      try{ await API.mkdirRight(target); await refreshRight(right.cwd); }catch(e){ alert('创建失败'); }
    });

    right.uploadBtn.addEventListener('click', ()=> right.uploadInput.click());
    right.uploadInput.addEventListener('change', async (e)=>{
      const files = e.target.files || []; if(!files.length) return;
      await doUpload(Array.from(files), right.cwd);
      right.uploadInput.value = '';
    });

    async function doUpload(files, targetDir){
      // 创建任务
      const task = addTask({ name:`上传到 ${targetDir}`, meta:`${files.length} 个文件`, progress:0 });
      try{
        await API.uploadRight(targetDir, files, (i, total, p)=>{
          const overall = Math.round(((i + p/100) / total) * 100);
          updateTask(task, overall);
        });
        finishTask(task, true);
        await refreshRight(targetDir);
      }catch(e){
        console.error(e); finishTask(task, false, '上传失败');
      }
    }

    // ---------- 双栏渲染与交互 ----------
    function renderTable(side, rows){
      side.selected.clear();
      const tbody = side.rows;
      tbody.innerHTML = '';
      const frag = document.createDocumentFragment();
      rows.forEach(r=>{
        const tr = document.createElement('tr');
        tr.draggable = true;
        tr.dataset.name = r.name; tr.dataset.type = r.type;
        tr.innerHTML = `
          <td>${r.type==='dir'?'📁':'📄'} <span class="fname">${esc(r.name)}</span></td>
          <td>${r.type==='dir'?'—':fmtSize(r.size)}</td>
          <td>${fmtTime(r.mtime)}</td>
        `;
        tr.addEventListener('click', (ev)=>{
          if (ev.ctrlKey || ev.metaKey){
            toggleSelect(side, r.name, tr);
          }else{
            side.selected.clear();
            $$('tr', tbody).forEach(x=>x.classList.remove('selected'));
            selectRow(side, r.name, tr);
          }
        });
        tr.addEventListener('dblclick', ()=>{
          if (r.type==='dir'){
            if (side===left){
              side.cwd = joinPath(side.cwd, r.name); renderLeft();
            }else{
              const p = joinPath(side.cwd, r.name); void refreshRight(p);
            }
          }else{
            if (side===right){
              // 右→本地下载（后端打包/直链）
              downloadRight([joinPath(side.cwd, r.name)]);
            }
          }
        });

        // drag
        tr.addEventListener('dragstart', (e)=>{
          const names = getSelectedOrSingle(side, r.name);
          e.dataTransfer.setData('text/plain', JSON.stringify({ from: (side===left?'left':'right'), path: side.cwd, names }));
          e.dataTransfer.effectAllowed = 'copyMove';
        });

        frag.appendChild(tr);
      });
      tbody.appendChild(frag);

      // 作为 drop 目标
      side.tableWrap.addEventListener('dragover', (e)=>{ e.preventDefault(); e.dataTransfer.dropEffect='copy'; });
      side.tableWrap.addEventListener('drop', async (e)=>{
        e.preventDefault();
        try{
          const payload = JSON.parse(e.dataTransfer.getData('text/plain'));
          if (!payload) return;
          if (payload.from==='left' && side===right){
            // 左→右 上传
            const files = local.readSelectedFiles(payload.path, payload.names);
            if (!files.length){
              alert('当前本地挂载方式无法直接读取文件，请使用“上传到此处”按钮'); return;
            }
            await doUpload(files, side.cwd);
          } else if (payload.from==='right' && side===left){
            // 右→左 下载（打包）
            const paths = payload.names.map(n => joinPath(right.cwd, n));
            await downloadRight(paths);
          }
        }catch(err){ /* ignore */ }
      });
    }

    function selectRow(side, name, tr){
      side.selected.add(name); tr.classList.add('selected');
    }
    function toggleSelect(side, name, tr){
      if (side.selected.has(name)){ side.selected.delete(name); tr.classList.remove('selected'); }
      else { side.selected.add(name); tr.classList.add('selected'); }
    }
    function getSelectedOrSingle(side, fallbackName){
      const arr = Array.from(side.selected);
      return arr.length? arr : [fallbackName];
    }

    function renderCrumbs(side, tag){
      side.crumbs.innerHTML = '';
      const parts = side.cwd.split('/').filter(Boolean);
      addCrumb(side.crumbs, tag==='left'? local.state.rootName : (tag==='right'?'/' : '/'), '/', tag);
      let p = '/';
      for (const seg of parts){
        p = joinPath(p, seg);
        addCrumb(side.crumbs, seg, p, tag);
      }
    }
    function addCrumb(container, label, path, tag){
      const d = document.createElement('div');
      d.className='crumb'; d.textContent=label;
      d.title = path;
      d.addEventListener('click', ()=> {
        if (tag==='left'){ left.cwd = path; renderLeft(); }
        else { void refreshRight(path); }
      });
      container.appendChild(d);
    }

    // ---------- 命令栏 ----------
    cmdRun.addEventListener('click', runCmd);
    cmdInput.addEventListener('keydown', e=> { if (e.key==='Enter') runCmd(); });

    async function runCmd(){
      const cmd = cmdInput.value.trim(); if(!cmd) return;
      if (cmd==='help'){
        alert('示例：\n  cd right:/var\n  mkdir right:/tmp/newdir\n  rm right:/tmp/a.txt\n  cp left:/work/a.txt right:/home/root/\n  mv right:/tmp/a.txt right:/home/root/a.txt');
        return;
      }
      try{
        const [verb, ...rest] = cmd.split(/\s+/);
        if (verb==='cd'){
          const target = rest[0]; const {side, path} = parseSidePath(target);
          if (side==='left'){ left.cwd = normPath(path); renderLeft(); }
          else { await refreshRight(normPath(path)); }
        }else if (verb==='mkdir'){
          const target = rest[0]; const {side, path} = parseSidePath(target);
          if (side==='right'){ await API.mkdirRight(normPath(path)); await refreshRight(parentDir(normPath(path))); }
          else { alert('左侧为本地浏览器工作区，暂不支持直接创建'); }
        }else if (verb==='rm'){
          const target = rest[0]; const {side, path} = parseSidePath(target);
          if (side==='right'){ if(confirm(`删除 ${path} ?`)){ await API.deleteRight(normPath(path)); await refreshRight(parentDir(normPath(path))); } }
          else { alert('左侧删除请在系统文件管理器中进行'); }
        }else if (verb==='mv' || verb==='cp'){
          const src = parseSidePath(rest[0]); const dst = parseSidePath(rest[1]);
          if (verb==='cp'){
            if (src.side==='left' && dst.side==='right'){
              // 从左到右的复制 = 上传
              // 简化：仅支持单文件名（可扩展为通配）
              const base = src.path.split('/').pop();
              const files = local.readSelectedFiles(parentDir(src.path), [base]);
              if (!files.length){ alert('本地读取失败：请通过“挂载+选择/拖拽/上传”'); return; }
              await doUpload(files, normPath(dst.path));
            }else if (src.side==='right' && dst.side==='left'){
              await downloadRight([normPath(src.path)]);
            }else{
              alert('当前仅支持 左→右（上传） 与 右→左（下载）');
            }
          }else{
            alert('演示版本未实现右侧移动 mv（可后端直接实现 server-side mv）');
          }
        }else{
          alert('不支持的命令');
        }
      }catch(e){ alert(`执行失败：${e.message||e}`); }
    }

    function parseSidePath(s){
      const m = String(s||'').match(/^(left|right):(.+)$/);
      if (!m) throw new Error('路径需形如 left:/x 或 right:/x');
      return { side:m[1], path:m[2] };
    }

    // ---------- 下载（右→左） ----------
    async function downloadRight(paths){
      if (!paths || !paths.length) return;
      const task = addTask({ name:'下载', meta:`${paths.length} 项`, progress:0 });
      try{
        // 后端建议：打包为 zip 返回 URL；若是单文件也可直接返回直链
        updateTask(task, 10);
        const { url, name } = await API.packRightToZip(paths);
        updateTask(task, 90);
        // 触发浏览器下载
        const a = document.createElement('a'); a.href = url; a.download = name || 'download.zip'; a.className='link'; document.body.appendChild(a); a.click(); a.remove();
        finishTask(task, true);
      }catch(e){
        console.error(e); finishTask(task, false, '下载失败');
      }
    }

    // ---------- 任务队列 ----------
    function addTask({ name, meta, progress }){
      const id = 't' + Math.random().toString(36).slice(2,8);
      const el = document.createElement('div');
      el.className = 'task'; el.id = id;
      el.innerHTML = `
        <div>
          <div class="name">${esc(name)}</div>
          <div class="meta">${esc(meta||'')}</div>
        </div>
        <div class="progress">
          <div class="progress-bar"><span style="width:${progress||0}%"></span></div>
        </div>
        <div class="act muted">进行中</div>
      `;
      taskList.querySelector('.placeholder')?.remove();
      taskList.appendChild(el);
      return el;
    }
    function updateTask(el, pct){
      const bar = el.querySelector('.progress-bar > span'); if(bar) bar.style.width = Math.max(0, Math.min(100, pct)) + '%';
    }
    function finishTask(el, ok, msg){
      el.classList.add(ok? 'ok':'fail');
      el.querySelector('.act').textContent = ok? '完成' : (msg || '失败');
      updateTask(el, 100);
    }
    taskClearDone.addEventListener('click', ()=>{
      $$('.task.ok', taskList).forEach(x=>x.remove());
      if (!taskList.children.length) taskList.innerHTML = '<div class="placeholder slim">暂无任务。</div>';
    });

    // ---------- 辅助 ----------
    function setStatus(kind, text){
      status.textContent = (kind==='ok'?'● 就绪': text || '● …');
    }
  }

  // ---------- 通用路径函数 ----------
  function parentDir(p){
    if (p==='/' || !p) return '/';
    const s = p.replace(/\/+$/,'').split('/'); s.pop(); const out = s.join('/') || '/'; return out;
  }
  function joinPath(a,b){
    if (a==='/') return '/' + b.replace(/^\/+/,'');
    return (a.replace(/\/+$/,'') + '/' + b.replace(/^\/+/,'')).replace(/\/+/g,'/');
  }
  function normPath(p){ return ('/' + String(p||'').replace(/^\/+/,'')).replace(/\/+/g,'/'); }

  // ---------- 自动挂载 ----------
  (function autoMount(){
    const content = document.querySelector('.content-area') || document.querySelector('#content') || document.body;
    let current = null;
    function tryInit(){
      const root = document.getElementById('ftv2-root');
      if (root && !root.__mounted){
        current?.destroy?.();
        current = new FTV2();
        root.__mounted = true;
      }
    }
    tryInit();
    const mo = new MutationObserver(tryInit);
    mo.observe(content, { childList:true, subtree:true });
    window.addEventListener('beforeunload', ()=> current?.destroy?.());
  })();
})();
