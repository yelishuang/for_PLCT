/* terminal-v2.js
 * 适配 loadContent 片段注入：预加载脚本 + MutationObserver 自动初始化
 * 仅前端：实现最小可用终端（键盘/粘贴/重连/清屏/尺寸上报/ANSI 基础色）
 * 后端接口预留：WebSocket wss?://host/ws/terminal?cols=..&rows=..（可带token）
 * 消息协议（建议）：
 *   发送：{op:"data", data:"..."} | {op:"resize", cols, rows} | {op:"control", key:"C"|"D"|...}
 *   接收：{op:"data", data:"..."} | {op:"title", data:"..."} | {op:"exit", code}
 */
(function(){
  const $ = (s,r=document)=>r.querySelector(s);
  const escHTML = s => String(s??'').replace(/[&<>"]/g, m=>({'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;'}[m]));

  // ---- ANSI 极简解析：仅处理 \x1b[ ... m 前景色 + reset ----
  function ansiToHTML(text){
    // 拆分 ANSI SGR：\x1b[ ... m
    const parts = String(text).split(/\x1b\[/);
    if (parts.length === 1) return escHTML(text);
    let out = escHTML(parts[0]);
    for (let i=1; i<parts.length; i++){
      const seg = parts[i];
      const m = seg.match(/^([0-9;]*)m([\s\S]*)$/);
      if (!m){ out += escHTML('\x1b[' + seg); continue; }
      const codes = m[1].split(';').filter(Boolean).map(x=>parseInt(x,10));
      const rest  = m[2];
      // 仅处理前景色 30-37 / 90-97 和 0 reset
      let cls = '';
      if (codes.includes(0) || codes.length===0) {
        out += '</span>' + escHTML(rest);
        continue;
      }
      const fg = codes.find(c => (c>=30 && c<=37) || (c>=90 && c<=97));
      if (fg!=null) cls = `ansi-fg-${fg}`;
      out += `<span class="${cls}">` + escHTML(rest);
    }
    return out;
  }

  function debounce(fn, ms){ let t=null; return (...a)=>{ clearTimeout(t); t=setTimeout(()=>fn(...a), ms); }; }

  // ---- 终端主体 ----
  function TerminalV2(){
    const root = $('#termv2-root'); if(!root) return;

    const viewport = $('#term-viewport', root);
    const screen   = $('#term-screen', root);
    const cursor   = $('#term-cursor', root);
    const input    = $('#term-input', root);
    const status   = $('#term-status', root);
    const btnReconnect = $('#btnReconnect', root);
    const btnClear     = $('#btnClear', root);
    const btnC         = $('#btnCtrlC', root);
    const btnD         = $('#btnCtrlD', root);
    const fontSizeSel  = $('#fontSize', root);
    const themeSel     = $('#themeSelect', root);
    const dimEl        = $('#term-dim', root);

    let ws=null, connected=false, closedByUser=false;
    let cols=0, rows=0;
    let charW=8, charH=16; // 渲染估算：随字体变更重算
    let bufLine='';        // 当前行构建
    let scrollAtBottom = true;

    // ---- 连接管理 ----
    function wsURL(){
      const proto = location.protocol==='https:' ? 'wss' : 'ws';
      const qs = new URLSearchParams({ cols, rows }).toString();
      // 预留 token：可从 cookie/localStorage 注入
      // const token = localStorage.getItem('authToken'); if (token) qs += `&token=${encodeURIComponent(token)}`;
      return `${proto}://${location.host}/ws/terminal?${qs}`;
    }
    function setStatus(mode, text){
      status.classList.remove('online','connecting','error');
      status.classList.add(mode);
      status.textContent = text || ({
        online:'● 在线', connecting:'● 连接中…', error:'● 离线'
      }[mode] || '● 离线');
    }

    function connect(){
      closedByUser = false;
      setStatus('connecting');
      try{
        ws = new WebSocket(wsURL());
        ws.onopen = ()=>{ connected=true; setStatus('online'); focusInput(); };
        ws.onclose = ()=>{ connected=false; setStatus(closedByUser?'error':'error'); retryLater(); };
        ws.onerror = ()=>{ connected=false; setStatus('error'); };
        ws.onmessage = (ev)=>{
          let data = ev.data;
          try { const j = JSON.parse(ev.data); if (j && j.op==='data') data = j.data; }
          catch{} // 普通流式文本
          write(data);
        };
      }catch(e){ setStatus('error'); }
    }
    let retryTimer=null;
    function retryLater(){
      if (closedByUser) return;
      clearTimeout(retryTimer);
      retryTimer = setTimeout(()=> connect(), 1500);
    }
    function disconnect(){
      closedByUser = true;
      try { ws && ws.close(); } catch {}
      ws = null; connected=false; setStatus('error');
    }

    // ---- 输出渲染 ----
    function write(s){
      if (!s) return;
      // 处理 \r \n 与退格（最小化）：按行渲染
      for (const ch of s){
        if (ch === '\r'){ bufLine=''; continue; }
        if (ch === '\n'){ flushLine(); continue; }
        if (ch === '\b'){ bufLine = bufLine.slice(0,-1); continue; }
        bufLine += ch;
      }
      // 若末尾没有换行，实时刷新当前行
      renderCurrentLine();
      keepCursorEnd();
    }
    function flushLine(){
      if (bufLine.length===0){ screen.insertAdjacentHTML('beforeend', '\n'); return; }
      screen.insertAdjacentHTML('beforeend', ansiToHTML(bufLine) + '\n');
      bufLine = '';
    }
    function renderCurrentLine(){
      // 使用最后一个换行后的内容作为“当前行”
      const html = screen.innerHTML;
      const idx = html.lastIndexOf('\n');
      const head = idx>=0 ? html.slice(0, idx+1) : '';
      const tail = idx>=0 ? html.slice(idx+1) : html;
      // 把当前行替换为 bufLine
      const newTail = ansiToHTML(bufLine);
      screen.innerHTML = head + newTail;
    }

    function keepCursorEnd(){
      const rect = screen.getBoundingClientRect();
      cursor.style.left = Math.max(6, rect.left) + 'px'; // 仅做闪烁提示，定位到左边缘
      cursor.style.top  = (viewport.scrollHeight - 20) + 'px';
      if (scrollAtBottom) viewport.scrollTop = viewport.scrollHeight;
    }

    // ---- 输入事件 ----
    function send(obj){
      if (!ws || ws.readyState!==1) return;
      try{ ws.send(JSON.stringify(obj)); }catch{}
    }
    function sendData(str){ send({op:'data', data:str}); }
    function sendResize(){ send({op:'resize', cols, rows}); }

    // 键盘：用隐藏 textarea 捕获，可兼容移动端软键盘
    function focusInput(){ input.focus(); }
    viewport.addEventListener('mousedown', focusInput);
    viewport.addEventListener('click', focusInput);

    input.addEventListener('keydown', (e)=>{
      // 控制键映射（尽量直通 TTY 序列）
      switch(e.key){
        case 'Enter': sendData('\r'); e.preventDefault(); break;
        case 'Backspace': sendData('\x7f'); e.preventDefault(); break;
        case 'Tab': sendData('\t'); e.preventDefault(); break;
        case 'ArrowUp': sendData('\x1b[A'); e.preventDefault(); break;
        case 'ArrowDown': sendData('\x1b[B'); e.preventDefault(); break;
        case 'ArrowRight': sendData('\x1b[C'); e.preventDefault(); break;
        case 'ArrowLeft': sendData('\x1b[D'); e.preventDefault(); break;
        case 'Home': sendData('\x1b[H'); e.preventDefault(); break;
        case 'End': sendData('\x1b[F'); e.preventDefault(); break;
        case 'PageUp': sendData('\x1b[5~'); e.preventDefault(); break;
        case 'PageDown': sendData('\x1b[6~'); e.preventDefault(); break;
        default:
          // Ctrl 组合
          if (e.ctrlKey && !e.altKey && !e.metaKey){
            const k = e.key.toUpperCase();
            if (k.length===1){
              const code = k.charCodeAt(0) - 64; // Ctrl+A -> 1
              if (code >= 1 && code <= 26){ sendData(String.fromCharCode(code)); e.preventDefault(); return; }
            }
          }
      }
    });

    // 输入法/可打印字符：用 input 事件拿增量
    let lastVal = '';
    input.addEventListener('input', ()=>{
      const val = input.value;
      if (val && val !== lastVal){
        const delta = val.replace(lastVal, '');
        sendData(delta);
        lastVal = val;
      }
      // 清空缓冲，避免越来越长
      if (input.value.length > 512){ input.value=''; lastVal=''; }
    });
    // 粘贴
    viewport.addEventListener('paste', (e)=>{
      const text = (e.clipboardData || window.clipboardData)?.getData('text');
      if (text){ e.preventDefault(); sendData(text); }
    });

    // 底栏按钮
    btnReconnect.addEventListener('click', ()=>{ disconnect(); clearScreen(); connect(); reportResize(); });
    btnClear.addEventListener('click', ()=> clearScreen());
    btnC.addEventListener('click', ()=> sendData(String.fromCharCode(3))); // Ctrl+C
    btnD.addEventListener('click', ()=> sendData(String.fromCharCode(4))); // Ctrl+D

    // 字号 / 主题
    fontSizeSel.addEventListener('change', ()=>{
      screen.style.fontSize = fontSizeSel.value + 'px';
      measureChar(); reportResize();
    });
    themeSel.addEventListener('change', ()=>{
      const isLight = themeSel.value==='light';
      root.classList.toggle('theme-light', isLight);
    });

    // ---- 尺寸测量/上报 ----
    function measureChar(){
      // 通过创建临时 span 测量单个字符大小
      const tmp = document.createElement('span');
      tmp.textContent='M';
      tmp.style.visibility='hidden';
      tmp.style.position='absolute';
      tmp.style.fontFamily = getComputedStyle(screen).fontFamily;
      tmp.style.fontSize   = getComputedStyle(screen).fontSize;
      document.body.appendChild(tmp);
      charW = tmp.getBoundingClientRect().width || 8;
      charH = tmp.getBoundingClientRect().height || 16;
      tmp.remove();
    }
    function computeColsRows(){
      const r = viewport.getBoundingClientRect();
      const pad = 20; // 内边距预留
      cols = Math.max(20, Math.floor((r.width - pad) / charW));
      rows = Math.max(5,  Math.floor((r.height - pad) / charH));
      dimEl.textContent = `${cols}×${rows}`;
    }
    const reportResize = debounce(()=>{ computeColsRows(); sendResize(); }, 100);
    window.addEventListener('resize', reportResize);
    viewport.addEventListener('scroll', ()=>{
      // 如果滚到最底则跟随，否则不打扰用户查看历史
      const nearBottom = viewport.scrollHeight - viewport.scrollTop - viewport.clientHeight < 8;
      scrollAtBottom = nearBottom;
    });

    // ---- 清屏 ----
    function clearScreen(){
      screen.innerHTML = '';
      bufLine = '';
      viewport.scrollTop = 0;
    }

    // ---- 初始化 ----
    function init(){
      measureChar();
      computeColsRows();
      connect();
      reportResize();
      focusInput();
    }

    init();

    // 导出销毁（离开页面）
    this.destroy = ()=>{ disconnect(); window.removeEventListener('resize', reportResize); };
  }

  // ---- 自动挂载：监听内容区片段注入 ----
  (function autoMount(){
    const content = document.querySelector('.content-area') || document.querySelector('#content') || document.body;
    let current = null;

    function tryInit(){
      const root = document.getElementById('termv2-root');
      if (root && !root.__mounted){
        current?.destroy?.();
        current = new TerminalV2();
        root.__mounted = true;
      }
    }
    tryInit();
    const mo = new MutationObserver(tryInit);
    mo.observe(content, { childList:true, subtree:true });
    window.addEventListener('beforeunload', ()=> current?.destroy?.());
  })();
})();
