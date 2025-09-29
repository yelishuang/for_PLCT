// 仅前端：统一封装 API。当前用“假数据+占位”，后续替换为真实接口即可。
export const StorageAPI = (() => {
  const LAT = 320; // 模拟网络延迟

  // 工具：字节 -> 带单位
  function fmtBytes(n){
    if (n == null || isNaN(n)) return "-";
    const u = ["B","KB","MB","GB","TB"]; let i=0, v = n;
    while (v >= 1024 && i < u.length - 1){ v/=1024; i++; }
    return `${v.toFixed(v<10?2:1)} ${u[i]}`;
  }

  // 假数据
  const mockSummary = {
    total: 32*1024**3,
    free:  18*1024**3,
    rootUsage: 43,         // %
    maxTemp: null          // 无温度 → 显示 N/A
  };

  const mockDevices = [
    { device:"mmcblk0", mountpoint:"/",    fstype:"ext4",  total:"29.7 GB", used:"12.8 GB", free:"16.9 GB", percent:"43%", health:"ok" },
    { device:"mmcblk0p1",mountpoint:"/boot",fstype:"vfat", total:"256 MB",  used:"68 MB",   free:"188 MB",  percent:"26%", health:"ok" },
    { device:"sda1",     mountpoint:"/data",fstype:"ext4", total:"58.4 GB", used:"5.2 GB",  free:"53.2 GB", percent:"8%",  health:"ok" }
  ];

  // ====== API 形态（未来直接改成 fetch） ======
  async function getSummary(){
    // const r = await fetch('/api/storage/summary'); return r.json();
    await delay(LAT);
    return mockSummary;
  }

  async function getDevices(){
    // const r = await fetch('/api/storage/devices'); return r.json();
    await delay(LAT);
    return mockDevices;
  }

  // SSE：返回 { connect, close }，当前用假流（可关闭）
  function openRealtime({ onMessage, onOpen, onError }){
    let timer = null, opened = false;
    return {
      connect(){
        try{
          opened = true;
          onOpen && onOpen();
          timer = setInterval(()=>{
            if (!opened) return;
            const now = Date.now();
            // 模拟 200~3500 KB/s
            const read  = (200 + Math.random()*3300) * 1024;
            const write = (100 + Math.random()*2500) * 1024;
            onMessage && onMessage({ ts: now, read_bps: Math.round(read), write_bps: Math.round(write) });
          }, 1000);
        }catch(e){ onError && onError(e); }
      },
      close(){
        opened = false;
        if (timer) clearInterval(timer);
      }
    };
  }

  function delay(ms){ return new Promise(r=>setTimeout(r,ms)); }

  return { fmtBytes, getSummary, getDevices, openRealtime };
})();
