// 轻量折线图（原生 Canvas），与 logs-v2 的风格一致
export class MiniLine {
  constructor(canvasId, color = "#4a90e2", maxPoints = 60){
    this.canvas = document.getElementById(canvasId);
    this.ctx = this.canvas.getContext("2d");
    this.color = color;
    this.max = maxPoints;
    this.data = [];
    // 让 canvas 自适应 DPR，提高清晰度
    this.resize();
    window.addEventListener('resize', () => this.resize());
  }
  resize(){
    const dpr = Math.max(1, window.devicePixelRatio || 1);
    const cssW = this.canvas.clientWidth || 300;
    const cssH = this.canvas.clientHeight || 120;
    this.canvas.width  = Math.floor(cssW * dpr);
    this.canvas.height = Math.floor(cssH * dpr);
    this.ctx.setTransform(dpr,0,0,dpr,0,0);
    this.draw();
  }
  push(v){
    if (typeof v !== 'number' || isNaN(v)) v = 0;
    this.data.push(v);
    if (this.data.length > this.max) this.data.shift();
    this.draw();
  }
  draw(){
    const c = this.ctx; const w = this.canvas.clientWidth; const h = this.canvas.clientHeight;
    c.clearRect(0,0,w,h);

    // grid
    c.strokeStyle = "#2a3340"; c.lineWidth = 1;
    for (let i=0;i<4;i++){
      const y = Math.round((h/4)*(i+1));
      c.beginPath(); c.moveTo(0,y); c.lineTo(w,y); c.stroke();
    }

    if (this.data.length < 2) return;
    const max = Math.max(...this.data, 1);
    const step = w / (this.max-1);

    c.strokeStyle = this.color; c.lineWidth = 2; c.beginPath();
    for (let i=0;i<this.data.length;i++){
      const x = i * step;
      const y = h - (this.data[i] / max) * (h - 6); // 上下留 6px
      if (i===0) c.moveTo(x,y); else c.lineTo(x,y);
    }
    c.stroke();
  }
}
