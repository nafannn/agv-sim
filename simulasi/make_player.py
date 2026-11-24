"""Membuat pemutar animasi HTML mandiri (tanpa internet, tanpa library).

Data dua kontroler (PID dan SF) dijalankan dari simulator yang sama, lalu
disematkan sebagai JSON ke dalam satu berkas HTML: lintasan tampak atas,
kecepatan, dan heading, lengkap dengan tombol putar/jeda dan penggeser waktu.

    py -3.12 make_player.py
"""

from __future__ import annotations

import json
import os
import sys

ROOT = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, ROOT)

from animate import jalankan

TEMPLATE = r"""<!DOCTYPE html>
<html lang="id">
<head>
<meta charset="utf-8">
<title>Pemutar simulasi AGV</title>
<style>
  :root { color-scheme: light dark; }
  body { margin: 0; background: transparent; color: var(--foreground, #111);
         font-family: var(--font, system-ui, sans-serif); font-size: 12px; }
  .wrap { width: 700px; }
  .row { display: flex; gap: 10px; align-items: flex-start; }
  .top { display: flex; gap: 12px; }
  canvas { display: block; border: 1px solid var(--border, #ccc); border-radius: 6px; }
  .ctrl { display: flex; gap: 8px; align-items: center; margin: 8px 0 6px; flex-wrap: wrap; }
  button, select {
    font: inherit; padding: 3px 10px; border-radius: 5px; cursor: pointer;
    border: 1px solid var(--border, #bbb); background: var(--card, #fff);
    color: var(--foreground, #111);
  }
  button.primary { border-color: var(--accent, #1f77b4); color: var(--accent, #1f77b4); }
  input[type=range] { width: 260px; accent-color: var(--accent, #1f77b4); }
  .status { font-variant-numeric: tabular-nums; color: var(--muted-foreground, #666);
            margin-left: auto; }
  label { color: var(--muted-foreground, #666); }
</style>
</head>
<body>
<div class="wrap">
  <div class="ctrl">
    <button id="play" class="primary">Putar</button>
    <button id="reset">Ulangi</button>
    <label>Kontroler
      <select id="who"><option value="pid">PID cascade</option><option value="sf">SF + integral</option></select>
    </label>
    <label>Kecepatan
      <select id="speed"><option value="0.5">0,5x</option><option value="1" selected>1x</option><option value="2">2x</option></select>
    </label>
    <input id="scrub" type="range" min="0" value="0" step="1">
    <span class="status" id="status"></span>
  </div>
  <div class="top">
    <canvas id="path" width="340" height="340"></canvas>
    <div style="display:flex; flex-direction:column; gap:10px;">
      <canvas id="cv" width="316" height="162"></canvas>
      <canvas id="ch" width="316" height="162"></canvas>
    </div>
  </div>
</div>
<script>
const DATA = __DATA__;
const css = (n, f) => (getComputedStyle(document.body).getPropertyValue(n) || f).trim();
const COL = { fg: css('--foreground', '#111'), muted: css('--muted-foreground', '#666'),
              acc: css('--accent', '#1f77b4'), border: css('--border', '#ccc') };

function fit(cv) {
  const r = window.devicePixelRatio || 1, w = cv.width, h = cv.height;
  cv.style.width = w + 'px'; cv.style.height = h + 'px';
  cv.width = w * r; cv.height = h * r;
  const ctx = cv.getContext('2d'); ctx.scale(r, r);
  return { ctx, w, h };
}
const P = fit(document.getElementById('path'));
const V = fit(document.getElementById('cv'));
const Hc = fit(document.getElementById('ch'));

const who = document.getElementById('who');
const scrub = document.getElementById('scrub');
const statusEl = document.getElementById('status');
const btnPlay = document.getElementById('play');
const selSpeed = document.getElementById('speed');

let key = 'pid', i = 0, playing = false, speed = 1, last = 0;

function D() { return DATA[key]; }
function resizeScrub() { scrub.max = D().t.length - 1; }

function drawPath() {
  const d = D(), { ctx, w, h } = P, m = 26;
  ctx.clearRect(0, 0, w, h);
  const xs = d.x, ys = d.y;
  let x0 = Math.min(...xs), x1 = Math.max(...xs), y0 = Math.min(...ys), y1 = Math.max(...ys);
  const pad = 0.12; x0 -= pad; x1 += pad; y0 -= pad; y1 += pad;
  const s = Math.min((w - 2 * m) / (x1 - x0), (h - 2 * m) / (y1 - y0));
  const cx = (x0 + x1) / 2, cy = (y0 + y1) / 2;
  const X = v => w / 2 + (v - cx) * s, Y = v => h / 2 - (v - cy) * s;

  // grid
  ctx.strokeStyle = COL.border; ctx.globalAlpha = .5; ctx.lineWidth = .6;
  const step = 0.5;
  for (let gx = Math.ceil(x0 / step) * step; gx <= x1; gx += step) {
    ctx.beginPath(); ctx.moveTo(X(gx), m); ctx.lineTo(X(gx), h - m); ctx.stroke();
  }
  for (let gy = Math.ceil(y0 / step) * step; gy <= y1; gy += step) {
    ctx.beginPath(); ctx.moveTo(m, Y(gy)); ctx.lineTo(w - m, Y(gy)); ctx.stroke();
  }
  ctx.globalAlpha = 1;
  // jejak penuh (samar)
  ctx.strokeStyle = COL.muted; ctx.globalAlpha = .45; ctx.setLineDash([2, 3]);
  ctx.beginPath(); ctx.moveTo(X(xs[0]), Y(ys[0]));
  for (let k = 1; k < xs.length; k++) ctx.lineTo(X(xs[k]), Y(ys[k]));
  ctx.stroke(); ctx.setLineDash([]); ctx.globalAlpha = 1;
  // jejak sampai indeks i
  ctx.strokeStyle = COL.acc; ctx.lineWidth = 2;
  ctx.beginPath(); ctx.moveTo(X(xs[0]), Y(ys[0]));
  for (let k = 1; k <= i; k++) ctx.lineTo(X(xs[k]), Y(ys[k]));
  ctx.stroke();
  // AGV
  const L = 0.22, W = 0.17, half = 0.075, px = X(xs[i]), py = Y(ys[i]), a = d.psi[i];
  ctx.save(); ctx.translate(px, py); ctx.rotate(-a);
  ctx.fillStyle = 'rgba(190,190,190,.95)'; ctx.strokeStyle = COL.fg; ctx.lineWidth = 1;
  ctx.beginPath(); ctx.rect(-L * s / 2, -W * s / 2, L * s, W * s); ctx.fill(); ctx.stroke();
  ctx.fillStyle = '#444';
  for (const sx of [-1, 1]) for (const sy of [-1, 1]) {
    ctx.fillRect(sx * (L / 2 - 0.045) * s - 0.025 * s, sy * half * s - 0.015 * s, 0.05 * s, 0.03 * s);
  }
  ctx.strokeStyle = COL.acc; ctx.lineWidth = 2; ctx.beginPath();
  ctx.moveTo(0, 0); ctx.lineTo(L * 0.6 * s, 0); ctx.stroke();
  ctx.beginPath(); ctx.moveTo(L * 0.6 * s, 0);
  ctx.lineTo(L * 0.6 * s - 0.03 * s, -0.03 * s);
  ctx.lineTo(L * 0.6 * s - 0.03 * s, 0.03 * s); ctx.closePath();
  ctx.fillStyle = COL.acc; ctx.fill();
  ctx.restore();
  // label
  ctx.fillStyle = COL.muted; ctx.font = '11px system-ui, sans-serif';
  ctx.fillText('X [m] →', w - 54, h - 8);
  ctx.save(); ctx.translate(12, h / 2); ctx.rotate(-Math.PI / 2);
  ctx.textAlign = 'center'; ctx.fillText('Y [m]', 0, 0); ctx.restore();
}

function drawChart(o, series, refSeries, unit, title, minSpan) {
  const { ctx, w, h } = o, m = { l: 40, r: 8, t: 18, b: 20 };
  ctx.clearRect(0, 0, w, h);
  const t = D().t, tmax = t[t.length - 1];
  // rentang sumbu dihitung setelah transien awal agar bentuk gelombang terlihat
  const k0 = t.findIndex(v => v >= 0.6);
  const win = a => a.slice(Math.max(0, k0));
  let lo = Math.min(...win(refSeries), ...win(series));
  let hi = Math.max(...win(refSeries), ...win(series));
  let span = Math.max(hi - lo, minSpan || 1e-3);
  const mid = (hi + lo) / 2;
  if (hi - lo < span) { lo = mid - span / 2; hi = mid + span / 2; }
  lo -= span * .12; hi += span * .12;
  const X = v => m.l + (v / tmax) * (w - m.l - m.r);
  const Y = v => h - m.b - ((v - lo) / (hi - lo)) * (h - m.t - m.b);

  ctx.strokeStyle = COL.border; ctx.globalAlpha = .55; ctx.lineWidth = .6;
  for (let k = 0; k <= 4; k++) {
    const yy = m.t + k * (h - m.t - m.b) / 4;
    ctx.beginPath(); ctx.moveTo(m.l, yy); ctx.lineTo(w - m.r, yy); ctx.stroke();
  }
  ctx.globalAlpha = 1;
  // klip supaya lonjakan di luar rentang tidak keluar kotak
  ctx.save();
  ctx.beginPath(); ctx.rect(m.l, m.t, w - m.l - m.r, h - m.t - m.b); ctx.clip();
  // referensi
  ctx.strokeStyle = COL.muted; ctx.setLineDash([4, 3]); ctx.lineWidth = 1;
  ctx.beginPath(); refSeries.forEach((v, k) => k ? ctx.lineTo(X(t[k]), Y(v)) : ctx.moveTo(X(t[k]), Y(v)));
  ctx.stroke(); ctx.setLineDash([]);
  // sinyal
  ctx.strokeStyle = COL.acc; ctx.lineWidth = 1.6;
  ctx.beginPath();
  for (let k = 0; k <= i; k++) k ? ctx.lineTo(X(t[k]), Y(series[k])) : ctx.moveTo(X(t[k]), Y(series[k]));
  ctx.stroke();
  ctx.restore();
  // kursor + titik
  ctx.strokeStyle = COL.border; ctx.beginPath();
  ctx.moveTo(X(t[i]), m.t); ctx.lineTo(X(t[i]), h - m.b); ctx.stroke();
  ctx.fillStyle = COL.acc; ctx.beginPath(); ctx.arc(X(t[i]), Y(series[i]), 3, 0, 7); ctx.fill();
  // teks
  ctx.fillStyle = COL.muted; ctx.font = '11px system-ui, sans-serif';
  ctx.fillText(title, m.l, 12);
  const dp = (hi - lo) < 1 ? 3 : 1;
  ctx.textAlign = 'right';
  ctx.fillText(hi.toFixed(dp), m.l - 4, m.t + 8);
  ctx.fillText(lo.toFixed(dp), m.l - 4, h - m.b + 2);
  ctx.textAlign = 'left';
  ctx.fillText(tmax.toFixed(0) + ' s', w - m.r - 16, h - 4);
  ctx.fillStyle = COL.fg;
  ctx.fillText(series[i].toFixed(3) + ' ' + unit, w - m.r - 62, 12);
}

function frame(now) {
  const d = D();
  drawPath();
  drawChart(V, d.v, d.v_ref, 'm/s', 'kecepatan linear', 0.05);
  drawChart(Hc, d.psi_deg, d.psi_ref_deg, 'deg', 'heading', 20);
  statusEl.textContent = `t = ${d.t[i].toFixed(2)} s  |  ψ = ${d.psi_deg[i].toFixed(1)}°  |  ` +
                         `V_L = ${d.VL[i].toFixed(2)} V, V_R = ${d.VR[i].toFixed(2)} V`;
  scrub.value = i;

  if (playing) {
    const dt = Math.max(0, Math.min(now - last, 100)); last = now;
    i += dt / 1000 * 30 * speed;          // 30 sampel per detik data
    if (i >= d.t.length - 1) { i = d.t.length - 1; playing = false; btnPlay.textContent = 'Putar'; }
    requestAnimationFrame(frame);
  }
}

function render() { requestAnimationFrame(frame); }

btnPlay.onclick = () => {
  playing = !playing; btnPlay.textContent = playing ? 'Jeda' : 'Putar';
  if (playing) { last = performance.now(); requestAnimationFrame(frame); }
  else render();
};
document.getElementById('reset').onclick = () => { i = 0; playing = false; btnPlay.textContent = 'Putar'; render(); };
who.onchange = e => { key = e.target.value; i = 0; playing = false; btnPlay.textContent = 'Putar'; resizeScrub(); render(); };
selSpeed.onchange = e => { speed = parseFloat(e.target.value); };
scrub.oninput = e => { i = parseInt(e.target.value, 10); render(); };
window.addEventListener('keydown', e => {
  if (e.code === 'Space') { e.preventDefault(); btnPlay.click(); }
});

resizeScrub();
render();
</script>
</body>
</html>
"""


def main():
    data = {}
    for key in ("pid", "sf"):
        tr = jalankan(key, 10.0)
        step = 2   # 50 Hz -> 500 sampel
        data[key] = {
            "t": [round(float(v), 4) for v in tr.t[::step]],
            "x": [round(float(v), 5) for v in tr.X[::step]],
            "y": [round(float(v), 5) for v in tr.Y[::step]],
            "psi": [round(float(v), 5) for v in tr.psi[::step]],
            "psi_deg": [round(float(v), 3) for v in (tr.psi[::step] * 180 / 3.141592653589793)],
            "psi_ref_deg": [round(float(v), 3) for v in (tr.psi_ref[::step] * 180 / 3.141592653589793)],
            "v": [round(float(v), 4) for v in tr.v[::step]],
            "v_ref": [round(float(v), 4) for v in tr.v_ref[::step]],
            "VL": [round(float(v), 3) for v in tr.VL[::step]],
            "VR": [round(float(v), 3) for v in tr.VR[::step]],
        }

    html = TEMPLATE.replace("__DATA__", json.dumps(data, separators=(",", ":")))
    out = os.path.join(ROOT, "figs", "agv_player.html")
    with open(out, "w", encoding="utf-8") as f:
        f.write(html)
    print("pemutar:", out, f"({os.path.getsize(out)/1024:.0f} KB)")


if __name__ == "__main__":
    main()
