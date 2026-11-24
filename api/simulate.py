"""Simulasi AGV (Python, tanpa dependensi) sebagai Vercel serverless function.

Endpoint:  GET /api/simulate?skenario=step&kontroler=pid&m_scale=1&mu_scale=1

Seluruh dinamika dan algoritma kontrol ditulis eksplisit (Runge-Kutta 4),
tidak memakai numpy/scipy — supaya cold start cepat dan tidak perlu
requirements.txt sama sekali.

Fungsi run_sim() juga dipakai oleh dev_server.py untuk uji lokal.
"""

from __future__ import annotations

import json
import math
import os
from http.server import BaseHTTPRequestHandler
from urllib.parse import parse_qs, urlparse

# ======================================================================
# Parameter fisis (Tabel 3.2 laporan)
# ======================================================================
BASE = dict(
    M=2.5, R=0.0318, B=0.15, JZ=8.70e-3, JW=1.204e-5, JT=2.0e-6, JM=1.0e-6,
    IG=48.0, ETA=0.85, MU_R=0.025, G=9.81, THETA=0.0,
    RA=4.0, LA=2.0e-3, KT=4.265e-3, KE=4.265e-3, BM=6.5e-8, TAU_C=6.0e-4,
    V_SUPPLY=6.0,
)

TS, TD, H = 0.010, 0.015, 1e-4
EPS_W, EPS_V = 0.20, 0.005

# penguatan final
KP_V, KI_V, BETA = 17.78, 54.44, 0.70
KP_POS, KP_W, KI_W = 4.0, 2.448, 8.162
KP1, KI1, KD1 = 7.050, 9.400, 1.273
KV_FB, KXI_V, N_FF, VC0 = 27.59, 136.5, 6.530, 0.675
KPSI, KW, KXIPSI = 15.86, 2.154, 31.72


def koef(m_scale=1.0, mu_scale=1.0, motor=None, v_supply=None):
    """Hitung koefisien turunan; m_scale/mu_scale/motor/v_supply untuk uji robustness."""
    p = dict(BASE)
    if motor:
        p.update(motor)
    if v_supply:
        p["V_SUPPLY"] = float(v_supply)
    p["M"] *= m_scale
    p["JW"] *= m_scale
    p["JT"] *= m_scale
    p["JM"] *= m_scale
    p["MU_R"] *= mu_scale
    p["BM"] *= mu_scale
    p["TAU_C"] *= mu_scale

    R, B = p["R"], p["B"]
    p["J_EQ"] = p["JW"] + p["JT"] + p["IG"] ** 2 * p["JM"]
    p["M_EQ"] = p["M"] + 4 * p["J_EQ"] / R ** 2
    p["J_PSI"] = p["JZ"] + p["J_EQ"] * B ** 2 / R ** 2
    p["C_F"] = 2 * p["ETA"] * p["IG"] * p["KT"] / R
    p["C_V"] = 4 * p["ETA"] * p["IG"] ** 2 * p["BM"] / R ** 2
    p["F_C"] = 4 * p["ETA"] * p["IG"] * p["TAU_C"] / R
    p["F_ROLL"] = p["MU_R"] * p["M"] * p["G"]
    p["C_PSI"] = 2 * p["ETA"] * p["IG"] ** 2 * p["BM"] * B ** 2 / R ** 2
    p["A_V"] = (2 * p["C_F"] * (p["KE"] * p["IG"] / R) / p["RA"] + p["C_V"]) / p["M_EQ"]
    p["B_V"] = 2 * p["C_F"] / (p["M_EQ"] * p["RA"])
    return p


def sgn_s(x, eps):
    if x > eps:
        return 1.0
    if x < -eps:
        return -1.0
    return x / eps


def sgn0(x, eps=1e-6):
    return 1.0 if x > eps else (-1.0 if x < -eps else 0.0)


def turunan(st, VL, VR, p, Fext=0.0, tau=0.0):
    iL, iR, v, w, psi, _x, _y = st
    R, B = p["R"], p["B"]
    wL = (v - 0.5 * B * w) / R
    wR = (v + 0.5 * B * w) / R
    diL = (VL - p["RA"] * iL - p["KE"] * p["IG"] * wL) / p["LA"]
    diR = (VR - p["RA"] * iR - p["KE"] * p["IG"] * wR) / p["LA"]
    sg_v = sgn_s(v, EPS_V)
    FL = (p["C_F"] * iL - 0.5 * p["C_V"] * (v - 0.5 * B * w)
          - 0.5 * p["F_C"] * sgn_s(wL, EPS_W) - 0.5 * p["F_ROLL"] * sg_v)
    FR = (p["C_F"] * iR - 0.5 * p["C_V"] * (v + 0.5 * B * w)
          - 0.5 * p["F_C"] * sgn_s(wR, EPS_W) - 0.5 * p["F_ROLL"] * sg_v)
    dv = (FL + FR - p["M"] * p["G"] * math.sin(p["THETA"]) - Fext) / p["M_EQ"]
    dw = (0.5 * B * (FR - FL) - p["C_PSI"] * w - tau) / p["J_PSI"]
    return (diL, diR, dv, dw, w, v * math.cos(psi), v * math.sin(psi))


def rk4(st, VL, VR, p, h, Fext=0.0, tau=0.0):
    iL, iR, v, w, psi, x, y = st
    h2 = 0.5 * h
    k1 = turunan((iL, iR, v, w, psi, x, y), VL, VR, p, Fext, tau)
    k2 = turunan((iL + h2 * k1[0], iR + h2 * k1[1], v + h2 * k1[2], w + h2 * k1[3],
                  psi + h2 * k1[4], x + h2 * k1[5], y + h2 * k1[6]), VL, VR, p, Fext, tau)
    k3 = turunan((iL + h2 * k2[0], iR + h2 * k2[1], v + h2 * k2[2], w + h2 * k2[3],
                  psi + h2 * k2[4], x + h2 * k2[5], y + h2 * k2[6]), VL, VR, p, Fext, tau)
    k4 = turunan((iL + h * k3[0], iR + h * k3[1], v + h * k3[2], w + h * k3[3],
                  psi + h * k3[4], x + h * k3[5], y + h * k3[6]), VL, VR, p, Fext, tau)
    h6 = h / 6.0
    return tuple(st[i] + h6 * (k1[i] + 2 * k2[i] + 2 * k3[i] + k4[i]) for i in range(7))


# ======================================================================
# Kontroler
# ======================================================================
class PID:
    def __init__(self, satu_loop=False):
        self.satu_loop = satu_loop
        self.Iv = self.Ih = self.df = 0.0

    def reset(self):
        self.Iv = self.Ih = self.df = 0.0

    def langkah(self, r, meas, dt, vmax):
        v_ref, psi_ref = r.get("v", 0.0), r.get("psi", 0.0)
        a_ref = r.get("a", 0.0)
        v, psi, w = meas

        vff = 6.530 * v_ref + 0.675 * sgn0(v_ref) + 2.133 * a_ref
        u = KP_V * (BETA * v_ref - v) + self.Iv + vff
        Vc = max(-vmax, min(vmax, u))
        self.Iv += (KI_V * (v_ref - v) + 25.0 * (Vc - u)) * dt
        self.Iv = max(-1.5 * vmax, min(1.5 * vmax, self.Iv))

        lim = vmax - abs(Vc)
        if self.satu_loop:
            eh = psi_ref - psi
            self.df += dt / (0.020 + dt) * (w - self.df)
            uh = KP1 * eh + self.Ih - KD1 * self.df
            Vd = max(-lim, min(lim, uh))
            self.Ih += (KI1 * eh + 25.0 * (Vd - uh)) * dt
        else:
            w_ref = max(-3.0, min(3.0, KP_POS * (psi_ref - psi)))
            eh = w_ref - w
            uh = KP_W * eh + self.Ih
            Vd = max(-lim, min(lim, uh))
            self.Ih += (KI_W * eh + 25.0 * (Vd - uh)) * dt
        self.Ih = max(-1.5 * vmax, min(1.5 * vmax, self.Ih))
        return Vc - Vd, Vc + Vd, Vc, Vd


class SF:
    def __init__(self):
        self.xiv = self.xip = 0.0

    def reset(self):
        self.xiv = self.xip = 0.0

    def langkah(self, r, meas, dt, vmax):
        v_ref, psi_ref = r.get("v", 0.0), r.get("psi", 0.0)
        v, psi, w = meas

        u = -KV_FB * v + KXI_V * self.xiv + N_FF * v_ref + VC0 * sgn0(v_ref)
        Vc = max(-vmax, min(vmax, u))
        self.xiv += (v_ref - v) * dt + (Vc - u) * dt / (0.05 * KXI_V)
        self.xiv = max(-20.0, min(20.0, self.xiv))

        lim = vmax - abs(Vc)
        uh = -KPSI * psi - KW * w + KXIPSI * self.xip
        Vd = max(-lim, min(lim, uh))
        self.xip += (psi_ref - psi) * dt + (Vd - uh) * dt / (0.05 * KXIPSI)
        self.xip = max(-5.0, min(5.0, self.xip))
        return Vc - Vd, Vc + Vd, Vc, Vd


# ======================================================================
# Skenario
# ======================================================================
def ref_step(t):
    return {"v": 0.35 if t >= 0.5 else 0.0}


def ref_multi(t):
    v = 0.0
    for t_on, val in ((0.5, 0.15), (2.0, 0.25), (3.5, 0.35), (5.0, 0.20)):
        if t >= t_on:
            v = val
    return {"v": v}


def ref_ramp(t):
    a = 0.4375
    if t < 0.5:
        return {"v": 0.0, "a": 0.0}
    if t < 1.3:
        return {"v": a * (t - 0.5), "a": a}
    if t < 4.0:
        return {"v": 0.35}
    if t < 4.8:
        return {"v": max(0.0, 0.35 - a * (t - 4.0)), "a": -a}
    return {"v": 0.0}


def ref_psi(deg):
    return lambda t: {"psi": math.radians(deg) if t >= 0.5 else 0.0}


def ref_sin(t):
    if t < 0.5:
        return {"psi": 0.0, "v": 0.20}
    return {"psi": 0.7 * math.sin(2 * math.pi * 0.2 * (t - 0.5)), "v": 0.20}


def ref_step_low(t):
    return {"v": 0.0347 if t >= 0.5 else 0.0}


SKENARIO = {
    "rpm500":  ("Kecepatan: 500 rpm motor (0,0347 m/s)", ref_step_low, 5.0, False),
    "step":    ("Kecepatan: tangga 0 → 0,35 m/s", ref_step, 4.0, False),
    "multi":   ("Kecepatan: referensi bertingkat", ref_multi, 7.0, False),
    "ramp":    ("Kecepatan: ramp 0,4375 m/s²", ref_ramp, 6.0, False),
    "gangguan": ("Kecepatan: gangguan beban 1,5 N", ref_step, 6.0, True),
    "psi90":   ("Heading: tangga 0 → 90°", ref_psi(90), 5.0, False),
    "psi180":  ("Heading: tangga 0 → 180°", ref_psi(180), 6.0, False),
    "sinus":   ("Heading: trayektori sinusoid", ref_sin, 6.0, False),
}


def dist_beban(t):
    return {"F_ext": 1.5 if 3.0 <= t < 4.5 else 0.0}


# ======================================================================
# Simulasi + metrik
# ======================================================================
def jalan(kontroler, t_end, ref, pakai_dist, p, h=None):
    k = SF() if kontroler == "sf" else PID(satu_loop=(kontroler == "pid1"))
    k.reset()
    st = [0.0] * 7
    n = int(round(t_end / TS))
    sub = max(1, int(round(TS / (h or H))))
    hh = TS / sub
    antre = []
    VLa = VRa = 0.0

    rec = {key: [] for key in ("t", "v", "v_ref", "psi", "psi_ref", "VL", "VR", "X", "Y")}

    for i in range(n + 1):
        t = i * TS
        r = ref(t)
        meas = (st[2], st[4], st[3])
        VL, VR, Vc, Vd = k.langkah(r, meas, TS, p["V_SUPPLY"])

        antre.append((t + TD, VL, VR))
        while antre and antre[0][0] <= t + 1e-12:
            _, VLa, VRa = antre.pop(0)

        rec["t"].append(t)
        rec["v"].append(st[2])
        rec["v_ref"].append(r.get("v", 0.0))
        rec["psi"].append(st[4])
        rec["psi_ref"].append(r.get("psi", 0.0))
        rec["VL"].append(VLa)
        rec["VR"].append(VRa)
        rec["X"].append(st[5])
        rec["Y"].append(st[6])

        if i == n:
            break
        d = dist_beban(t) if pakai_dist else {}
        for _ in range(sub):
            st = rk4(st, VLa, VRa, p, hh, d.get("F_ext", 0.0), d.get("tau_ext", 0.0))
    return rec


def metrik_tangga(t, y, rf, t0=0.5, t1=None, tol=0.02):
    idx = [i for i, tt in enumerate(t) if tt >= t0 - 1e-12 and (t1 is None or tt < t1 - 1e-12)]
    tt = [t[i] - t0 for i in idx]
    yy = [y[i] for i in idx]
    ref = [rf[i] for i in idx]
    y0, yf = yy[0], ref[-1]
    d = yf - y0
    out = {"tr": None, "ts": None, "MP": 0.0, "ess": abs(yf - yy[-1]), "IAE": None}
    if abs(d) < 1e-9:
        return out
    ys = [(v - y0) / d for v in yy]
    i10 = next((i for i, v in enumerate(ys) if v >= 0.10), None)
    i90 = next((i for i, v in enumerate(ys) if v >= 0.90), None)
    if i10 is not None and i90 is not None:
        out["tr"] = tt[i90] - tt[i10]
    luar = [i for i, v in enumerate(ys) if abs(v - 1.0) > tol]
    out["ts"] = tt[min(luar[-1] + 1, len(tt) - 1)] if luar else 0.0
    out["MP"] = max(0.0, (max(ys) - 1.0) * 100.0)
    iae = 0.0
    for i in range(1, len(tt)):
        iae += 0.5 * (abs(yy[i] - ref[i]) + abs(yy[i - 1] - ref[i - 1])) * (tt[i] - tt[i - 1])
    out["IAE"] = iae
    return out


def run_sim(skenario="step", kontroler="pid", m_scale=1.0, mu_scale=1.0):
    if skenario not in SKENARIO:
        raise ValueError(f"skenario tidak dikenal: {skenario}")
    if kontroler not in ("pid", "pid1", "sf"):
        raise ValueError(f"kontroler tidak dikenal: {kontroler}")

    nama, ref, t_end, pakai_dist = SKENARIO[skenario]
    p = koef(m_scale=float(m_scale), mu_scale=float(mu_scale))
    rec = jalan(kontroler, t_end, ref, pakai_dist, p)

    heading = skenario in ("psi90", "psi180", "sinus")
    y = [math.degrees(v) for v in rec["psi"]] if heading else rec["v"]
    rf = [math.degrees(v) for v in rec["psi_ref"]] if heading else rec["v_ref"]

    if skenario in ("ramp", "sinus"):
        idx = [i for i, tt in enumerate(rec["t"]) if tt >= 0.5]
        e = [y[i] - rf[i] for i in idx]
        met = {"rmse": math.sqrt(sum(v * v for v in e) / len(e)),
               "max_err": max(abs(v) for v in e)}
    else:
        jendela = {"multi": [(0.5, 2.0), (2.0, 3.5), (3.5, 5.0), (5.0, 7.0)],
                   "gangguan": [(0.5, 3.0)]}.get(skenario, [(0.5, None)])
        seg = [metrik_tangga(rec["t"], y, rf, a, b) for a, b in jendela]
        met = {"tr": max((s["tr"] or 0.0) for s in seg),
               "ts": max((s["ts"] or 0.0) for s in seg),
               "MP": max(s["MP"] for s in seg),
               "ess": seg[-1]["ess"],
               "IAE": sum(s["IAE"] for s in seg)}

    # downsampling agar payload ringan (maks ~500 titik)
    step = max(1, int(math.ceil(len(rec["t"]) / 500)))
    sl = slice(None, None, step)

    return {
        "skenario": skenario,
        "nama": nama,
        "kontroler": kontroler,
        "satuan": "deg" if heading else "m/s",
        "metrik": met,
        "t": [round(v, 3) for v in rec["t"][sl]],
        "v": [round(v, 4) for v in rec["v"][sl]],
        "v_ref": [round(v, 4) for v in rec["v_ref"][sl]],
        "psi_deg": [round(math.degrees(v), 2) for v in rec["psi"][sl]],
        "psi_ref_deg": [round(math.degrees(v), 2) for v in rec["psi_ref"][sl]],
        "VL": [round(v, 3) for v in rec["VL"][sl]],
        "VR": [round(v, 3) for v in rec["VR"][sl]],
        "x": [round(v, 4) for v in rec["X"][sl]],
        "y": [round(v, 4) for v in rec["Y"][sl]],
        "params": {"m_eq": round(p["M_EQ"], 3), "J_psi": round(p["J_PSI"], 4),
                   "c_F": round(p["C_F"], 3), "a_v": round(p["A_V"], 3),
                   "b_v": round(p["B_V"], 4)},
    }




LITE_HTML = """<!DOCTYPE html><html lang="id"><head><meta charset="utf-8">
<title>Simulasi AGV (mode ringkas)</title>
<style>body{font-family:system-ui,sans-serif;margin:24px;color:#14171a}
select,button{font:inherit;padding:6px 9px;border-radius:6px;border:1px solid #ccc}
canvas{display:block;margin-top:14px;border:1px solid #ddd;border-radius:6px}
table{border-collapse:collapse;margin-top:12px;font-size:14px}
td,th{border:1px solid #ddd;padding:5px 9px;text-align:left}</style></head><body>
<h2>Simulasi AGV Line Follower 4WD</h2>
<p style="color:#666">Mode ringkas: halaman utama tidak ditemukan di bundel fungsi,
jadi antarmuka ini dilayani langsung oleh fungsi Python.</p>
<label>skenario <select id="s">
<option value="sinus">heading sinusoid</option><option value="step">kecepatan tangga</option>
<option value="psi90">heading 90</option><option value="psi180">heading 180</option>
<option value="ramp">ramp</option><option value="gangguan">gangguan</option>
<option value="multi">bertingkat</option></select></label>
<label style="margin-left:8px">kontroler <select id="k">
<option value="pid">PID cascade</option><option value="sf">SF + integral</option>
<option value="pid1">PID 1 loop</option></select></label>
<button id="go" style="margin-left:8px">Jalankan</button>
<canvas id="c" width="720" height="260"></canvas>
<table id="t"></table>
<script>
const $ = i => document.getElementById(i);
$('go').onclick = async () => {
  const r = await fetch('/api/simulate?skenario=' + $('s').value + '&kontroler=' + $('k').value);
  const d = await r.json();
  if (d.error) { alert(d.error); return; }
  const cv = $('c'), ctx = cv.getContext('2d'), m = 34;
  ctx.clearRect(0, 0, cv.width, cv.height);
  const deg = d.satuan === 'deg';
  const ser = deg ? d.psi_deg : d.v, ref = deg ? d.psi_ref_deg : d.v_ref;
  const lo = Math.min(...ser, ...ref), hi = Math.max(...ser, ...ref);
  const X = i => m + i / (d.t.length - 1) * (cv.width - 2 * m);
  const Y = v => cv.height - m - (v - lo) / (hi - lo || 1) * (cv.height - 2 * m);
  ctx.strokeStyle = '#9aa3ab'; ctx.setLineDash([4, 3]);
  ref.forEach((v, i) => i ? ctx.lineTo(X(i), Y(v)) : ctx.moveTo(X(i), Y(v))); ctx.stroke();
  ctx.setLineDash([]); ctx.strokeStyle = '#1f77b4'; ctx.lineWidth = 1.8;
  ctx.beginPath(); ser.forEach((v, i) => i ? ctx.lineTo(X(i), Y(v)) : ctx.moveTo(X(i), Y(v)));
  ctx.stroke();
  const mm = d.metrik, f = (v, n = 3) => v === undefined ? '-' : (+v).toFixed(n);
  $('t').innerHTML = '<tr><th>metrik</th><th>nilai</th></tr>' +
    Object.keys(mm).map(k => '<tr><td>' + k + '</td><td>' + f(mm[k]) + '</td></tr>').join('');
};
$('go').click();
</script></body></html>"""


def api_respon(kueri: str):
    """Kembalikan (body, content_type, kode) untuk satu permintaan /api/simulate.

    Dipakai bersama oleh handler Vercel dan dev_server.py agar rutenya tidak pernah
    berbeda antara lokal dan produksi.
    """
    q = parse_qs(kueri)
    nama = q.get("gambar", [None])[0]
    if nama:
        png = gambar(nama, q.get("m_scale", ["1"])[0], q.get("mu_scale", ["1"])[0])
        return png, "image/png", 200
    data = run_sim(
        skenario=q.get("skenario", ["step"])[0],
        kontroler=q.get("kontroler", ["pid"])[0],
        m_scale=q.get("m_scale", ["1"])[0],
        mu_scale=q.get("mu_scale", ["1"])[0],
    )
    return (json.dumps(data, separators=(",", ":")).encode(),
            "application/json; charset=utf-8", 200)


def halaman_utama() -> bytes:
    """Cari index.html di beberapa lokasi; kalau tidak ada, pakai halaman ringkas."""
    akar = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    for kandidat in (os.path.join(akar, "index.html"), "index.html",
                     os.path.join(os.getcwd(), "index.html")):
        try:
            with open(kandidat, "rb") as f:
                return f.read()
        except OSError:
            continue
    return LITE_HTML.encode()


# ======================================================================
# Gambar gaya laporan (matplotlib, dirender saat diminta)
# ======================================================================
MOTOR_N20 = dict(IG=30.0, KT=4.0e-3, KE=4.0e-3, RA=6.0, LA=1.0e-3,
                 JM=1.2e-6, BM=1.0e-7, TAU_C=5.0e-4)
MOTOR_HOT = dict(RA=1.2 * BASE["RA"], KT=0.9 * BASE["KT"], KE=0.9 * BASE["KE"])

KONDISI = [
    ("nominal", dict()),
    ("massa +30%", dict(ms=1.3)),
    ("gesek ×2", dict(mus=2.0)),
    ("Ra +20%\nKt −10%", dict(motor=MOTOR_HOT)),
    ("suplai 5,0 V", dict(v_supply=5.0)),
]


def _plt():
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    plt.rcParams.update({
        "figure.dpi": 140, "font.size": 8.5, "axes.titlesize": 9,
        "axes.grid": True, "grid.alpha": 0.3, "grid.linewidth": 0.6,
        "axes.spines.top": False, "axes.spines.right": False,
        "legend.frameon": False, "legend.fontsize": 7.5, "lines.linewidth": 1.3,
    })
    return plt


def _png(fig):
    import io
    import matplotlib.pyplot as plt
    buf = io.BytesIO()
    fig.savefig(buf, format="png", bbox_inches="tight")
    plt.close(fig)
    return buf.getvalue()


def _jalan(sken, ctrl, ms=1.0, mus=1.0, motor=None, v_supply=None):
    _, ref, t_end, pakai = SKENARIO[sken]
    p = koef(ms, mus, motor=motor, v_supply=v_supply)
    return jalan(ctrl, t_end, ref, pakai, p, h=2.5e-4), p


def _deg(a):
    return [math.degrees(v) for v in a]


def _laju(rec):
    t, psi = rec["t"], rec["psi"]
    w = [0.0] * len(t)
    for i in range(1, len(t) - 1):
        w[i] = (psi[i + 1] - psi[i - 1]) / (t[i + 1] - t[i - 1])
    return w


def g_4_6_step(ms=1.0, mus=1.0):
    plt = _plt()
    rec, p = _jalan("step", "pid", ms, mus)
    fig, ax = plt.subplots(2, 1, figsize=(6.2, 4.4), sharex=True)
    ax[0].plot(rec["t"], rec["v_ref"], "k--", lw=1.0, label="referensi")
    ax[0].plot(rec["t"], rec["v"], label="PID cascade + feedforward")
    ax[0].set_ylabel(r"$v$ [m/s]")
    ax[0].set_title("Respons kecepatan terhadap tangga 0 → 0,35 m/s")
    ax[0].legend(loc="lower right")
    vc = [0.5 * (a + b) for a, b in zip(rec["VL"], rec["VR"])]
    ax[1].plot(rec["t"], vc, color="#d62728", label=r"$V_c$")
    for sgn in (1, -1):
        ax[1].axhline(sgn * p["V_SUPPLY"], ls="--", lw=0.9, color="0.45")
    ax[1].set_ylabel(r"$V_c$ [V]")
    ax[1].set_xlabel("waktu [s]")
    ax[1].set_title("Tegangan kendali common-mode (garis putus-putus = batas 6 V)")
    fig.tight_layout()
    return _png(fig)


def _gambar_tangga_heading(derajat, ms=1.0, mus=1.0):
    plt = _plt()
    r_pid, _ = _jalan("psi%d" % derajat, "pid", ms, mus)
    r_pid1, _ = _jalan("psi%d" % derajat, "pid1", ms, mus)
    fig, ax = plt.subplots(2, 1, figsize=(6.2, 4.6), sharex=True)
    ax[0].plot(r_pid["t"], _deg(r_pid["psi_ref"]), "k--", lw=1.0, label="referensi")
    ax[0].plot(r_pid["t"], _deg(r_pid["psi"]), label="cascade P–PI")
    ax[0].plot(r_pid1["t"], _deg(r_pid1["psi"]), label="PID satu loop")
    ax[0].set_ylabel(r"$\psi$ [derajat]")
    ax[0].set_title("Tangga heading 0 → %d° (rotasi di tempat)" % derajat)
    ax[0].legend(loc="lower right")
    ax[1].plot(r_pid["t"], _laju(r_pid), label="cascade")
    ax[1].plot(r_pid1["t"], _laju(r_pid1), label="PID satu loop")
    ax[1].axhline(3.0, ls="--", lw=0.9, color="0.45")
    ax[1].axhline(-3.0, ls="--", lw=0.9, color="0.45")
    ax[1].set_ylabel(r"$\dot{\psi}$ [rad/s]")
    ax[1].set_xlabel("waktu [s]")
    ax[1].set_title("Laju yaw (batas rancangan 3 rad/s)")
    fig.tight_layout()
    return _png(fig)


def g_4_7_step90(ms=1.0, mus=1.0):
    return _gambar_tangga_heading(90, ms, mus)


def g_4_7_step180(ms=1.0, mus=1.0):
    return _gambar_tangga_heading(180, ms, mus)


def g_5_7_all(ms=1.0, mus=1.0):
    plt = _plt()
    fig, ax = plt.subplots(2, 2, figsize=(8.4, 5.6))
    pasangan = [("step", "Kecepatan: tangga 0,35 m/s", r"$v$ [m/s]"),
                ("gangguan", "Kecepatan: gangguan 1,5 N", r"$v$ [m/s]"),
                ("psi90", "Heading: tangga 90°", r"$\psi$ [derajat]"),
                ("sinus", "Heading: sinusoid 0,2 Hz", r"$\psi$ [derajat]")]
    for a, (sken, judul, ylab) in zip(ax.flat, pasangan):
        r_pid, _ = _jalan(sken, "pid", ms, mus)
        r_sf, _ = _jalan(sken, "sf", ms, mus)
        deg = sken in ("psi90", "sinus")
        f = _deg if deg else (lambda x: x)
        a.plot(r_pid["t"], f(r_pid["psi_ref"] if deg else r_pid["v_ref"]), "k--", lw=0.9)
        a.plot(r_pid["t"], f(r_pid["psi"] if deg else r_pid["v"]), label="PID")
        a.plot(r_sf["t"], f(r_sf["psi"] if deg else r_sf["v"]), label="SF + integral")
        a.set_title(judul)
        a.set_ylabel(ylab)
        a.set_xlabel("waktu [s]")
    ax.flat[0].legend(loc="lower right")
    fig.suptitle("Pengontrol alternatif (SF + integral) dibandingkan PID", y=1.005)
    fig.tight_layout()
    return _png(fig)


def g_6_A_speed(ms=1.0, mus=1.0):
    plt = _plt()
    fig, ax = plt.subplots(2, 3, figsize=(9.4, 5.2))
    pasangan = [("step", "(a) tangga 0,35 m/s"), ("multi", "(b) bertingkat"),
                ("ramp", "(c) ramp 0,4375 m/s²"), ("gangguan", "(d) gangguan 1,5 N"),
                ("rpm500", "(e) 500 rpm (0,0347 m/s)")]
    for k, (sken, judul) in enumerate(pasangan):
        a = ax.flat[k]
        r_pid, _ = _jalan(sken, "pid", ms, mus)
        r_sf, _ = _jalan(sken, "sf", ms, mus)
        a.plot(r_pid["t"], r_pid["v_ref"], "k--", lw=0.9, label="referensi")
        a.plot(r_pid["t"], r_pid["v"], label="PID")
        a.plot(r_sf["t"], r_sf["v"], label="SF")
        a.set_title(judul)
        a.set_xlabel("waktu [s]")
        if k % 3 == 0:
            a.set_ylabel(r"$v$ [m/s]")
    ax.flat[-1].axis("off")
    h, l = ax.flat[0].get_legend_handles_labels()
    fig.legend(h, l, loc="lower center", ncol=3, frameon=False)
    fig.suptitle("Perbandingan PID dan SF pada lima skenario kecepatan", y=1.01)
    fig.tight_layout(rect=(0, 0.05, 1, 1))
    return _png(fig)


def g_6_B_pos(ms=1.0, mus=1.0):
    plt = _plt()
    fig, ax = plt.subplots(2, 2, figsize=(8.4, 5.6))
    pasangan = [("psi90", "(a) tangga 0 → 90°"), ("psi180", "(b) tangga 0 → 180°"),
                ("multi", "(c) bertingkat"), ("sinus", "(d) sinusoid 0,2 Hz")]
    for a, (sken, judul) in zip(ax.flat, pasangan):
        r_pid, _ = _jalan(sken, "pid", ms, mus)
        r_sf, _ = _jalan(sken, "sf", ms, mus)
        a.plot(r_pid["t"], _deg(r_pid["psi_ref"]), "k--", lw=0.9, label="referensi")
        a.plot(r_pid["t"], _deg(r_pid["psi"]), label="PID")
        a.plot(r_sf["t"], _deg(r_sf["psi"]), label="SF + integral")
        a.set_title(judul)
        a.set_ylabel(r"$\psi$ [derajat]")
        a.set_xlabel("waktu [s]")
    ax.flat[0].legend(loc="lower right")
    fig.suptitle("Perbandingan PID dan SF pada empat skenario posisi", y=1.005)
    fig.tight_layout()
    return _png(fig)


def g_6_C_robust(ms=1.0, mus=1.0):
    plt = _plt()
    data = {"PID": [], "SF": []}
    for _, kw in KONDISI:
        for lab, ctrl in (("PID", "pid"), ("SF", "sf")):
            rec, _p = _jalan("step", ctrl, kw.get("ms", 1.0), kw.get("mus", 1.0),
                             motor=kw.get("motor"), v_supply=kw.get("v_supply"))
            m = metrik_tangga(rec["t"], rec["v"], rec["v_ref"], 0.5)
            data[lab].append(m["ts"] if m["ts"] is not None else 0.0)
    fig, ax = plt.subplots(figsize=(7.0, 3.8))
    x = range(len(KONDISI))
    w = 0.36
    ax.bar([i - w / 2 for i in x], data["PID"], w, label="PID", color="#1f77b4")
    ax.bar([i + w / 2 for i in x], data["SF"], w, label="SF + integral", color="#ff7f0e")
    ax.set_xticks(list(x))
    ax.set_xticklabels([n for n, _ in KONDISI], fontsize=7.5)
    ax.set_ylabel(r"$t_s$ kecepatan (2%) [s]")
    ax.set_title("Waktu penetapan kecepatan pada lima kondisi operasi")
    ax.legend(loc="upper left")
    fig.tight_layout()
    return _png(fig)


def g_6_D_motor(ms=1.0, mus=1.0):
    plt = _plt()
    fig, ax = plt.subplots(figsize=(6.6, 3.8))
    for lab, motor, warna in (("TT 48:1", None, "#1f77b4"),
                              ("TT 48:1 (SF)", None, "#1f77b4"),
                              ("Alternatif 1:30", MOTOR_N20, "#d62728"),
                              ("Alternatif 1:30 (SF)", MOTOR_N20, "#d62728")):
        ctrl = "sf" if "(SF)" in lab else "pid"
        rec, _p = _jalan("step", ctrl, ms, mus, motor=motor)
        ax.plot(rec["t"], rec["v"], ls="--" if "(SF)" in lab else "-",
                color=warna, label=lab)
    rec, _p = _jalan("step", "pid", ms, mus)
    ax.plot(rec["t"], rec["v_ref"], "k:", lw=0.9, label="referensi")
    ax.set_xlabel("waktu [s]")
    ax.set_ylabel(r"$v$ [m/s]")
    ax.set_title("Pengaruh karakteristik motor (penguatan tidak diubah)")
    ax.legend(loc="lower right", ncol=2)
    fig.tight_layout()
    return _png(fig)


GAMBAR = {
    "4_6_step": g_4_6_step,
    "4_7_step90": g_4_7_step90,
    "4_7_step180": g_4_7_step180,
    "5_7_all": g_5_7_all,
    "6_A_speed": g_6_A_speed,
    "6_B_pos": g_6_B_pos,
    "6_C_robust": g_6_C_robust,
    "6_D_motor": g_6_D_motor,
}


def gambar(nama, ms=1.0, mus=1.0):
    if nama not in GAMBAR:
        raise ValueError("gambar tidak dikenal: %s" % nama)
    return GAMBAR[nama](float(ms), float(mus))


# ======================================================================
# Handler Vercel
# ======================================================================
class handler(BaseHTTPRequestHandler):
    def _kirim(self, body: bytes, ctype: str, kode: int = 200):
        self.send_response(kode)
        self.send_header("Content-Type", ctype)
        self.send_header("Access-Control-Allow-Origin", "*")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        jalur = urlparse(self.path).path
        if jalur in ("/", "/index.html"):
            self._kirim(halaman_utama(), "text/html; charset=utf-8")
            return
        if not jalur.startswith("/api/simulate"):
            self._kirim(json.dumps({"error": "rute tidak dikenal",
                                    "path": jalur}).encode(), "application/json", 404)
            return
        try:
            body, ctype, kode = api_respon(urlparse(self.path).query)
            self.send_response(kode)
            self.send_header("Content-Type", ctype)
            self.send_header("Access-Control-Allow-Origin", "*")
            self.send_header("Cache-Control", "public, max-age=60")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)
        except Exception as exc:  # noqa: BLE001
            body = json.dumps({"error": str(exc)}).encode()
            self.send_response(400)
            self.send_header("Content-Type", "application/json; charset=utf-8")
            self.send_header("Access-Control-Allow-Origin", "*")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)
