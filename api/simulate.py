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


def koef(m_scale=1.0, mu_scale=1.0):
    """Hitung koefisien turunan; m_scale & mu_scale untuk uji robustness."""
    p = dict(BASE)
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


SKENARIO = {
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
def jalan(kontroler, t_end, ref, pakai_dist, p):
    k = SF() if kontroler == "sf" else PID(satu_loop=(kontroler == "pid1"))
    k.reset()
    st = [0.0] * 7
    n = int(round(t_end / TS))
    sub = max(1, int(round(TS / H)))
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


# ======================================================================
# Handler Vercel
# ======================================================================
class handler(BaseHTTPRequestHandler):
    def do_GET(self):
        try:
            q = parse_qs(urlparse(self.path).query)
            data = run_sim(
                skenario=q.get("skenario", ["step"])[0],
                kontroler=q.get("kontroler", ["pid"])[0],
                m_scale=q.get("m_scale", ["1"])[0],
                mu_scale=q.get("mu_scale", ["1"])[0],
            )
            body = json.dumps(data, separators=(",", ":")).encode()
            self.send_response(200)
            self.send_header("Content-Type", "application/json; charset=utf-8")
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
