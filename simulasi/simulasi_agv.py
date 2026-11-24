"""
Simulasi AGV line follower 4WD + kontroler PID / state feedback.

Versi SATU BERKAS dan TANPA DEPENDENSI: hanya memakai pustaka standar Python,
sehingga dapat dijalankan di komputer mana pun dengan Python 3.8+:

    python simulasi_agv.py                 # semua skenario, tabel metrik
    python simulasi_agv.py --plot          # + grafik PNG (butuh matplotlib)
    python simulasi_agv.py --skenario step # satu skenario saja
    python simulasi_agv.py --kontroler sf  # pakai state feedback

Dinamika diintegrasikan sendiri dengan Runge-Kutta orde 4; algoritma kontrol
dieksekusi diskret setiap Ts = 10 ms. Tidak ada step(), lsim(), tf(), ss(),
maupun feedback() — sesuai ketentuan teknis tugas.

Referensi: Tugas Proyek 2 TF4107, subbab 4.6, 4.7, 5.7, dan Bab 6.
"""

from __future__ import annotations

import argparse
import math
import os
import sys

# ======================================================================
# 1. PARAMETER FISIK (Tabel 3.2 laporan)
# ======================================================================
M = 2.5            # massa total AGV + payload [kg]
R = 0.0318         # jari-jari roda [m]
B = 0.15           # track width [m]
JZ = 8.70e-3       # inersia yaw body [kg m^2]
JW = 1.204e-5      # inersia satu roda [kg m^2]
JT = 2.0e-6        # inersia transmisi sisi roda [kg m^2]
JM = 1.0e-6        # inersia rotor [kg m^2]
IG = 48.0          # rasio gearbox
ETA = 0.85         # efisiensi transmisi
MU_R = 0.025       # gesek gelinding
G = 9.81           # gravitasi [m/s^2]
THETA = 0.0        # kemiringan lintasan [rad]

RA = 4.0           # resistansi armatur [ohm]
LA = 2.0e-3        # induktansi armatur [H]
KT = 4.265e-3      # konstanta torsi [N m/A]
KE = 4.265e-3      # konstanta back-EMF [V s/rad]
BM = 6.5e-8        # gesek viskos motor [N m s/rad]
TAU_C = 6.0e-4     # gesek Coulomb motor [N m]
V_SUPPLY = 6.0     # tegangan baterai [V]

# --- koefisien turunan (dihitung ulang dari besaran dasar) -------------
J_EQ = JW + JT + IG ** 2 * JM                     # inersia ekuivalen per roda
M_EQ = M + 4.0 * J_EQ / R ** 2                    # massa ekuivalen translasi
J_PSI = JZ + J_EQ * B ** 2 / R ** 2               # inersia yaw ekuivalen
C_F = 2.0 * ETA * IG * KT / R                     # gaya traksi per sisi per A
C_V = 4.0 * ETA * IG ** 2 * BM / R ** 2           # redaman viskos translasi
F_C = 4.0 * ETA * IG * TAU_C / R                  # gesek Coulomb total [N]
F_ROLL = MU_R * M * G                             # gesek gelinding [N]
K_V = KE * IG / R                                 # back-EMF per m/s
K_PSI = KE * IG * B / (2.0 * R)                   # back-EMF per rad/s yaw
C_T = KT * ETA * IG * B / R                       # torsi yaw per A
C_PSI = 2.0 * ETA * IG ** 2 * BM * B ** 2 / R ** 2  # redaman viskos yaw

A_V = (2.0 * C_F * K_V / RA + C_V) / M_EQ         # 1/tau kanal kecepatan
B_V = 2.0 * C_F / (M_EQ * RA)                     # gain kanal kecepatan
A_W = (2.0 * C_T * K_PSI / RA + C_PSI) / J_PSI    # 1/tau kanal yaw
B_W = 2.0 * C_T / (J_PSI * RA)                    # gain kanal yaw

# --- parameter simulasi -------------------------------------------------
TS = 0.010         # periode cuplik kontrol [s]
TD = 0.015         # tunda transportasi [s]
H = 1e-4           # langkah integrasi RK4 [s]
EPS_W, EPS_V = 0.20, 0.005   # pita penghalusan fungsi sign

# --- penguatan final ----------------------------------------------------
# PID: PI kecepatan + feedforward, setpoint weighting beta, cascade heading
KP_V, KI_V, BETA = 17.78, 54.44, 0.70
KP_POS, KP_W, KI_W = 4.0, 2.448, 8.162
KP1, KI1, KD1 = 7.050, 9.400, 1.273
# State feedback + integral
KV_FB, KXI_V, N_FF, VC0 = 27.59, 136.5, 6.530, 0.675
KPSI, KW, KXIPSI = 15.86, 2.154, 31.72


# ======================================================================
# 2. MODEL DINAMIK
# ======================================================================
def sgn_s(x, eps):
    """Aproksimasi kontinu fungsi sign (anti-chattering untuk RK4)."""
    if x > eps:
        return 1.0
    if x < -eps:
        return -1.0
    return x / eps


def turunan(st, VL, VR, F_ext=0.0, tau_ext=0.0):
    """Turunan state [iL, iR, v, w, psi, X, Y] (persamaan E1,E2,M1,M2,K)."""
    iL, iR, v, w, psi, _x, _y = st
    wL = (v - 0.5 * B * w) / R
    wR = (v + 0.5 * B * w) / R

    diL = (VL - RA * iL - KE * IG * wL) / LA
    diR = (VR - RA * iR - KE * IG * wR) / LA

    sg_v = sgn_s(v, EPS_V)
    F_L = (C_F * iL - 0.5 * C_V * (v - 0.5 * B * w)
           - 0.5 * F_C * sgn_s(wL, EPS_W) - 0.5 * F_ROLL * sg_v)
    F_R = (C_F * iR - 0.5 * C_V * (v + 0.5 * B * w)
           - 0.5 * F_C * sgn_s(wR, EPS_W) - 0.5 * F_ROLL * sg_v)

    dv = (F_L + F_R - M * G * math.sin(THETA) - F_ext) / M_EQ
    dw = (0.5 * B * (F_R - F_L) - C_PSI * w - tau_ext) / J_PSI
    return (diL, diR, dv, dw, w, v * math.cos(psi), v * math.sin(psi))


def rk4(st, VL, VR, h, F_ext=0.0, tau_ext=0.0):
    """Satu langkah Runge-Kutta orde 4 (implementasi manual)."""
    iL, iR, v, w, psi, x, y = st
    h2 = 0.5 * h
    k1 = turunan((iL, iR, v, w, psi, x, y), VL, VR, F_ext, tau_ext)
    k2 = turunan((iL + h2 * k1[0], iR + h2 * k1[1], v + h2 * k1[2],
                  w + h2 * k1[3], psi + h2 * k1[4], x + h2 * k1[5],
                  y + h2 * k1[6]), VL, VR, F_ext, tau_ext)
    k3 = turunan((iL + h2 * k2[0], iR + h2 * k2[1], v + h2 * k2[2],
                  w + h2 * k2[3], psi + h2 * k2[4], x + h2 * k2[5],
                  y + h2 * k2[6]), VL, VR, F_ext, tau_ext)
    k4 = turunan((iL + h * k3[0], iR + h * k3[1], v + h * k3[2],
                  w + h * k3[3], psi + h * k3[4], x + h * k3[5],
                  y + h * k3[6]), VL, VR, F_ext, tau_ext)
    h6 = h / 6.0
    return tuple(st[i] + h6 * (k1[i] + 2.0 * k2[i] + 2.0 * k3[i] + k4[i])
                 for i in range(7))


# ======================================================================
# 3. KONTROLER
# ======================================================================
def clamp(x, lo, hi):
    return lo if x < lo else (hi if x > hi else x)


def sgn0(x, eps=1e-6):
    return 1.0 if x > eps else (-1.0 if x < -eps else 0.0)


class KontrolerPID:
    """PI kecepatan + feedforward (dengan setpoint weighting) + cascade heading."""

    def __init__(self, satu_loop=False):
        self.satu_loop = satu_loop
        self.Iv = 0.0      # integrator kecepatan
        self.Ih = 0.0      # integrator heading
        self.df = 0.0      # turunan terfilter (hanya PID satu loop)

    def reset(self):
        self.Iv = self.Ih = self.df = 0.0

    def langkah(self, ref, meas, dt, vmax):
        v_ref, psi_ref = ref.get("v", 0.0), ref.get("psi", 0.0)
        a_ref = ref.get("a", 0.0)
        v, psi, w = meas["v"], meas["psi"], meas["w"]

        # --- kanal kecepatan: PI + feedforward + anti-windup
        e = v_ref - v
        vff = (6.530 * v_ref + 0.675 * sgn0(v_ref) + 2.133 * a_ref)
        u = KP_V * (BETA * v_ref - v) + self.Iv + vff
        Vc = clamp(u, -vmax, vmax)
        self.Iv += (KI_V * e + 25.0 * (Vc - u)) * dt
        self.Iv = clamp(self.Iv, -1.5 * vmax, 1.5 * vmax)

        # --- kanal heading
        lim = vmax - abs(Vc)
        if self.satu_loop:
            e_h = psi_ref - psi
            self.df += dt / (0.020 + dt) * (w - self.df)
            u_h = KP1 * e_h + self.Ih - KD1 * self.df
            Vd = clamp(u_h, -lim, lim)
            self.Ih += (KI1 * e_h + 25.0 * (Vd - u_h)) * dt
        else:
            w_ref = clamp(KP_POS * (psi_ref - psi), -3.0, 3.0)
            e_h = w_ref - w
            u_h = KP_W * e_h + self.Ih
            Vd = clamp(u_h, -lim, lim)
            self.Ih += (KI_W * e_h + 25.0 * (Vd - u_h)) * dt
        self.Ih = clamp(self.Ih, -1.5 * vmax, 1.5 * vmax)

        return Vc - Vd, Vc + Vd, Vc, Vd


class KontrolerSF:
    """State feedback + aksi integral (pole placement), subbab 5.2."""

    def __init__(self):
        self.xiv = 0.0
        self.xip = 0.0

    def reset(self):
        self.xiv = self.xip = 0.0

    def langkah(self, ref, meas, dt, vmax):
        v_ref, psi_ref = ref.get("v", 0.0), ref.get("psi", 0.0)
        v, psi, w = meas["v"], meas["psi"], meas["w"]

        u = -KV_FB * v + KXI_V * self.xiv + N_FF * v_ref + VC0 * sgn0(v_ref)
        Vc = clamp(u, -vmax, vmax)
        self.xiv += (v_ref - v) * dt + (Vc - u) * dt / (0.05 * KXI_V)
        self.xiv = clamp(self.xiv, -20.0, 20.0)

        lim = vmax - abs(Vc)
        u_h = -KPSI * psi - KW * w + KXIPSI * self.xip
        Vd = clamp(u_h, -lim, lim)
        self.xip += (psi_ref - psi) * dt + (Vd - u_h) * dt / (0.05 * KXIPSI)
        self.xip = clamp(self.xip, -5.0, 5.0)

        return Vc - Vd, Vc + Vd, Vc, Vd


# ======================================================================
# 4. SIMULASI CLOSED-LOOP
# ======================================================================
def simulasi(kontroler, t_end, ref, dist=None, vmax=V_SUPPLY):
    """Jalankan satu simulasi; kembalikan rekaman sinyal (dict of lists)."""
    kontroler.reset()
    st = [0.0] * 7
    n = int(round(t_end / TS))
    sub = max(1, int(round(TS / H)))
    hh = TS / sub
    antre = []                      # perintah tertunda: (t_terpasang, VL, VR)
    VL_a = VR_a = 0.0

    rec = {k: [] for k in ("t", "v_ref", "psi_ref", "v", "psi", "w",
                           "iL", "iR", "VL", "VR", "Vc", "Vd", "X", "Y")}

    for k in range(n + 1):
        t = k * TS
        r = ref(t)
        d = (dist(t) if dist else None) or {}
        meas = {"v": st[2], "psi": st[4], "w": st[3]}
        VL, VR, Vc, Vd = kontroler.langkah(r, meas, TS, vmax)

        antre.append((t + TD, VL, VR))
        while antre and antre[0][0] <= t + 1e-12:
            _, VL_a, VR_a = antre.pop(0)

        for key, val in (("t", t), ("v_ref", r.get("v", 0.0)),
                         ("psi_ref", r.get("psi", 0.0)), ("v", st[2]),
                         ("psi", st[4]), ("w", st[3]), ("iL", st[0]),
                         ("iR", st[1]), ("VL", VL_a), ("VR", VR_a),
                         ("Vc", Vc), ("Vd", Vd), ("X", st[5]), ("Y", st[6])):
            rec[key].append(val)

        if k == n:
            break
        for _ in range(sub):
            st = rk4(st, VL_a, VR_a, hh, d.get("F_ext", 0.0), d.get("tau_ext", 0.0))

    return rec


# ======================================================================
# 5. METRIK
# ======================================================================
def metrik_tangga(t, y, ref, t0=0.5, t1=None, tol=0.02):
    """Rise time, settling time 2%, overshoot, galat akhir, IAE.

    t1 dipakai untuk membatasi jendela evaluasi pada referensi bertingkat.
    """
    idx = [i for i, tt in enumerate(t)
           if tt >= t0 - 1e-12 and (t1 is None or tt < t1 - 1e-12)]
    tt = [t[i] - t0 for i in idx]
    yy = [y[i] for i in idx]
    rf = [ref[i] for i in idx]
    y0, yf = yy[0], rf[-1]
    d = yf - y0
    out = {"t_r": None, "t_s": None, "MP": None, "e_ss": abs(yf - yy[-1]), "IAE": None}
    if abs(d) < 1e-9:
        return out

    ys = [(v - y0) / d for v in yy]
    i10 = next((i for i, v in enumerate(ys) if v >= 0.10), None)
    i90 = next((i for i, v in enumerate(ys) if v >= 0.90), None)
    if i10 is not None and i90 is not None:
        out["t_r"] = tt[i90] - tt[i10]
    luar = [i for i, v in enumerate(ys) if abs(v - 1.0) > tol]
    out["t_s"] = tt[min(luar[-1] + 1, len(tt) - 1)] if luar else 0.0
    out["MP"] = max(0.0, (max(ys) - 1.0) * 100.0)
    iae = 0.0
    for i in range(1, len(tt)):
        iae += 0.5 * (abs(yy[i] - rf[i]) + abs(yy[i - 1] - rf[i - 1])) * (tt[i] - tt[i - 1])
    out["IAE"] = iae
    return out


def metrik_tracking(t, y, ref, t0=0.5):
    idx = [i for i, tt in enumerate(t) if tt >= t0 - 1e-12]
    e = [y[i] - ref[i] for i in idx]
    rmse = math.sqrt(sum(v * v for v in e) / max(len(e), 1))
    return {"RMSE": rmse, "max_err": max(abs(v) for v in e)}


# ======================================================================
# 6. SKENARIO
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
        return {"v": 0.35, "a": 0.0}
    if t < 4.8:
        return {"v": max(0.0, 0.35 - a * (t - 4.0)), "a": -a}
    return {"v": 0.0, "a": 0.0}


def ref_psi(deg):
    return lambda t: {"psi": math.radians(deg) if t >= 0.5 else 0.0}


def ref_sin(t):
    if t < 0.5:
        return {"psi": 0.0, "v": 0.20}
    return {"psi": 0.7 * math.sin(2 * math.pi * 0.2 * (t - 0.5)), "v": 0.20}


SKENARIO = {
    "step":    ("kecepatan tangga 0 -> 0,35 m/s", ref_step, 4.0, None),
    "multi":   ("kecepatan bertingkat", ref_multi, 7.0, None),
    "ramp":    ("kecepatan ramp 0,4375 m/s^2", ref_ramp, 6.0, None),
    "gangguan": ("kecepatan + gangguan 1,5 N", ref_step, 6.0,
                 lambda t: {"F_ext": 1.5 if 3.0 <= t < 4.5 else 0.0}),
    "psi90":   ("heading 0 -> 90 deg", ref_psi(90), 5.0, None),
    "psi180":  ("heading 0 -> 180 deg", ref_psi(180), 6.0, None),
    "sinus":   ("heading sinusoid 0,2 Hz", ref_sin, 6.0, None),
}


def pilih_kontroler(nama):
    if nama == "sf":
        return KontrolerSF()
    if nama == "pid1":
        return KontrolerPID(satu_loop=True)
    return KontrolerPID()


# ======================================================================
# 7. MAIN
# ======================================================================
def hitung(key, kontroler):
    """Jalankan satu kombinasi (skenario, kontroler); kembalikan sinyal + metrik."""
    nama, ref, t_end, dist = SKENARIO[key]
    rec = simulasi(pilih_kontroler(kontroler), t_end, ref, dist)
    t = rec["t"]
    if key in ("psi90", "psi180", "sinus"):
        y = [math.degrees(v) for v in rec["psi"]]
        rf = [math.degrees(v) for v in rec["psi_ref"]]
        sat = "deg"
    else:
        y, rf = rec["v"], rec["v_ref"]
        sat = "m/s"

    if key in ("ramp", "sinus"):
        met = metrik_tracking(t, y, rf, 0.5)
    else:
        jendela = {"multi": [(0.5, 2.0), (2.0, 3.5), (3.5, 5.0), (5.0, 7.0)],
                   "gangguan": [(0.5, 3.0)]}.get(key, [(0.5, None)])
        seg = [metrik_tangga(t, y, rf, a, b) for a, b in jendela]
        met = {"t_r": max((x["t_r"] or 0.0) for x in seg),
               "t_s": max((x["t_s"] or 0.0) for x in seg),
               "MP": max(x["MP"] for x in seg),
               "e_ss": seg[-1]["e_ss"],
               "IAE": sum(x["IAE"] for x in seg)}

    vmax = max(max(abs(v) for v in rec["VL"]), max(abs(v) for v in rec["VR"]))
    return {"rec": rec, "y": y, "rf": rf, "sat": sat, "nama": nama,
            "met": met, "vmax": vmax}


def cetak(key, r):
    m, sat = r["met"], r["sat"]
    print(f"\n[{key}] {r['nama']}")
    if "RMSE" in m:
        print(f"  RMSE = {m['RMSE']:.4f} {sat} | galat maks = {m['max_err']:.4f} {sat}")
    else:
        print(f"  t_r = {m['t_r']:.3f} s | t_s (2%) = {m['t_s']:.3f} s | "
              f"MP = {m['MP']:.2f} % | e_ss = {m['e_ss']:.4g} {sat}")
        print(f"  IAE = {m['IAE']:.4f} | |V| maks = {r['vmax']:.2f} V")


LABEL_KONTROLER = {"pid": "PID cascade", "pid1": "PID 1 loop", "sf": "SF + integral"}


def cetak_banding(key, hasil):
    """Tabel perbandingan tiga kontroler untuk satu skenario."""
    r0 = next(iter(hasil.values()))
    m0 = r0["met"]
    print(f"\n[{key}] {r0['nama']}")
    if "RMSE" in m0:
        print(f"  {'kontroler':<14}{'RMSE [' + r0['sat'] + ']':>14}{'galat maks':>14}"
              f"{'|V| maks [V]':>14}")
        print("  " + "-" * 56)
        for c, r in hasil.items():
            print(f"  {LABEL_KONTROLER[c]:<14}{r['met']['RMSE']:>14.4f}"
                  f"{r['met']['max_err']:>14.4f}{r['vmax']:>14.2f}")
    else:
        print(f"  {'kontroler':<14}{'t_r [s]':>9}{'t_s [s]':>9}{'MP [%]':>8}"
              f"{'e_ss':>12}{'IAE':>10}{'|V| [V]':>9}")
        print("  " + "-" * 71)
        for c, r in hasil.items():
            m = r["met"]
            print(f"  {LABEL_KONTROLER[c]:<14}{m['t_r']:>9.3f}{m['t_s']:>9.3f}"
                  f"{m['MP']:>8.2f}{m['e_ss']:>12.3g}{m['IAE']:>10.4f}{r['vmax']:>9.2f}")


def ekspor_csv(r, skenario, kontroler, folder):
    """Tulis deret waktu satu simulasi ke CSV."""
    os.makedirs(folder, exist_ok=True)
    path = os.path.join(folder, f"{skenario}_{kontroler}.csv")
    with open(path, "w", encoding="utf-8", newline="") as f:
        f.write(f"t[s],referensi[{r['sat']}],keluaran[{r['sat']}],VL[V],VR[V]\n")
        for i, t in enumerate(r["rec"]["t"]):
            f.write(f"{t:.4f},{r['rf'][i]:.6f},{r['y'][i]:.6f},"
                    f"{r['rec']['VL'][i]:.4f},{r['rec']['VR'][i]:.4f}\n")
    return path


def ekspor_ringkasan(baris, folder):
    """Tulis tabel metrik gabungan ke CSV."""
    os.makedirs(folder, exist_ok=True)
    path = os.path.join(folder, "ringkasan_metrik.csv")
    kolom = ["skenario", "kontroler", "t_r[s]", "t_s[s]", "MP[%]",
             "e_ss", "IAE", "RMSE", "galat_maks", "V_maks[V]"]
    with open(path, "w", encoding="utf-8", newline="") as f:
        f.write(",".join(kolom) + "\n")
        for b in baris:
            m = b["met"]
            f.write(",".join([
                b["skenario"], b["kontroler"],
                f"{m.get('t_r', float('nan')):.4f}" if m.get("t_r") is not None else "",
                f"{m.get('t_s', float('nan')):.4f}" if m.get("t_s") is not None else "",
                f"{m.get('MP', float('nan')):.3f}" if m.get("t_s") is not None else "",
                f"{m.get('e_ss', float('nan')):.6g}",
                f"{m.get('IAE', float('nan')):.5f}" if m.get("IAE") is not None else "",
                f"{m.get('RMSE', float('nan')):.5f}" if "RMSE" in m else "",
                f"{m.get('max_err', float('nan')):.5f}" if "max_err" in m else "",
                f"{b['vmax']:.3f}" if "vmax" in b else "",
            ]) + "\n")
    return path


def judul():
    print("=" * 78)
    print("Simulasi AGV line follower 4WD - TF4107 Tugas Proyek 2")
    print(f"m_eq = {M_EQ:.3f} kg | J_psi = {J_PSI:.4f} kg.m^2 | c_F = {C_F:.3f} N/A")
    print(f"a_v = {A_V:.3f} 1/s | b_v = {B_V:.4f} | a_w = {A_W:.3f} 1/s | b_w = {B_W:.3f}")
    print(f"Ts = {TS*1000:.0f} ms | tunda = {TD*1000:.0f} ms | RK4 h = {H*1000:.1f} ms")
    print("=" * 78)


def main():
    ap = argparse.ArgumentParser(
        description="Simulasi AGV line follower 4WD (satu berkas, tanpa dependensi).")
    ap.add_argument("--skenario", default="semua", choices=["semua"] + sorted(SKENARIO))
    ap.add_argument("--kontroler", default="pid", choices=["pid", "pid1", "sf"])
    ap.add_argument("--bandingkan", action="store_true",
                    help="bandingkan tiga kontroler pada skenario terpilih")
    ap.add_argument("--csv", default=None, metavar="DIR",
                    help="ekspor hasil ke CSV pada folder DIR")
    ap.add_argument("--plot", action="store_true", help="simpan grafik PNG (butuh matplotlib)")
    ap.add_argument("--outdir", default="hasil_simulasi")
    args = ap.parse_args()

    judul()
    daftar = sorted(SKENARIO) if args.skenario == "semua" else [args.skenario]
    ringkasan = []

    if args.bandingkan:
        for key in daftar:
            hasil = {}
            for k in ("pid", "pid1", "sf"):
                hasil[k] = hitung(key, k)
                ringkasan.append({"skenario": key, "kontroler": k, **hasil[k]})
                if args.csv:
                    ekspor_csv(hasil[k], key, k, args.csv)
            cetak_banding(key, hasil)
            if args.plot:
                for k in ("pid", "sf"):
                    buat_plot(hasil[k]["rec"], f"{key}_{k}", hasil[k]["y"],
                              hasil[k]["rf"], hasil[k]["sat"], args.outdir)
    else:
        for key in daftar:
            r = hitung(key, args.kontroler)
            ringkasan.append({"skenario": key, "kontroler": args.kontroler, **r})
            cetak(key, r)
            if args.csv:
                ekspor_csv(r, key, args.kontroler, args.csv)
            if args.plot:
                buat_plot(r["rec"], key, r["y"], r["rf"], r["sat"], args.outdir)

    if args.csv:
        print("\nCSV ringkasan:", ekspor_ringkasan(ringkasan, args.csv))
    print("\nselesai.")


def buat_plot(rec, key, y, rf, sat, outdir):
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except Exception:
        print("  (matplotlib tidak tersedia, grafik dilewati)")
        return
    os.makedirs(outdir, exist_ok=True)
    fig, ax = plt.subplots(2, 1, figsize=(7.0, 4.6), sharex=True,
                           gridspec_kw={"height_ratios": [2, 1]})
    ax[0].plot(rec["t"], rf, "k--", lw=1.0, label="referensi")
    ax[0].plot(rec["t"], y, label="keluaran")
    ax[0].set_ylabel(f"keluaran [{sat}]")
    ax[0].set_title(f"skenario: {key}")
    ax[0].legend(loc="lower right")
    ax[0].grid(alpha=0.3)
    ax[1].plot(rec["t"], rec["VL"], label="V_L")
    ax[1].plot(rec["t"], rec["VR"], "--", label="V_R")
    ax[1].set_ylabel("tegangan [V]")
    ax[1].set_xlabel("waktu [s]")
    ax[1].legend(loc="lower right")
    ax[1].grid(alpha=0.3)
    fig.tight_layout()
    path = os.path.join(outdir, f"{key}.png")
    fig.savefig(path, dpi=150)
    plt.close(fig)
    print(f"  grafik: {path}")


if __name__ == "__main__":
    main()
