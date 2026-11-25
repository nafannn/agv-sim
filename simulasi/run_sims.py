"""Menjalankan seluruh skenario simulasi untuk subbab 4.6, 4.7, 5.7, dan Bab 6.

Keluarannya:
  figs/*.png        grafik siap tempel
  results/*.json    seluruh angka metrik (dipakai untuk menyusun laporan)
  results/tables.md tabel markdown
"""

from __future__ import annotations

import json
import os
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

ROOT = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, ROOT)

from agv import (Plant, PLANT_N20, ControllerPID, ControllerSF, SpeedPI,
                 SpeedSF, HeadingCascade, HeadingPID, HeadingSF, simulate)
from agv.metrics import step_metrics, tracking_metrics, effort_metrics

FIG = os.path.join(ROOT, "figs")
RES = os.path.join(ROOT, "results")
os.makedirs(FIG, exist_ok=True)
os.makedirs(RES, exist_ok=True)

plt.rcParams.update({
    "figure.dpi": 160,
    "savefig.dpi": 160,
    "font.size": 9,
    "axes.grid": True,
    "grid.alpha": 0.3,
    "grid.linewidth": 0.6,
    "axes.spines.top": False,
    "axes.spines.right": False,
    "legend.fontsize": 8,
    "legend.frameon": False,
    "lines.linewidth": 1.6,
})

P = Plant()
C_PID = "C1"      # PID: PI kecepatan + cascade heading
C_PID1 = "C2"     # PID: PI kecepatan + PID satu loop heading
C_SF = "C3"       # State feedback + integral

# Penguatan final (dibahas di 4.6 & 4.7)
Kp_v, Ki_v = 17.78, 54.44         # IMC lambda = 0,12 s
Kp_pos, Kp_w, Ki_w = 4.0, 2.448, 8.162
Kp1, Ki1, Kd1 = 7.050, 9.400, 1.273
KV_SF, KXI_SF, N_SF, VC0_SF = 27.59, 136.5, 6.530, 0.675
KPSI_SF, KW_SF, KXIPSI_SF = 15.86, 2.154, 31.72


def make_ctrl(kind: str, ff: bool = True):
    if kind == C_SF:
        return ControllerSF(SpeedSF(KV_SF, KXI_SF, N_SF, VC0_SF),
                            HeadingSF(KPSI_SF, KW_SF, KXIPSI_SF))
    speed = SpeedPI(Kp=Kp_v, Ki=Ki_v, use_ff=ff)
    if kind == C_PID1:
        return ControllerPID(speed=speed, heading_pid1=HeadingPID(Kp1, Ki1, Kd1))
    return ControllerPID(speed=speed, heading=HeadingCascade(Kp_pos, Kp_w, Ki_w))


# ----------------------------------------------------------------------
# Skenario referensi
# ----------------------------------------------------------------------
def ref_speed_step(t):
    return {"v": 0.35 if t >= 0.5 else 0.0}


def ref_speed_multi(t):
    v = 0.0
    for t_on, val in [(0.5, 0.15), (2.0, 0.25), (3.5, 0.35), (5.0, 0.20)]:
        if t >= t_on:
            v = val
    return {"v": v}


def ref_speed_ramp(t):
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


def ref_speed_low(t):
    # 500 rpm poros motor = 500/48 rpm roda = 0,0347 m/s
    return {"v": 0.0347 if t >= 0.5 else 0.0}


def dist_load(t):
    return {"F_ext": 1.5 if 3.0 <= t < 4.5 else 0.0}


def ref_psi_step(deg):
    return lambda t: {"psi": np.deg2rad(deg) if t >= 0.5 else 0.0}


def ref_psi_multi(t):
    psi = 0.0
    for t_on, val in [(0.5, 0.5), (2.5, -0.5), (4.5, 1.0)]:
        if t >= t_on:
            psi = val
    return {"psi": psi}


def ref_psi_sin(t):
    if t < 0.5:
        return {"psi": 0.0, "v": 0.20}
    return {"psi": 0.7 * np.sin(2 * np.pi * 0.2 * (t - 0.5)), "v": 0.20}


def ref_cruise_psi_step(t):
    # jelajah 0,25 m/s lalu belok 90 derajat
    return {"v": 0.25, "psi": np.deg2rad(90) if t >= 1.0 else 0.0}


def ref_cruise_psi_multi(t):
    psi = 0.0
    for t_on, val in [(1.0, 0.6), (3.0, -0.4), (5.0, 0.8)]:
        if t >= t_on:
            psi = val
    return {"v": 0.25, "psi": psi}


def run(ctrl_kind, ref, t_end, dist=None, p=P, ff=True, noise=None, seed=1):
    return simulate(p, make_ctrl(ctrl_kind, ff), t_end, ref, dist_fn=dist,
                    noise=noise, seed=seed)


def m_step(tr, key, ref_key, t0, tol=0.02):
    m = step_metrics(tr.t, tr[key], tr[ref_key], t0, tol=tol)
    return m


def deg(x):
    return np.rad2deg(x)


def sin_lag(tr, amp):
    """Selisih waktu puncak sinyal terhadap referensi + rasio amplitudo."""
    m = tr.t >= 1.0
    psi = np.where(m, tr.psi, -9.0)
    ref = np.where(m, tr.psi_ref, -9.0)
    lag = float(tr.t[np.argmax(psi)] - tr.t[np.argmax(ref)])
    return {"lag_s": lag, "amp_ratio": float(np.max(psi) / amp)}


# ----------------------------------------------------------------------
# Kumpulan hasil
# ----------------------------------------------------------------------
out = {"params": {k: float(v) for k, v in P.summary().items()},
       "gains": {"Kp_v": Kp_v, "Ki_v": Ki_v, "Kp_pos": Kp_pos, "Kp_w": Kp_w,
                 "Ki_w": Ki_w, "Kp1": Kp1, "Ki1": Ki1, "Kd1": Kd1,
                 "SF": [KV_SF, KXI_SF, N_SF, VC0_SF, KPSI_SF, KW_SF, KXIPSI_SF]},
       "s46": {}, "s47": {}, "s57": {}, "s6": {}}


def rec_step(tr, key, ref_key, t0, tol=0.02):
    m = m_step(tr, key, ref_key, t0, tol)
    e = effort_metrics(tr.t, tr.VL, tr.VR)
    return {k: (None if v is None else float(v)) for k, v in m.items()
            if k in ("rise_time", "settling_time", "overshoot_pct",
                     "error_final", "error_rel_pct", "iae", "rmse", "peak")} | e


def multi_metrics(t, y, ref, segs, tol=0.02):
    """Ringkas metrik referensi bertingkat: nilai terburuk per segmen."""
    res = [step_metrics(t, y, ref, a, tol=tol, t_stop=b) for a, b in segs]
    ts = [r["settling_time"] for r in res if r["settling_time"] is not None]
    return {
        "ts_max": float(max(ts)) if ts else None,
        "os_max": float(max(r["overshoot_pct"] for r in res)),
        "ess_end": float(res[-1]["error_final"]),
        "iae": float(sum(r["iae"] for r in res)),
        "per_step": [{k: (None if v is None else float(v)) for k, v in r.items()
                      if k in ("settling_time", "overshoot_pct", "error_final")}
                     for r in res],
    }


def eval_speed(tr, name):
    if name == "multi":
        segs = [(0.5, 2.0), (2.0, 3.5), (3.5, 5.0), (5.0, 7.0)]
        m = multi_metrics(tr.t, tr.v, tr.v_ref, segs)
        m["settling_time"] = m.pop("ts_max")
        m["overshoot_pct"] = m.pop("os_max")
        m["error_final"] = m.pop("ess_end")
        return m
    if name == "ramp":
        return tracking_metrics(tr.t, tr.v, tr.v_ref, 0.5)
    if name == "load":
        m = step_metrics(tr.t, tr.v, tr.v_ref, 0.5, t_stop=3.0)
        m["drop_pct"] = float(100 * (0.35 - np.min(tr.v[(tr.t > 3.0) & (tr.t < 4.5)])) / 0.35)
        return m
    return step_metrics(tr.t, tr.v, tr.v_ref, 0.5)


def eval_pos(tr, name):
    if name == "multi":
        segs = [(1.0, 3.0), (3.0, 5.0), (5.0, 7.0)]
        m = multi_metrics(tr.t, deg(tr.psi), deg(tr.psi_ref), segs)
        m["settling_time"] = m.pop("ts_max")
        m["overshoot_pct"] = m.pop("os_max")
        m["error_final"] = m.pop("ess_end")
        return m
    if name == "sin":
        return tracking_metrics(tr.t, deg(tr.psi), deg(tr.psi_ref), 1.0)
    return step_metrics(tr.t, deg(tr.psi), deg(tr.psi_ref), 1.0)


# ======================================================================
# 4.6  Kontrol kecepatan (PID + feedforward)
# ======================================================================
import time as _time

_T0 = _time.time()
_TAHAP = {"nama": "", "t": _T0}


def tahap(nama):
    """Mulai satu tahap; tahap sebelumnya ditutup dengan durasinya."""
    _tutup()
    _TAHAP["nama"], _TAHAP["t"] = nama, _time.time()
    print(f"  {nama:38s} ... ", end="", flush=True)


def _tutup():
    if _TAHAP["nama"]:
        print(f"selesai ({_time.time() - _TAHAP['t']:.1f} s)")
        _TAHAP["nama"] = ""


def ok(catatan=""):
    if _TAHAP["nama"]:
        print(f"selesai ({_time.time() - _TAHAP['t']:.1f} s){(' — ' + catatan) if catatan else ''}")
        _TAHAP["nama"] = ""


import argparse as _argparse

BAGIAN = [
    ("4.6", "kontrol kecepatan"),
    ("4.7", "kontrol posisi heading"),
    ("5.7", "state feedback"),
    ("6A", "perbandingan kecepatan"),
    ("6B", "perbandingan posisi"),
    ("6C", "robustness"),
    ("6D", "variasi motor"),
]
_KUNCI = {k for k, _ in BAGIAN}


def _menu():
    print()
    print("Bagian yang tersedia:")
    for i, (k, n) in enumerate(BAGIAN, 1):
        print(f"  {i}) {k:4s} {n}")
    print("  a) semua")
    try:
        jawab = input("\nPilih nomor atau kode (pisah koma, Enter = semua): ").strip()
    except EOFError:
        print()
        print("(tidak ada input; menjalankan semua)")
        return set(_KUNCI)
    if not jawab or jawab.lower() in ("a", "semua", "all"):
        return set(_KUNCI)
    pilih = set()
    for b in jawab.replace(" ", "").split(","):
        if b.isdigit() and 1 <= int(b) <= len(BAGIAN):
            pilih.add(BAGIAN[int(b) - 1][0])
        elif b.upper() in _KUNCI:
            pilih.add(b.upper())
        else:
            print(f"  lewati: {b}")
    return pilih or set(_KUNCI)


def _tentukan_pilihan():
    ap = _argparse.ArgumentParser(
        description="Simulasi AGV: jalankan semua bagian atau pilih beberapa.",
        epilog="Tanpa argumen: menu pilihan. Contoh lain: python run_sims.py --bagian 4.6,6A | python run_sims.py --semua")
    ap.add_argument("--bagian", metavar="KODE", help="dipisah koma: 4.6,4.7,5.7,6A,6B,6C,6D")
    ap.add_argument("--semua", action="store_true", help="jalankan semua bagian tanpa menu")
    ap.add_argument("--pilih", action="store_true", help="tanya interaktif bagian mana (default)")
    ap.add_argument("--daftar", action="store_true", help="tampilkan daftar bagian lalu keluar")
    a = ap.parse_args()
    if a.daftar:
        print("Bagian yang tersedia:")
        for k, n in BAGIAN:
            print(f"  {k:4s} {n}")
        raise SystemExit(0)
    if a.semua:
        return set(_KUNCI)
    if a.pilih or not a.bagian:
        return _menu()
    if a.bagian:
        pilih = {b.strip().upper() for b in a.bagian.split(",") if b.strip()}
        salah = pilih - _KUNCI
        if salah:
            raise SystemExit("kode tidak dikenal: " + ", ".join(sorted(salah))
                             + " | pilihan: " + ", ".join(sorted(_KUNCI)))
        return pilih
    return set(_KUNCI)


PILIH = _tentukan_pilihan()

# beberapa bagian memakai hasil bagian lain; ikut dijalankan bila belum dipilih
_BUTUH = {
    "5.7": {"4.6", "4.7"},   # gambar 5.7.1 memplot ulang respons 4.6 dan 4.7
    "6A": {"4.6", "5.7"},    # panel (a) memakai tangga 4.6 dan kurva SF dari 5.7
    "6D": {"4.6"},           # membandingkan kurva TT 48:1 dengan alternatif
}
_diminta = set(PILIH)
_berubah = True
while _berubah:
    _berubah = False
    for _k in list(PILIH):
        _tambah = _BUTUH.get(_k, set()) - PILIH
        if _tambah:
            PILIH |= _tambah
            _berubah = True
_ikut = sorted(PILIH - _diminta)
if _ikut:
    print(f"  ikut dijalankan karena dipakai bagian lain: {', '.join(_ikut)}")

_PENUH = PILIH == _KUNCI
_NAMA_JSON = "results.json" if _PENUH else "results_bagian_" + "_".join(sorted(PILIH)) + ".json"

print("=" * 78)
print("Simulasi AGV line follower 4WD - TF4107 Tugas Proyek 2")
print("Bagian: " + ", ".join(k for k, _ in BAGIAN if k in PILIH)
      + (" (semua)" if _PENUH else ""))
print(f"Keluaran: {os.path.abspath(FIG)}/ dan {os.path.abspath(os.path.join(RES, _NAMA_JSON))}")
print("=" * 78)
print()

if "4.6" in PILIH:
    tahap("4.6 kontrol kecepatan")
    tr_ff = run(C_PID, ref_speed_step, 4.0)
    tr_noff = run(C_PID, ref_speed_step, 4.0, ff=False)
    tr_lam20 = simulate(P, ControllerPID(speed=SpeedPI(Kp=10.66, Ki=32.65)), 4.0,
                        ref_speed_step)
    out["s46"]["step_ff"] = rec_step(tr_ff, "v", "v_ref", 0.5)
    out["s46"]["step_noff"] = rec_step(tr_noff, "v", "v_ref", 0.5)
    out["s46"]["step_lambda020"] = rec_step(tr_lam20, "v", "v_ref", 0.5)

    fig, ax = plt.subplots(2, 1, figsize=(7.0, 4.8), sharex=True,
                           gridspec_kw={"height_ratios": [2, 1]})
    ax[0].plot(tr_noff.t, tr_noff.v_ref, "k--", lw=1.0, label="referensi")
    ax[0].plot(tr_noff.t, tr_noff.v, label="tanpa feedforward")
    ax[0].plot(tr_ff.t, tr_ff.v, label="dengan feedforward")
    ax[0].set_ylabel(r"$v$ [m/s]")
    ax[0].set_title("Respons kecepatan: tangga 0 → 0,35 m/s")
    ax[0].legend(loc="lower right")
    ax[1].plot(tr_ff.t, tr_ff.Vc, label=r"$V_c$ (feedforward)")
    ax[1].plot(tr_noff.t, tr_noff.Vc, "--", label=r"$V_c$ (tanpa FF)")
    ax[1].axhline(6.0, color="r", ls=":", lw=1.0, label="saturasi 6 V")
    ax[1].set_ylabel(r"$V_c$ [V]")
    ax[1].set_xlabel("waktu [s]")
    ax[1].legend(loc="lower right")
    ax[1].set_ylim(-0.5, 7)
    fig.tight_layout()
    fig.savefig(os.path.join(FIG, "4_6_step.png"))
    plt.close(fig)

    tr_multi = run(C_PID, ref_speed_multi, 7.0)
    mm = multi_metrics(tr_multi.t, tr_multi.v, tr_multi.v_ref,
                       [(0.5, 2.0), (2.0, 3.5), (3.5, 5.0), (5.0, 7.0)])
    out["s46"]["multi"] = mm

    # pembanding: tanpa setpoint weighting (beta = 1), prefilter dimatikan
    tr_multi_np = simulate(P, ControllerPID(speed=SpeedPI(Kp=Kp_v, Ki=Ki_v, beta=1.0)),
                           7.0, ref_speed_multi)
    out["s46"]["multi_beta1"] = multi_metrics(tr_multi_np.t, tr_multi_np.v,
                                              tr_multi_np.v_ref,
                                              [(0.5, 2.0), (2.0, 3.5), (3.5, 5.0), (5.0, 7.0)])
    tr_step_np = simulate(P, ControllerPID(speed=SpeedPI(Kp=Kp_v, Ki=Ki_v, beta=1.0)),
                          4.0, ref_speed_step)
    out["s46"]["step_beta1"] = rec_step(tr_step_np, "v", "v_ref", 0.5)

    fig, ax = plt.subplots(2, 1, figsize=(7.0, 4.8), sharex=True)
    ax[0].plot(tr_multi.t, tr_multi.v_ref, "k--", lw=1.0, label="referensi")
    ax[0].plot(tr_multi.t, tr_multi.v, label=r"$v$")
    ax[0].set_ylabel(r"$v$ [m/s]")
    ax[0].set_title("Referensi kecepatan bertingkat")
    ax[0].legend(loc="lower right")
    ax[1].plot(tr_multi.t, tr_multi.iL, label=r"$i_L$")
    ax[1].plot(tr_multi.t, tr_multi.iR, "--", label=r"$i_R$")
    ax[1].set_ylabel("arus [A]")
    ax[1].set_xlabel("waktu [s]")
    ax[1].legend(loc="upper right")
    fig.tight_layout()
    fig.savefig(os.path.join(FIG, "4_6_multistep.png"))
    plt.close(fig)

    tr_ramp = run(C_PID, ref_speed_ramp, 6.0)
    out["s46"]["ramp"] = tracking_metrics(tr_ramp.t, tr_ramp.v, tr_ramp.v_ref, 0.5)
    out["s46"]["ramp"]["max_abs_err"] = float(np.max(np.abs(tr_ramp.v - tr_ramp.v_ref)))

    fig, ax = plt.subplots(2, 1, figsize=(7.0, 4.8), sharex=True,
                           gridspec_kw={"height_ratios": [2, 1]})
    ax[0].plot(tr_ramp.t, tr_ramp.v_ref, "k--", lw=1.0, label="referensi ramp")
    ax[0].plot(tr_ramp.t, tr_ramp.v, label=r"$v$")
    ax[0].set_ylabel(r"$v$ [m/s]")
    ax[0].set_title("Pelacakan referensi ramp (0,4375 m/s²) dan ramp turun")
    ax[0].legend(loc="upper right")
    ax[1].plot(tr_ramp.t, 1000 * (tr_ramp.v - tr_ramp.v_ref))
    ax[1].set_ylabel("galat [mm/s]")
    ax[1].set_xlabel("waktu [s]")
    fig.tight_layout()
    fig.savefig(os.path.join(FIG, "4_6_ramp.png"))
    plt.close(fig)

    tr_dist = run(C_PID, ref_speed_step, 6.0, dist=dist_load)
    out["s46"]["dist"] = {
        "drop": float(np.min(tr_dist.v[(tr_dist.t > 3.0) & (tr_dist.t < 4.5)])),
    }
    for tag, tol in [("1pct", 0.0035), ("0p3pct", 0.001)]:
        out["s46"]["dist"][f"recovery_{tag}"] = float(
            next((tr_dist.t[i] for i in range(len(tr_dist.t))
                  if tr_dist.t[i] > 4.5 and abs(tr_dist.v[i] - 0.35) < tol), np.nan) - 4.5)
    out["s46"]["dist"]["drop_pct"] = 100.0 * (0.35 - out["s46"]["dist"]["drop"]) / 0.35

    fig, ax = plt.subplots(2, 1, figsize=(7.0, 4.8), sharex=True)
    ax[0].plot(tr_dist.t, tr_dist.v_ref, "k--", lw=1.0, label="referensi")
    ax[0].plot(tr_dist.t, tr_dist.v, label=r"$v$")
    ax[0].axvspan(3.0, 4.5, color="tab:red", alpha=0.08, label=r"gangguan $F_{ext}=1,5$ N")
    ax[0].set_ylabel(r"$v$ [m/s]")
    ax[0].set_title("Penolakan gangguan beban luar")
    ax[0].legend(loc="lower right")
    ax[1].plot(tr_dist.t, tr_dist.Vc)
    ax[1].set_ylabel(r"$V_c$ [V]")
    ax[1].set_xlabel("waktu [s]")
    fig.tight_layout()
    fig.savefig(os.path.join(FIG, "4_6_dist.png"))
    plt.close(fig)

    # kasus 500 rpm motor
    tr_low = run(C_PID, ref_speed_low, 5.0)
    out["s46"]["low_speed"] = rec_step(tr_low, "v", "v_ref", 0.5)

    # ======================================================================
    # 4.7  Kontrol posisi (heading): cascade vs PID satu loop
    # ======================================================================

if "4.7" in PILIH:
    tahap("4.7 kontrol posisi heading")
    tr90_c = run(C_PID, ref_psi_step(90), 5.0)
    tr90_p = run(C_PID1, ref_psi_step(90), 5.0)
    tr180_c = run(C_PID, ref_psi_step(180), 6.0)
    tr180_p = run(C_PID1, ref_psi_step(180), 6.0)
    trmult_c = run(C_PID, ref_psi_multi, 7.0)
    trmult_p = run(C_PID1, ref_psi_multi, 7.0)
    trsin_c = run(C_PID, ref_psi_sin, 6.0)
    trsin_p = run(C_PID1, ref_psi_sin, 6.0)

    for name, tr in [("cascade_90", tr90_c), ("pid1_90", tr90_p),
                     ("cascade_180", tr180_c), ("pid1_180", tr180_p)]:
        key = "psi"
        m = step_metrics(tr.t, deg(tr[key]), deg(tr.psi_ref), 0.5, tol=0.02)
        out["s47"][name] = {k: (None if v is None else float(v)) for k, v in m.items()
                            if k in ("rise_time", "settling_time", "overshoot_pct",
                                     "error_final", "iae", "rmse")}
        out["s47"][name].update(effort_metrics(tr.t, tr.VL, tr.VR))
        out["s47"][name]["psi_final_deg"] = float(deg(tr.psi[-1]))
        out["s47"][name]["w_max"] = float(np.max(np.abs(tr.w)))

    for name, tr in [("cascade_sin", trsin_c), ("pid1_sin", trsin_p)]:
        out["s47"][name] = tracking_metrics(tr.t, deg(tr.psi), deg(tr.psi_ref), 1.0)
        out["s47"][name]["max_abs_err_deg"] = float(np.max(np.abs(deg(tr.psi - tr.psi_ref))))
        out["s47"][name].update(sin_lag(tr, 0.7))

    for a, b, key in [(0.5, 2.5, "s1"), (2.5, 4.5, "s2"), (4.5, 7.0, "s3")]:
        m_c = step_metrics(trmult_c.t, deg(trmult_c.psi), deg(trmult_c.psi_ref), a,
                           t_stop=b)
        m_p = step_metrics(trmult_p.t, deg(trmult_p.psi), deg(trmult_p.psi_ref), a,
                           t_stop=b)
        out["s47"][f"multi_{key}_cascade"] = {k: (None if v is None else float(v))
                                             for k, v in m_c.items()
                                             if k in ("settling_time", "overshoot_pct", "error_final")}
        out["s47"][f"multi_{key}_pid1"] = {k: (None if v is None else float(v))
                                           for k, v in m_p.items()
                                           if k in ("settling_time", "overshoot_pct", "error_final")}


    def plot_heading(trs, labels, fname, title, deg_ylim=None):
        fig, ax = plt.subplots(2, 1, figsize=(7.0, 4.8), sharex=True,
                               gridspec_kw={"height_ratios": [2, 1]})
        ax[0].plot(trs[0].t, deg(trs[0].psi_ref), "k--", lw=1.0, label="referensi")
        for tr, lab in zip(trs, labels):
            ax[0].plot(tr.t, deg(tr.psi), label=lab)
        ax[0].set_ylabel(r"$\psi$ [derajat]")
        ax[0].set_title(title)
        ax[0].legend(loc="lower right")
        if deg_ylim:
            ax[0].set_ylim(*deg_ylim)
        for tr, lab in zip(trs, labels):
            ax[1].plot(tr.t, tr.w, label=lab)
        ax[1].set_ylabel(r"$\dot\psi$ [rad/s]")
        ax[1].set_xlabel("waktu [s]")
        fig.tight_layout()
        fig.savefig(os.path.join(FIG, fname))
        plt.close(fig)


    plot_heading([tr90_c, tr90_p], ["cascade (P + PI)", "PID satu loop"],
                 "4_7_step90.png", "Tangga heading 0 → 90°")
    plot_heading([tr180_c, tr180_p], ["cascade (P + PI)", "PID satu loop"],
                 "4_7_step180.png", "Tangga heading 0 → 180°")
    plot_heading([trmult_c, trmult_p], ["cascade (P + PI)", "PID satu loop"],
                 "4_7_multistep.png", "Referensi heading bertingkat")

    fig, ax = plt.subplots(2, 1, figsize=(7.0, 4.8), sharex=True,
                           gridspec_kw={"height_ratios": [2, 1]})
    ax[0].plot(trsin_c.t, deg(trsin_c.psi_ref), "k--", lw=1.0, label="referensi")
    ax[0].plot(trsin_c.t, deg(trsin_c.psi), label="cascade (P + PI)")
    ax[0].plot(trsin_p.t, deg(trsin_p.psi), label="PID satu loop")
    ax[0].set_ylabel(r"$\psi$ [derajat]")
    ax[0].set_title("Pelacakan trayektori sinusoidal 0,2 Hz")
    ax[0].legend(loc="lower right")
    ax[1].plot(trsin_c.t, deg(trsin_c.psi - trsin_c.psi_ref), label="cascade")
    ax[1].plot(trsin_p.t, deg(trsin_p.psi - trsin_p.psi_ref), label="PID satu loop")
    ax[1].set_ylabel("galat [derajat]")
    ax[1].set_xlabel("waktu [s]")
    ax[1].legend()
    fig.tight_layout()
    fig.savefig(os.path.join(FIG, "4_7_sinus.png"))
    plt.close(fig)

    # ======================================================================
    # 5.7  Pengontrol alternatif: state feedback + integral
    # ======================================================================

if "5.7" in PILIH:
    tahap("5.7 state feedback")
    trsf_step = run(C_SF, ref_speed_step, 4.0)
    trsf_dist = run(C_SF, ref_speed_step, 6.0, dist=dist_load)
    trsf90 = run(C_SF, ref_psi_step(90), 5.0)
    trsf_sin = run(C_SF, ref_psi_sin, 6.0)

    out["s57"]["speed_step"] = rec_step(trsf_step, "v", "v_ref", 0.5)
    out["s57"]["speed_dist"] = {
        "drop": float(np.min(trsf_dist.v[(trsf_dist.t > 3.0) & (trsf_dist.t < 4.5)])),
    }
    out["s57"]["speed_dist"]["drop_pct"] = 100 * (0.35 - out["s57"]["speed_dist"]["drop"]) / 0.35
    m = step_metrics(trsf90.t, deg(trsf90.psi), deg(trsf90.psi_ref), 0.5)
    out["s57"]["heading90"] = {k: (None if v is None else float(v)) for k, v in m.items()
                               if k in ("rise_time", "settling_time", "overshoot_pct",
                                        "error_final", "iae", "rmse")}
    out["s57"]["heading90"].update(effort_metrics(trsf90.t, trsf90.VL, trsf90.VR))
    out["s57"]["sin"] = tracking_metrics(trsf_sin.t, deg(trsf_sin.psi), deg(trsf_sin.psi_ref), 1.0)
    out["s57"]["sin"]["max_abs_err_deg"] = float(np.max(np.abs(deg(trsf_sin.psi - trsf_sin.psi_ref))))
    out["s57"]["sin"].update(sin_lag(trsf_sin, 0.7))

    fig, ax = plt.subplots(2, 2, figsize=(7.2, 5.2))
    ax[0, 0].plot(tr_ff.t, tr_ff.v_ref, "k--", lw=1.0)
    ax[0, 0].plot(tr_ff.t, tr_ff.v, label="PID")
    ax[0, 0].plot(trsf_step.t, trsf_step.v, label="SF + integral")
    ax[0, 0].set_title("(a) tangga kecepatan")
    ax[0, 0].set_ylabel(r"$v$ [m/s]")
    ax[0, 0].legend(loc="lower right")
    ax[0, 1].plot(tr_dist.t, tr_dist.v_ref, "k--", lw=1.0)
    ax[0, 1].plot(tr_dist.t, tr_dist.v, label="PID")
    ax[0, 1].plot(trsf_dist.t, trsf_dist.v, label="SF + integral")
    ax[0, 1].axvspan(3.0, 4.5, color="tab:red", alpha=0.08)
    ax[0, 1].set_title("(b) gangguan beban 1,5 N")
    ax[0, 1].legend(loc="lower right")
    ax[1, 0].plot(tr90_c.t, deg(tr90_c.psi_ref), "k--", lw=1.0)
    ax[1, 0].plot(tr90_c.t, deg(tr90_c.psi), label="PID cascade")
    ax[1, 0].plot(trsf90.t, deg(trsf90.psi), label="SF + integral")
    ax[1, 0].set_title("(c) tangga heading 90°")
    ax[1, 0].set_ylabel(r"$\psi$ [derajat]")
    ax[1, 0].set_xlabel("waktu [s]")
    ax[1, 0].legend(loc="lower right")
    ax[1, 1].plot(trsin_c.t, deg(trsin_c.psi_ref), "k--", lw=1.0)
    ax[1, 1].plot(trsin_c.t, deg(trsin_c.psi), label="PID cascade")
    ax[1, 1].plot(trsf_sin.t, deg(trsf_sin.psi), label="SF + integral")
    ax[1, 1].set_title("(d) sinusoid 0,2 Hz")
    ax[1, 1].set_xlabel("waktu [s]")
    ax[1, 1].legend(loc="lower right")
    fig.tight_layout()
    fig.savefig(os.path.join(FIG, "5_7_all.png"))
    plt.close(fig)

    # ======================================================================
    # 6A  Pengujian kontrol kecepatan: PID vs SF
    # ======================================================================

if "6A" in PILIH:
    tahap("6A perbandingan kecepatan")
    scen_speed = [
        ("step", ref_speed_step, 4.0, None),
        ("multi", ref_speed_multi, 7.0, None),
        ("ramp", ref_speed_ramp, 6.0, None),
        ("load", ref_speed_step, 6.0, dist_load),
        ("rpm500", ref_speed_low, 5.0, None),
    ]
    runs_speed = {}
    out["s6"]["A"] = {}
    for name, ref, t_end, dist in scen_speed:
        tr_p = tr_ff if name == "step" else run(C_PID, ref, t_end, dist=dist)
        tr_s = trsf_step if name == "step" else run(C_SF, ref, t_end, dist=dist)
        runs_speed[name] = (tr_p, tr_s)
        m_p = eval_speed(tr_p, name)
        m_s = eval_speed(tr_s, name)
        out["s6"]["A"][name] = {
            "PID": {k: (None if v is None else float(v)) for k, v in m_p.items()
                    if k in ("rise_time", "settling_time", "overshoot_pct", "error_final",
                             "iae", "rmse")} | effort_metrics(tr_p.t, tr_p.VL, tr_p.VR),
            "SF": {k: (None if v is None else float(v)) for k, v in m_s.items()
                   if k in ("rise_time", "settling_time", "overshoot_pct", "error_final",
                            "iae", "rmse")} | effort_metrics(tr_s.t, tr_s.VL, tr_s.VR),
        }

    fig, ax = plt.subplots(2, 3, figsize=(9.8, 5.7))
    titles = {"step": "(a) tangga 0 → 0,35 m/s", "multi": "(b) referensi bertingkat",
              "ramp": "(c) referensi ramp", "load": "(d) gangguan beban 1,5 N",
              "rpm500": "(e) 500 rpm motor (0,0347 m/s)"}
    for k, (name, _, _, _) in enumerate(scen_speed):
        a = ax.flat[k]
        tr_p, tr_s = runs_speed[name]
        a.plot(tr_p.t, tr_p.v_ref, "k--", lw=1.0, label="referensi")
        a.plot(tr_p.t, tr_p.v, label="PID")
        a.plot(tr_s.t, tr_s.v, label="SF + integral")
        a.set_title(titles[name])
        a.set_xlabel("waktu [s]")
        if k % 3 == 0:
            a.set_ylabel(r"$v$ [m/s]")
    for j in range(len(scen_speed), ax.size):
        ax.flat[j].axis("off")
    h_, l_ = ax.flat[0].get_legend_handles_labels()
    fig.legend(h_, l_, loc="lower center", ncol=3, fontsize=8.5, frameon=False)
    fig.tight_layout(rect=(0, 0.045, 1, 1))
    fig.savefig(os.path.join(FIG, "6_A_speed.png"))
    plt.close(fig)

    # ======================================================================
    # 6B  Pengujian kontrol posisi: PID vs SF
    # ======================================================================

if "6B" in PILIH:
    tahap("6B perbandingan posisi")
    scen_pos = [
        ("step90", lambda t: {"psi": np.deg2rad(90) if t >= 1.0 else 0.0, "v": 0.25}, 5.0),
        ("step180", lambda t: {"psi": np.deg2rad(180) if t >= 1.0 else 0.0, "v": 0.25}, 6.0),
        ("multi", ref_cruise_psi_multi, 7.0),
        ("sin", ref_psi_sin, 6.0),
    ]
    runs_pos = {}
    out["s6"]["B"] = {}
    for name, ref, t_end in scen_pos:
        tr_p = run(C_PID, ref, t_end)
        tr_s = run(C_SF, ref, t_end)
        runs_pos[name] = (tr_p, tr_s)
        m_p = eval_pos(tr_p, name)
        m_s = eval_pos(tr_s, name)
        out["s6"]["B"][name] = {
            "PID": {k: (None if v is None else float(v)) for k, v in m_p.items()
                    if k in ("rise_time", "settling_time", "overshoot_pct", "error_final",
                             "iae", "rmse")} | effort_metrics(tr_p.t, tr_p.VL, tr_p.VR),
            "SF": {k: (None if v is None else float(v)) for k, v in m_s.items()
                   if k in ("rise_time", "settling_time", "overshoot_pct", "error_final",
                            "iae", "rmse")} | effort_metrics(tr_s.t, tr_s.VL, tr_s.VR),
        }

    fig, ax = plt.subplots(2, 2, figsize=(7.2, 5.4))
    titles_p = {"step90": "(a) 0 → 90° saat jelajah", "step180": "(b) 0 → 180°",
                "multi": "(c) referensi bertingkat", "sin": "(d) trayektori sinusoid"}
    for k, (name, _, _) in enumerate(scen_pos):
        a = ax.flat[k]
        tr_p, tr_s = runs_pos[name]
        a.plot(tr_p.t, deg(tr_p.psi_ref), "k--", lw=1.0, label="referensi")
        a.plot(tr_p.t, deg(tr_p.psi), label="PID")
        a.plot(tr_s.t, deg(tr_s.psi), label="SF + integral")
        a.set_title(titles_p[name])
        a.set_xlabel("waktu [s]")
        if k % 2 == 0:
            a.set_ylabel(r"$\psi$ [derajat]")
    h_, l_ = ax.flat[0].get_legend_handles_labels()
    fig.legend(h_, l_, loc="lower center", ncol=3, fontsize=8.5, frameon=False)
    fig.tight_layout(rect=(0, 0.05, 1, 1))
    fig.savefig(os.path.join(FIG, "6_B_pos.png"))
    plt.close(fig)

    # ======================================================================
    # 6C  Robustness
    # ======================================================================

if "6C" in PILIH:
    tahap("6C robustness")
    perturb = [
        ("nominal", P),
        ("massa+inersia +30%", P.scaled(m=1.3 * P.m, Jw=1.3 * P.Jw, Jt=1.3 * P.Jt, Jm=1.3 * P.Jm)),
        ("gesek x2", P.scaled(Bm=2 * P.Bm, tau_c=2 * P.tau_c, mu_r=2 * P.mu_r)),
        ("Ra +20%, Kt -10%", P.scaled(Ra=1.2 * P.Ra, Kt=0.9 * P.Kt, Ke=0.9 * P.Ke)),
        ("baterai 5,0 V", P.scaled(V_supply=5.0)),
    ]
    rob = []
    rob_norm = []
    for name, pv in perturb:
        row = {"case": name}
        for lab, kind in [("PID", C_PID), ("SF", C_SF)]:
            tr_v = simulate(pv, make_ctrl(kind), 5.0, ref_speed_step)
            tr_h = simulate(pv, make_ctrl(kind), 5.0, ref_psi_step(90))
            mv = step_metrics(tr_v.t, tr_v.v, tr_v.v_ref, 0.5)
            mh = step_metrics(tr_h.t, deg(tr_h.psi), deg(tr_h.psi_ref), 0.5)
            row[lab] = {
                "ts_v": float(mv["settling_time"]), "OS_v": float(mv["overshoot_pct"]),
                "ess_v": float(mv["error_rel_pct"]),
                "ts_psi": float(mh["settling_time"]), "OS_psi": float(mh["overshoot_pct"]),
                "ess_psi_deg": float(abs(deg(tr_h.psi[-1]) - 90.0)),
            }
        # gangguan torsi eksternal pada heading
        row_g = {"case": name + " + torsi 0,05 N m (1 s)"}
        for lab, kind in [("PID", C_PID), ("SF", C_SF)]:
            tr = simulate(pv, make_ctrl(kind), 5.0,
                          lambda t: {"psi": 0.0, "v": 0.0},
                          dist_fn=lambda t: {"tau_ext": 0.05 if 2.0 <= t < 3.0 else 0.0})
            row_g[lab] = {"psi_dev_deg": float(deg(np.max(np.abs(tr.psi)))),
                          "psi_end_deg": float(deg(tr.psi[-1]))}
        rob.append(row)
        rob.append(row_g)
        rob_norm.append(row)
    out["s6"]["C"] = rob

    # derau sensor (tabel tersendiri, Tabel 6.7)
    noise = {"v": 0.01, "psi": 1e-3, "w": 0.02}
    out["s6"]["C_noise"] = {}
    for lab, kind in [("PID", C_PID), ("SF", C_SF)]:
        tr = simulate(P, make_ctrl(kind), 5.0, ref_speed_step, noise=noise)
        mv = step_metrics(tr.t, tr.v, tr.v_ref, 0.5)
        out["s6"]["C_noise"][lab] = {
            "ts_v": float(mv["settling_time"]),
            "OS_v": float(mv["overshoot_pct"]),
            "ess_v": float(mv["error_rel_pct"]),
            "rms_V": float(np.sqrt(np.mean(np.vstack([tr.VL, tr.VR]) ** 2))),
        }

    # grafik ringkas robustness: settling time kecepatan
    labels = [r["case"] for r in rob_norm]
    ts_pid = [r["PID"]["ts_v"] for r in rob_norm]
    ts_sf = [r["SF"]["ts_v"] for r in rob_norm]
    x = np.arange(len(labels))
    fig, ax = plt.subplots(figsize=(7.0, 3.4))
    ax.bar(x - 0.18, ts_pid, 0.36, label="PID")
    ax.bar(x + 0.18, ts_sf, 0.36, label="SF + integral")
    ax.set_xticks(x)
    ax.set_xticklabels(labels, rotation=12, ha="right", fontsize=7.5)
    ax.set_ylabel(r"$t_s$ kecepatan (2%) [s]")
    ax.set_title("Ketahanan waktu penetapan terhadap perubahan parameter")
    ax.legend()
    fig.tight_layout()
    fig.savefig(os.path.join(FIG, "6_C_robust.png"))
    plt.close(fig)

    # ======================================================================
    # 6D  Variasi motor
    # ======================================================================

if "6D" in PILIH:
    tahap("6D variasi motor")
    trB = simulate(PLANT_N20, make_ctrl(C_PID), 6.0, ref_speed_step)
    trB_sf = simulate(PLANT_N20, make_ctrl(C_SF), 6.0, ref_speed_step)
    out["s6"]["D"] = {
        "motor_B": {k: float(v) for k, v in PLANT_N20.summary().items()},
        "PID": {k: (None if v is None else float(v))
                for k, v in step_metrics(trB.t, trB.v, trB.v_ref, 0.5).items()
                if k in ("rise_time", "settling_time", "overshoot_pct", "error_final", "iae")},
        "SF": {k: (None if v is None else float(v))
               for k, v in step_metrics(trB_sf.t, trB_sf.v, trB_sf.v_ref, 0.5).items()
               if k in ("rise_time", "settling_time", "overshoot_pct", "error_final", "iae")},
        "TT": {k: (None if v is None else float(v))
               for k, v in step_metrics(tr_ff.t, tr_ff.v, tr_ff.v_ref, 0.5).items()
               if k in ("rise_time", "settling_time", "overshoot_pct", "error_final", "iae")},
    }
    # kecepatan maksimum yang bisa dicapai motor B (tanpa kontrol, V = 6 V)
    fig, ax = plt.subplots(2, 1, figsize=(7.0, 4.6), sharex=True)
    ax[0].plot(tr_ff.t, tr_ff.v_ref, "k--", lw=1.0, label="referensi")
    ax[0].plot(tr_ff.t, tr_ff.v, label="TT 48:1 (motor terpilih)")
    ax[0].plot(trB.t, trB.v, label="alternatif 30:1, Ra 6 Ω")
    ax[0].set_ylabel(r"$v$ [m/s]")
    ax[0].set_title("Pengaruh karakteristik motor terhadap respons kecepatan")
    ax[0].legend(loc="lower right")
    ax[1].plot(tr_ff.t, tr_ff.Vc, label="TT")
    ax[1].plot(trB.t, trB.Vc, label="alternatif")
    ax[1].set_ylabel(r"$V_c$ [V]")
    ax[1].set_xlabel("waktu [s]")
    ax[1].legend(loc="lower right")
    fig.tight_layout()
    fig.savefig(os.path.join(FIG, "6_D_motor.png"))
    plt.close(fig)

    with open(os.path.join(RES, _NAMA_JSON), "w", encoding="utf-8") as f:
        json.dump(out, f, indent=1, ensure_ascii=False)


ok()

ns = 0
if os.path.isdir(FIG):
    ns = sum(1 for b in os.listdir(FIG)
             if b.endswith('.png')
             and os.path.getmtime(os.path.join(FIG, b)) >= _T0)
print()
print("=" * 78)
print(f"SELESAI dalam {_time.time() - _T0:.1f} s | bagian: "
      + ", ".join(k for k, _ in BAGIAN if k in PILIH))
print(f"  grafik PNG     : {ns} berkas baru di {os.path.abspath(FIG)}")
print(f"  angka simulasi : {os.path.abspath(os.path.join(RES, _NAMA_JSON))}")
print()
print("Angka kunci (sama dengan tabel laporan):")
try:
    if "6A" not in PILIH or "4.7" not in PILIH:
        raise KeyError("bagian 6A/4.7 tidak dijalankan")
    a = out["s6"]["A"]
    b = out["s6"]["B"]
    print(f"  kecepatan tangga  PID t_s {a['step']['PID']['settling_time']:.3f} s"
          f" | SF t_s {a['step']['SF']['settling_time']:.3f} s"
          f" | 500 rpm PID t_s {a['rpm500']['PID']['settling_time']:.3f} s")
    print(f"  heading 90        PID t_s {out['s47']['cascade_90']['settling_time']:.3f} s"
          f" | SF t_s {out['s57']['heading90']['settling_time']:.3f} s"
          f" | PID 1 loop {out['s47']['pid1_90']['settling_time']:.3f} s"
          "  (rotasi dari diam, seperti tabel 4.7.1)")
    print(f"  ramp RMSE {out['s46']['ramp']['rmse'] * 1000:.1f} mm/s"
          f" | sinus RMSE {b['sin']['PID']['rmse']:.2f} derajat")
except Exception:  # noqa: BLE001
    print("  (butuh bagian 6A dan 4.7; jalankan tanpa --bagian untuk melihat semua angka)")
print()
print(f"Buka folder figs/ untuk grafiknya, atau results/{_NAMA_JSON} untuk angkanya.")
print("=" * 78)
