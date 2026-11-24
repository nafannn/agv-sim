"""Jalankan satu skenario simulasi saja (cepat, untuk coba-coba).

Contoh:
    python demo.py --kasus step
    python demo.py --kasus psi90 --kontroler sf --tampilkan
    python demo.py --kasus sinus --tampilkan --simpan figs/coba_sinus.png
"""

from __future__ import annotations

import argparse
import os
import sys

import numpy as np

ROOT = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, ROOT)

import matplotlib
from agv import (Plant, ControllerPID, ControllerSF, SpeedPI, SpeedSF,
                 HeadingCascade, HeadingPID, HeadingSF, simulate)
from agv.metrics import step_metrics, tracking_metrics, effort_metrics


# --- penguatan final (sama dengan run_sims.py) -------------------------
Kp_v, Ki_v = 17.78, 54.44
Kp_pos, Kp_w, Ki_w = 4.0, 2.448, 8.162
Kp1, Ki1, Kd1 = 7.050, 9.400, 1.273
KV_SF, KXI_SF, N_SF, VC0_SF = 27.59, 136.5, 6.530, 0.675
KPSI_SF, KW_SF, KXIPSI_SF = 15.86, 2.154, 31.72

P = Plant()


def buat_kontroler(nama: str, ff: bool = True):
    if nama == "sf":
        return ControllerSF(SpeedSF(KV_SF, KXI_SF, N_SF, VC0_SF),
                            HeadingSF(KPSI_SF, KW_SF, KXIPSI_SF))
    if nama == "pid1":
        return ControllerPID(speed=SpeedPI(Kp_v, Ki_v, use_ff=ff),
                             heading_pid1=HeadingPID(Kp1, Ki1, Kd1))
    return ControllerPID(speed=SpeedPI(Kp_v, Ki_v, use_ff=ff),
                         heading=HeadingCascade(Kp_pos, Kp_w, Ki_w))


KASUS = {
    "step": ("v", lambda t: {"v": 0.35 if t >= 0.5 else 0.0}, 4.0, None, 0.5),
    "multi": ("v", lambda t: {"v": 0.15 if 0.5 <= t < 2.0 else
                              (0.25 if 2.0 <= t < 3.5 else
                               (0.35 if 3.5 <= t < 5.0 else
                                (0.20 if t >= 5.0 else 0.0)))}, 7.0, None, 0.5),
    "ramp": ("v", lambda t: {"v": (0.4375 * (t - 0.5) if 0.5 <= t < 1.3 else
                                  (0.35 if 1.3 <= t < 4.0 else
                                   (max(0.0, 0.35 - 0.4375 * (t - 4.0))
                                    if 4.0 <= t < 4.8 else 0.0))),
                            "a": (0.4375 if 0.5 <= t < 1.3 else
                                  (-0.4375 if 4.0 <= t < 4.8 else 0.0))}, 6.0, None, 0.5),
    "gangguan": ("v", lambda t: {"v": 0.35 if t >= 0.5 else 0.0}, 6.0,
                 lambda t: {"F_ext": 1.5 if 3.0 <= t < 4.5 else 0.0}, 0.5),
    "psi90": ("psi", lambda t: {"psi": np.deg2rad(90) if t >= 0.5 else 0.0}, 5.0, None, 0.5),
    "psi180": ("psi", lambda t: {"psi": np.deg2rad(180) if t >= 0.5 else 0.0}, 6.0, None, 0.5),
    "sinus": ("psi", lambda t: {"psi": (0.0 if t < 0.5 else
                                        0.7 * np.sin(2 * np.pi * 0.2 * (t - 0.5))),
                                "v": 0.20}, 6.0, None, 1.0),
}


def main():
    ap = argparse.ArgumentParser(description="Demo satu skenario simulasi AGV.")
    ap.add_argument("--kasus", default="step", choices=sorted(KASUS),
                    help="skenario yang dijalankan (default: step)")
    ap.add_argument("--kontroler", default="pid", choices=["pid", "pid1", "sf"],
                    help="pid = cascade (default), pid1 = PID satu loop, sf = state feedback")
    ap.add_argument("--tanpa-ff", action="store_true", help="matikan feedforward (PID saja)")
    ap.add_argument("--simpan", default=None, help="path PNG keluaran")
    ap.add_argument("--tampilkan", action="store_true", help="buka jendela grafik")
    args = ap.parse_args()

    if not args.tampilkan:
        matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    kunci, ref, t_end, dist, t0 = KASUS[args.kasus]
    ctrl = buat_kontroler(args.kontroler, ff=not args.tanpa_ff)
    tr = simulate(P, ctrl, t_end, ref, dist_fn=dist)

    y = tr.psi if kunci == "psi" else tr.v
    ref_y = tr.psi_ref if kunci == "psi" else tr.v_ref
    if kunci == "psi":
        y, ref_y = np.rad2deg(y), np.rad2deg(ref_y)

    print(f"kasus={args.kasus} kontroler={args.kontroler} durasi={t_end} s")
    if args.kasus in ("sinus", "ramp"):
        m = tracking_metrics(tr.t, y, ref_y, t0)
    else:
        m = step_metrics(tr.t, y, ref_y, t0)
    m.update(effort_metrics(tr.t, tr.VL, tr.VR))
    satuan = "deg" if kunci == "psi" else "m/s"
    tampil = [("rise_time", "s"), ("settling_time", "s (2%)"),
              ("overshoot_pct", "%"), ("error_final", satuan),
              ("iae", "-"), ("rmse", "-"), ("max_abs_V", "V")]
    for k, u in tampil:
        if k in m and m[k] is not None:
            print(f"  {k:14s} = {m[k]:8.4g}  {u}")

    fig, ax = plt.subplots(2, 1, figsize=(7.0, 4.4), sharex=True,
                           gridspec_kw={"height_ratios": [2, 1]})
    ax[0].plot(tr.t, ref_y, "k--", lw=1.0, label="referensi")
    ax[0].plot(tr.t, y, label=f"{args.kontroler}")
    ax[0].set_ylabel(r"$\psi$ [deg]" if kunci == "psi" else r"$v$ [m/s]")
    ax[0].set_title(f"Demo: {args.kasus} ({args.kontroler})")
    ax[0].legend(loc="lower right")
    ax[1].plot(tr.t, tr.VL, label=r"$V_L$")
    ax[1].plot(tr.t, tr.VR, "--", label=r"$V_R$")
    ax[1].set_ylabel("tegangan [V]")
    ax[1].set_xlabel("waktu [s]")
    ax[1].legend(loc="lower right")
    ax[1].grid(alpha=0.3)
    ax[0].grid(alpha=0.3)
    fig.tight_layout()

    out = args.simpan or os.path.join(ROOT, "figs", f"demo_{args.kasus}_{args.kontroler}.png")
    os.makedirs(os.path.dirname(out), exist_ok=True)
    fig.savefig(out, dpi=150)
    print("grafik:", out)
    if args.tampilkan:
        plt.show()


if __name__ == "__main__":
    main()
