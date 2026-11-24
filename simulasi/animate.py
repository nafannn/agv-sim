"""Animasi simulasi AGV: lintasan S-curve, kecepatan, dan heading.

Menghasilkan MP4 (H.264, butuh ffmpeg) dan opsional GIF. Data diambil dari
simulasi yang sama dengan laporan (RK4 manual, kontroler diskret 10 ms).

Contoh:
    py -3.12 animate.py
    py -3.12 animate.py --gif
    py -3.12 animate.py --kontroler sf --simpan figs/anim_sf.mp4
"""

from __future__ import annotations

import argparse
import os
import shutil
import sys

import numpy as np

ROOT = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, ROOT)

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation, FFMpegWriter, PillowWriter
from matplotlib.patches import Rectangle, FancyArrow

from agv import (Plant, ControllerPID, ControllerSF, SpeedPI, SpeedSF,
                 HeadingCascade, HeadingSF, simulate)

# --- penguatan final (sama dengan run_sims.py) -------------------------
Kp_v, Ki_v = 17.78, 54.44
Kp_pos, Kp_w, Ki_w = 4.0, 2.448, 8.162
KV_SF, KXI_SF, N_SF, VC0_SF = 27.59, 136.5, 6.530, 0.675
KPSI_SF, KW_SF, KXIPSI_SF = 15.86, 2.154, 31.72

# dimensi ilustrasi body untuk gambar (track width 0,15 m dari Bab 3)
BODY_L = 0.22     # panjang body [m]
BODY_W = 0.17     # lebar body [m]
HALF_TRACK = 0.075

PLANT = Plant()


def buat_kontroler(nama):
    if nama == "sf":
        return ControllerSF(SpeedSF(KV_SF, KXI_SF, N_SF, VC0_SF),
                            HeadingSF(KPSI_SF, KW_SF, KXIPSI_SF))
    return ControllerPID(speed=SpeedPI(Kp_v, Ki_v),
                         heading=HeadingCascade(Kp_pos, Kp_w, Ki_w))


def jalankan(kontroler="pid", t_end=10.0):
    """S-curve: jelajah 0,30 m/s sambil heading mengikuti sinusoid 0,2 Hz."""
    def ref(t):
        psi = 0.0 if t < 0.5 else 0.75 * np.sin(2 * np.pi * 0.2 * (t - 0.5))
        return {"v": 0.30, "psi": psi}

    return simulate(PLANT, buat_kontroler(kontroler), t_end, ref)


def gambar_agv(ax, x, y, psi):
    """Gambar body + 4 roda + panah arah pada posisi (x, y, psi)."""
    from matplotlib.transforms import Affine2D

    tr = Affine2D().rotate(psi).translate(x, y) + ax.transData
    body = Rectangle((-BODY_L / 2, -BODY_W / 2), BODY_L, BODY_W,
                     facecolor="#d9d9d9", edgecolor="#333333", lw=1.0,
                     alpha=0.95, transform=tr, zorder=5)
    ax.add_patch(body)
    # roda: 4 titik di sisi kiri/kanan, jarak antar sisi 0,15 m
    wheels = []
    for sx in (-1, 1):
        for sy in (-1, 1):
            w = Rectangle((sx * (BODY_L / 2 - 0.045) - 0.025, sy * HALF_TRACK - 0.015),
                          0.050, 0.030, facecolor="#444444", edgecolor="none",
                          transform=tr, zorder=6)
            ax.add_patch(w)
            wheels.append(w)
    arrow = FancyArrow(0, 0, BODY_L * 0.60, 0, width=0.010,
                       head_width=0.030, head_length=0.026,
                       color="#1f77b4", transform=tr, zorder=7,
                       length_includes_head=True)
    ax.add_patch(arrow)
    return [body, arrow] + wheels


def main():
    ap = argparse.ArgumentParser(description="Animasi simulasi AGV.")
    ap.add_argument("--kontroler", default="pid", choices=["pid", "sf"])
    ap.add_argument("--durasi", type=float, default=10.0)
    ap.add_argument("--fps", type=int, default=30)
    ap.add_argument("--stride", type=int, default=3,
                    help="ambil 1 frame tiap N sampel kontrol (Ts=10 ms)")
    ap.add_argument("--gif", action="store_true", help="juga buat GIF (15 fps)")
    ap.add_argument("--simpan", default=None)
    args = ap.parse_args()

    tr = jalankan(args.kontroler, args.durasi)
    idx = np.arange(0, len(tr.t), args.stride)
    t = tr.t[idx]
    x, y, psi = tr.X[idx], tr.Y[idx], tr.psi[idx]
    v, v_ref = tr.v[idx], tr.v_ref[idx]
    psid, psi_ref = np.rad2deg(tr.psi[idx]), np.rad2deg(tr.psi_ref[idx])
    VL, VR = tr.VL[idx], tr.VR[idx]

    fig = plt.figure(figsize=(9.2, 4.9))
    gs = fig.add_gridspec(2, 2, width_ratios=[1.35, 1.0], wspace=0.24,
                          hspace=0.45, left=0.07, right=0.985,
                          top=0.85, bottom=0.11)
    ax_p = fig.add_subplot(gs[:, 0])
    ax_v = fig.add_subplot(gs[0, 1])
    ax_h = fig.add_subplot(gs[1, 1], sharex=ax_v)

    pad = 0.12
    ax_p.set_xlim(x.min() - pad, x.max() + pad)
    ax_p.set_ylim(y.min() - pad, y.max() + pad)
    ax_p.set_aspect("equal", adjustable="datalim")
    ax_p.grid(alpha=0.3)
    ax_p.set_xlabel("X [m]", fontsize=9)
    ax_p.set_ylabel("Y [m]", fontsize=9)
    ax_p.set_title("Lintasan (tampak atas)", fontsize=10)
    ax_p.tick_params(labelsize=8)

    ax_p.plot(x, y, color="0.72", lw=1.0, ls=":", zorder=1)
    (path_line,) = ax_p.plot([], [], color="#1f77b4", lw=1.8, zorder=3)

    ax_v.plot(t, v_ref, "k--", lw=1.0, label="referensi")
    (lv,) = ax_v.plot([], [], color="#1f77b4", lw=1.6)
    ax_v.set_ylabel(r"$v$ [m/s]", fontsize=9)
    ax_v.grid(alpha=0.3)
    ax_v.set_title("Kecepatan linear", fontsize=10)
    ax_v.tick_params(labelsize=8)

    ax_h.plot(t, psi_ref, "k--", lw=1.0, label="referensi")
    (lh,) = ax_h.plot([], [], color="#1f77b4", lw=1.6)
    ax_h.set_ylabel(r"$\psi$ [deg]", fontsize=9)
    ax_h.set_xlabel("waktu [s]", fontsize=9)
    ax_h.grid(alpha=0.3)
    ax_h.set_title("Heading", fontsize=10)
    ax_h.tick_params(labelsize=8)

    judul = fig.suptitle("", fontsize=9.5, y=0.972)
    agv = []

    def update(k):
        nonlocal agv
        path_line.set_data(x[:k + 1], y[:k + 1])
        lv.set_data(t[:k + 1], v[:k + 1])
        lh.set_data(t[:k + 1], psid[:k + 1])
        for a in agv:
            a.remove()
        agv = gambar_agv(ax_p, x[k], y[k], psi[k])
        judul.set_text(
            f"t = {t[k]:4.2f} s   |   v = {v[k]:4.2f} m/s   |   "
            f"ψ = {psid[k]:+5.1f}°   |   "
            f"V_L = {VL[k]:+4.2f} V, V_R = {VR[k]:+4.2f} V")
        return path_line, lv, lh, judul

    anim = FuncAnimation(fig, update, frames=len(t),
                         interval=1000 / args.fps, blit=False)

    out = args.simpan or os.path.join(ROOT, "figs",
                                      f"anim_agv_{args.kontroler}.mp4")
    os.makedirs(os.path.dirname(out), exist_ok=True)

    if shutil.which("ffmpeg"):
        anim.save(out, writer=FFMpegWriter(fps=args.fps, bitrate=2400,
                                           codec="libx264",
                                           extra_args=["-pix_fmt", "yuv420p"]),
                  dpi=120)
        print("animasi:", out, f"({os.path.getsize(out)/1e6:.1f} MB)")
    else:
        out = out.replace(".mp4", ".gif")
        anim.save(out, writer=PillowWriter(fps=args.fps), dpi=110)
        print("animasi (GIF, ffmpeg tidak ada):", out)

    if args.gif:
        gif = os.path.join(ROOT, "figs", f"anim_agv_{args.kontroler}.gif")
        anim.save(gif, writer=PillowWriter(fps=15), dpi=110)
        print("gif:", gif, f"({os.path.getsize(gif)/1e6:.1f} MB)")

    # frame statis untuk pratinjau
    png = os.path.join(ROOT, "figs", f"anim_frame_{args.kontroler}.png")
    update(len(t) // 2)
    fig.savefig(png, dpi=130)
    plt.close(fig)
    print("frame contoh:", png)


if __name__ == "__main__":
    main()
