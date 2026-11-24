"""Menggambar flowchart algoritma kontrol (PID cascade dan state feedback).

Keluaran: figs/flow_pid.png dan figs/flow_sf.png untuk Lampiran B laporan.
Gaya: satu kolom vertikal, penghubung ortogonal, tanpa cabang menyilang.
"""

from __future__ import annotations

import os

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import FancyBboxPatch, Polygon

ROOT = os.path.dirname(os.path.abspath(__file__))
FIG = os.path.join(ROOT, "figs")
os.makedirs(FIG, exist_ok=True)

plt.rcParams.update({"font.family": "DejaVu Sans", "font.size": 8.4})
FS = 7.9

XK = 0.54      # sumbu kolom kotak
XL = 0.085     # jalur loop balik di kiri


def kotak(ax, y, w, h, teks, jenis="proses"):
    x = XK
    if jenis == "keputusan":
        ax.add_patch(Polygon([[x, y + h / 2], [x + w / 2, y], [x, y - h / 2],
                              [x - w / 2, y]], closed=True, facecolor="#fdf1e0",
                             edgecolor="#222", lw=1.0, zorder=2))
    else:
        r = 0.030 if jenis == "terminal" else 0.016
        ax.add_patch(FancyBboxPatch((x - w / 2, y - h / 2), w, h,
                                    boxstyle=f"round,pad=0.002,rounding_size={r}",
                                    facecolor="#e9eef4" if jenis == "proses" else "#e4e4e4",
                                    edgecolor="#222", lw=1.0, zorder=2))
    ax.text(x, y, teks, ha="center", va="center", fontsize=FS, zorder=3,
            linespacing=1.4)


def garis(ax, pts):
    """Garis ortogonal melalui beberapa titik; panah di ujung terakhir."""
    for i in range(len(pts) - 1):
        (x1, y1), (x2, y2) = pts[i], pts[i + 1]
        ax.plot([x1, x2], [y1, y2], color="#222", lw=1.0, zorder=1,
                solid_capstyle="round")
    (x1, y1), (x2, y2) = pts[-2], pts[-1]
    dx, dy = x2 - x1, y2 - y1
    n = max((dx ** 2 + dy ** 2) ** 0.5, 1e-9)
    ux, uy = dx / n, dy / n
    L, W = 0.020, 0.009
    ax.add_patch(Polygon([[x2, y2],
                          [x2 - L * ux - W * uy, y2 - L * uy + W * ux],
                          [x2 - L * ux + W * uy, y2 - L * uy - W * ux]],
                         closed=True, facecolor="#222", edgecolor="none", zorder=2))


def label(ax, x, y, teks, ha="center", rot=0):
    ax.text(x, y, teks, fontsize=FS - 0.3, ha=ha, va="center", rotation=rot,
            color="#222", zorder=4,
            bbox=dict(boxstyle="round,pad=0.16", fc="white", ec="none", alpha=0.92))


def bingkai(judul, tinggi, langkah):
    fig, ax = plt.subplots(figsize=(5.6, tinggi))
    ax.set_xlim(0, 1)
    ax.set_ylim(0, 1)
    ax.axis("off")

    n = len(langkah)
    y_top, y_bot = 0.935, 0.075
    ys = [y_top - i * (y_top - y_bot) / (n - 1) for i in range(n)]

    for i, (_, w, h, teks, jenis) in enumerate(langkah):
        kotak(ax, ys[i], w, h, teks, jenis)

    for i in range(n - 1):
        garis(ax, [(XK, ys[i] - langkah[i][2] / 2), (XK, ys[i + 1] + langkah[i + 1][2] / 2)])

    ax.set_title(judul, fontsize=9.6, pad=12)
    return fig, ax, ys


def gambar_pid(path):
    langkah = [
        (None, 0.50, 0.060, "Mulai\nk = 0, t = 0", "terminal"),
        (None, 0.68, 0.060, "Ukur state: v, ψ, ψ̇  (encoder)", "proses"),
        (None, 0.76, 0.090,
         "e$_v$ = v$_{ref}$ − v\n"
         "V$_{ff}$ = 6,53·v$_{ref}$ + 0,675·sgn(v$_{ref}$) + 2,13·a$_{ref}$", "proses"),
        (None, 0.76, 0.090,
         "V$_c$ = K$_p$(β·v$_{ref}$ − v) + I$_v$ + V$_{ff}$\n"
         "batasi ±6 V; bila saturasi: I$_v$ ← K$_{aw}$(V$_{c,sat}$ − V$_c$)$Δt$", "proses"),
        (None, 0.80, 0.130,
         "ω$_{ref}$ = clamp[K$_{p,pos}$(ψ$_{ref}$ − ψ), ±3 rad/s]\n"
         "V$_d$ = K$_{p,ω}$(ω$_{ref}$ − ψ̇) + I$_ω$\n"
         "batasi ±(6 − |V$_c$|); bila saturasi: I$_ω$ ← K$_{aw}$(V$_{d,sat}$ − V$_d$)$Δt$",
         "proses"),
        (None, 0.76, 0.075,
         "V$_L$ = V$_c$ − V$_d$,   V$_R$ = V$_c$ + V$_d$\n"
         "terapkan PWM ke driver + tunda 1,5·T$_s$ = 15 ms", "proses"),
        (None, 0.72, 0.070,
         "Integrasi plant: RK4, h = 0,1 ms\n(100 sub-langkah per T$_s$)", "proses"),
        (None, 0.34, 0.080, "t ≥ t$_{end}$ ?", "keputusan"),
        (None, 0.46, 0.055, "Selesai", "terminal"),
    ]
    fig, ax, ys = bingkai(
        "Algoritma kontrol PID cascade + feedforward\n(satu siklus kendali, T$_s$ = 10 ms)",
        7.4, langkah)

    y_dec = ys[7]
    label(ax, XK + 0.04, y_dec - 0.050, "ya", ha="left")
    garis(ax, [(XK, y_dec - 0.040), (XK, ys[8] + 0.028)])

    garis(ax, [(XK - 0.17, y_dec), (XL, y_dec), (XL, ys[1]), (XK - 0.34, ys[1])])
    label(ax, XL + 0.016, (ys[1] + y_dec) / 2, "tidak (k ← k+1, t ← kT$_s$)",
          ha="left", rot=90)

    fig.tight_layout()
    fig.savefig(path, dpi=220)
    plt.close(fig)


def gambar_sf(path):
    langkah = [
        (None, 0.54, 0.060, "Mulai\nk = 0, ξ$_v$ = 0, ξ$_ψ$ = 0", "terminal"),
        (None, 0.68, 0.060, "Ukur state: v, ψ, ψ̇  (encoder)", "proses"),
        (None, 0.80, 0.100,
         "V$_c$ = −k$_v$v + k$_{ξv}$ξ$_v$ + N·v$_{ref}$ + V$_{c0}$sgn(v$_{ref}$)\n"
         "batasi ±6 V; bila saturasi: ξ$_v$ ← (V$_{c,sat}$−V$_c$)/(T$_t$k$_{ξv}$)", "proses"),
        (None, 0.80, 0.100,
         "V$_d$ = −k$_ψ$ψ − k$_ω$ψ̇ + k$_{ξψ}$ξ$_ψ$\n"
         "batasi ±(6 − |V$_c$|); bila saturasi: ξ$_ψ$ ← (V$_{d,sat}$−V$_d$)/(T$_t$k$_{ξψ}$)",
         "proses"),
        (None, 0.80, 0.100,
         "ξ$_v$ ← ξ$_v$ + T$_s$(v$_{ref}$ − v),  ξ$_ψ$ ← ξ$_ψ$ + T$_s$(ψ$_{ref}$ − ψ)\n"
         "V$_L$ = V$_c$ − V$_d$, V$_R$ = V$_c$ + V$_d$ → PWM + tunda 15 ms", "proses"),
        (None, 0.72, 0.070,
         "Integrasi plant: RK4, h = 0,1 ms\n(100 sub-langkah per T$_s$)", "proses"),
        (None, 0.34, 0.080, "t ≥ t$_{end}$ ?", "keputusan"),
        (None, 0.46, 0.055, "Selesai", "terminal"),
    ]
    fig, ax, ys = bingkai(
        "Algoritma pengontrol alternatif: state feedback + integral\n"
        "(satu siklus kendali, T$_s$ = 10 ms)", 6.9, langkah)

    y_dec = ys[6]
    label(ax, XK + 0.04, y_dec - 0.050, "ya", ha="left")
    garis(ax, [(XK, y_dec - 0.040), (XK, ys[7] + 0.028)])

    garis(ax, [(XK - 0.17, y_dec), (XL, y_dec), (XL, ys[1]), (XK - 0.34, ys[1])])
    label(ax, XL + 0.016, (ys[1] + y_dec) / 2, "tidak (k ← k+1)", ha="left", rot=90)

    fig.tight_layout()
    fig.savefig(path, dpi=220)
    plt.close(fig)


if __name__ == "__main__":
    gambar_pid(os.path.join(FIG, "flow_pid.png"))
    gambar_sf(os.path.join(FIG, "flow_sf.png"))
    print("ok:", os.path.join(FIG, "flow_pid.png"), "|", os.path.join(FIG, "flow_sf.png"))
