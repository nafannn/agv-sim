"""Model dinamik nonlinear AGV + motor DC, dan integrator RK4 manual.

State:  x = [i_L, i_R, v, psi_dot, psi, X, Y]
  i_L, i_R : arus tiap motor [A] (sisi kiri / kanan, 2 motor identik per sisi)
  v        : kecepatan linear body [m/s]
  psi_dot  : laju yaw [rad/s]
  psi      : heading [rad]
  X, Y     : posisi global [m]

Input:  u = [V_L, V_R] (tegangan motor kiri/kanan, di-clamp oleh driver).

Tidak ada fungsi kontrol bawaan yang dipakai: integrasi dilakukan sendiri
dengan Runge-Kutta orde 4, langkah tetap.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Sequence, Tuple

from .params import Plant  # noqa: F401  (tipe dokumentasi)

State = Tuple[float, float, float, float, float, float, float]

# Ambang penghalusan fungsi sign agar RK4 tidak "chattering" di kecepatan nol.
_EPS_W = 0.20   # [rad/s]
_EPS_V = 0.005  # [m/s]


@dataclass(frozen=True, slots=True)
class Coef:
    """Koefisien numerik beku (hasil Plant.coef()).

    Dibuat sekali per simulasi supaya loop integrasi hanya berisi aritmetika
    skalar (tidak menghitung ulang properti tiap evaluasi turunan).
    """

    r: float
    b: float
    m: float
    g: float
    theta: float
    Ra: float
    La: float
    Ke: float
    ig: float
    m_eq: float
    J_psi: float
    c_F: float
    c_v: float
    F_C: float
    F_roll: float
    c_psi: float


def sgn_s(x: float, eps: float) -> float:
    """Aproksimasi kontinu fungsi sign: linear di dalam pita +-eps."""
    if x > eps:
        return 1.0
    if x < -eps:
        return -1.0
    return x / eps


def rhs(
    st: Sequence[float],
    V_L: float,
    V_R: float,
    c: Coef,
    F_ext: float = 0.0,
    tau_ext: float = 0.0,
) -> State:
    """Turunan waktu seluruh state (persamaan E1, E2, M1, M2, K laporan)."""
    iL, iR, v, w, psi, _x, _y = st
    b = c.b
    r = c.r

    wL = (v - 0.5 * b * w) / r      # kecepatan sudut roda kiri
    wR = (v + 0.5 * b * w) / r      # kecepatan sudut roda kanan

    # Persamaan listrik tiap motor
    diL = (V_L - c.Ra * iL - c.Ke * c.ig * wL) / c.La
    diR = (V_R - c.Ra * iR - c.Ke * c.ig * wR) / c.La

    # Gaya traksi per sisi (2 roda digabung): viskos + Coulomb + gelinding
    half_cv = 0.5 * c.c_v
    half_cf = 0.5 * c.F_C
    half_roll = 0.5 * c.F_roll
    sgn_v = sgn_s(v, _EPS_V)
    F_L = (c.c_F * iL - half_cv * (v - 0.5 * b * w)
           - half_cf * sgn_s(wL, _EPS_W) - half_roll * sgn_v)
    F_R = (c.c_F * iR - half_cv * (v + 0.5 * b * w)
           - half_cf * sgn_s(wR, _EPS_W) - half_roll * sgn_v)

    F_slope = c.m * c.g * math.sin(c.theta)

    dv = (F_L + F_R - F_slope - F_ext) / c.m_eq
    dw = (0.5 * b * (F_R - F_L) - c.c_psi * w - tau_ext) / c.J_psi
    return (diL, diR, dv, dw, w, v * math.cos(psi), v * math.sin(psi))


def rk4_step(
    st: Sequence[float],
    V_L: float,
    V_R: float,
    c: Coef,
    h: float,
    F_ext: float = 0.0,
    tau_ext: float = 0.0,
) -> State:
    """Satu langkah integrasi RK4 (eksplisit, implementasi sendiri)."""
    iL, iR, v, w, psi, x, y = st
    h2 = 0.5 * h

    k1 = rhs((iL, iR, v, w, psi, x, y), V_L, V_R, c, F_ext, tau_ext)
    s2 = (iL + h2 * k1[0], iR + h2 * k1[1], v + h2 * k1[2], w + h2 * k1[3],
          psi + h2 * k1[4], x + h2 * k1[5], y + h2 * k1[6])
    k2 = rhs(s2, V_L, V_R, c, F_ext, tau_ext)
    s3 = (iL + h2 * k2[0], iR + h2 * k2[1], v + h2 * k2[2], w + h2 * k2[3],
          psi + h2 * k2[4], x + h2 * k2[5], y + h2 * k2[6])
    k3 = rhs(s3, V_L, V_R, c, F_ext, tau_ext)
    s4 = (iL + h * k3[0], iR + h * k3[1], v + h * k3[2], w + h * k3[3],
          psi + h * k3[4], x + h * k3[5], y + h * k3[6])
    k4 = rhs(s4, V_L, V_R, c, F_ext, tau_ext)

    h6 = h / 6.0
    return (iL + h6 * (k1[0] + 2.0 * k2[0] + 2.0 * k3[0] + k4[0]),
            iR + h6 * (k1[1] + 2.0 * k2[1] + 2.0 * k3[1] + k4[1]),
            v + h6 * (k1[2] + 2.0 * k2[2] + 2.0 * k3[2] + k4[2]),
            w + h6 * (k1[3] + 2.0 * k2[3] + 2.0 * k3[3] + k4[3]),
            psi + h6 * (k1[4] + 2.0 * k2[4] + 2.0 * k3[4] + k4[4]),
            x + h6 * (k1[5] + 2.0 * k2[5] + 2.0 * k3[5] + k4[5]),
            y + h6 * (k1[6] + 2.0 * k2[6] + 2.0 * k3[6] + k4[6]))
