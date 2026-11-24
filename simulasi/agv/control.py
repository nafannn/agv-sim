"""Algoritma kontrol: PID (speed + heading) dan state feedback + integral.

Semua hukum kendali diimplementasikan eksplisit dalam bentuk diskret
(Ts = 10 ms, zero-order hold), lengkap dengan saturasi dan anti-windup.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field


def clamp(x: float, lo: float, hi: float) -> float:
    return lo if x < lo else (hi if x > hi else x)


def sgn0(x: float, eps: float = 1e-6) -> float:
    if x > eps:
        return 1.0
    if x < -eps:
        return -1.0
    return 0.0


# ----------------------------------------------------------------------
# Loop kecepatan
# ----------------------------------------------------------------------
@dataclass
class SpeedPI:
    """PI kecepatan + feedforward back-EMF / gesek / inersia (subbab 4.2.2).

    Memakai setpoint weighting 2-DOF: aksi P bekerja pada beta*v_ref - v,
    sedangkan integrator bekerja pada galat penuh (v_ref - v). Nilai beta < 1
    menekan lonjakan saat referensi berubah tanpa memperlambat pelacakan ramp.
    """

    Kp: float = 17.78
    Ki: float = 54.44
    beta: float = 0.70        # bobot setpoint untuk aksi proporsional
    ff_kv: float = 6.530      # [V/(m/s)]  = Ra*c_v/(2c_F) + k_v
    ff_fric: float = 0.675    # [V]        = Ra*(F_C + F_roll)/(2c_F)
    ff_acc: float = 2.133     # [V s^2/m]  = Ra*m_eq/(2c_F)
    use_ff: bool = True
    Kaw: float = 25.0         # penguatan back-calculation anti-windup [1/s]
    I: float = 0.0

    def reset(self) -> None:
        self.I = 0.0

    def step(self, v_ref: float, a_ref: float, v: float, dt: float, limit: float):
        vff = 0.0
        if self.use_ff:
            vff = self.ff_kv * v_ref + self.ff_fric * sgn0(v_ref) + self.ff_acc * a_ref
        u = self.Kp * (self.beta * v_ref - v) + self.I + vff
        us = clamp(u, -limit, limit)
        self.I += (self.Ki * (v_ref - v) + self.Kaw * (us - u)) * dt
        self.I = clamp(self.I, -1.5 * limit, 1.5 * limit)
        return us, {"e": v_ref - v, "vff": vff, "I": self.I}


@dataclass
class SpeedSF:
    """State feedback + integrator pada kanal kecepatan (subbab 5.2.1)."""

    kv_fb: float = 27.59      # umpan balik v
    kxi: float = 136.5        # umpan balik integrator
    N: float = 6.530          # feedforward referensi
    Vc0: float = 0.675        # kompensasi gesek Coulomb + gelinding
    Tt: float = 0.05          # konstanta waktu anti-windup
    xi: float = 0.0

    def reset(self) -> None:
        self.xi = 0.0

    def step(self, v_ref: float, a_ref: float, v: float, dt: float, limit: float):
        u = -self.kv_fb * v + self.kxi * self.xi + self.N * v_ref + self.Vc0 * sgn0(v_ref)
        us = clamp(u, -limit, limit)
        self.xi += (v_ref - v) * dt + (us - u) * dt / (self.Tt * max(self.kxi, 1e-3))
        self.xi = clamp(self.xi, -20.0, 20.0)
        return us, {"e": v_ref - v, "xi": self.xi}


# ----------------------------------------------------------------------
# Loop heading (posisi sudut)
# ----------------------------------------------------------------------
@dataclass
class HeadingCascade:
    """Cascade: P pada psi (luar) + PI pada psi_dot (dalam)."""

    Kp_pos: float = 4.0
    Kp_w: float = 2.448
    Ki_w: float = 8.162
    w_max: float = 3.0        # batas referensi laju yaw [rad/s]
    Kaw: float = 25.0
    I: float = 0.0

    def reset(self) -> None:
        self.I = 0.0

    def step(self, psi_ref: float, psi: float, w: float, dt: float, limit: float):
        w_ref = clamp(self.Kp_pos * (psi_ref - psi), -self.w_max, self.w_max)
        e = w_ref - w
        u = self.Kp_w * e + self.I
        us = clamp(u, -limit, limit)
        self.I += (self.Ki_w * e + self.Kaw * (us - u)) * dt
        self.I = clamp(self.I, -1.5 * limit, 1.5 * limit)
        return us, {"w_ref": w_ref, "e": e, "I": self.I}


@dataclass
class HeadingPID:
    """PID satu loop pada heading (pembanding arsitektur cascade)."""

    Kp: float = 7.050
    Ki: float = 9.400
    Kd: float = 1.273
    Tf: float = 0.020
    Kaw: float = 25.0
    I: float = 0.0
    df: float = 0.0

    def reset(self) -> None:
        self.I = 0.0
        self.df = 0.0

    def step(self, psi_ref: float, psi: float, w: float, dt: float, limit: float):
        e = psi_ref - psi
        # turunan atas pengukuran (bukan atas error) + filter orde satu
        self.df += dt / (self.Tf + dt) * (w - self.df)
        u = self.Kp * e + self.I - self.Kd * self.df
        us = clamp(u, -limit, limit)
        self.I += (self.Ki * e + self.Kaw * (us - u)) * dt
        self.I = clamp(self.I, -1.5 * limit, 1.5 * limit)
        return us, {"e": e, "df": self.df, "I": self.I}


@dataclass
class HeadingSF:
    """State feedback + integrator pada kanal heading (subbab 5.2.2)."""

    k_psi: float = 15.86
    k_w: float = 2.154
    k_xi: float = 31.72
    Tt: float = 0.05
    xi: float = 0.0

    def reset(self) -> None:
        self.xi = 0.0

    def step(self, psi_ref: float, psi: float, w: float, dt: float, limit: float):
        u = -self.k_psi * psi - self.k_w * w + self.k_xi * self.xi
        us = clamp(u, -limit, limit)
        self.xi += (psi_ref - psi) * dt + (us - u) * dt / (self.Tt * self.k_xi)
        self.xi = clamp(self.xi, -5.0, 5.0)
        return us, {"e": psi_ref - psi, "xi": self.xi}


# ----------------------------------------------------------------------
# Penggabung (mixer) aktuator
# ----------------------------------------------------------------------
def mixer(Vc: float, Vd: float, Vmax: float) -> tuple[float, float]:
    """Petakan tegangan common-mode + diferensial ke kedua sisi motor.

    Menggunakan driver dua arah (H-bridge), batas |V| <= Vmax.
    """
    Vc = clamp(Vc, -Vmax, Vmax)
    Vd = clamp(Vd, -(Vmax - abs(Vc)), Vmax - abs(Vc))
    return Vc - Vd, Vc + Vd


# ----------------------------------------------------------------------
# Kontroler gabungan
# ----------------------------------------------------------------------
class ControllerPID:
    """Kontroler PID: PI kecepatan + cascade heading."""

    name = "PID (cascade)"

    def __init__(self, speed: SpeedPI | None = None, heading: HeadingCascade | None = None,
                 heading_pid1: HeadingPID | None = None):
        self.speed = speed or SpeedPI()
        self.heading = heading or HeadingCascade()
        self.heading_pid1 = heading_pid1
        self.single_loop_heading = heading_pid1 is not None

    def reset(self) -> None:
        self.speed.reset()
        self.heading.reset()
        if self.heading_pid1 is not None:
            self.heading_pid1.reset()

    def step(self, ref, meas, dt, Vmax):
        v_ref = ref.get("v", 0.0)
        a_ref = ref.get("a", 0.0)
        psi_ref = ref.get("psi", 0.0)
        Vc, dc = self.speed.step(v_ref, a_ref, meas["v"], dt, Vmax)
        if self.heading_pid1 is not None:
            Vd, dh = self.heading_pid1.step(psi_ref, meas["psi"], meas["w"], dt,
                                            Vmax - abs(Vc))
        else:
            Vd, dh = self.heading.step(psi_ref, meas["psi"], meas["w"], dt,
                                       Vmax - abs(Vc))
        VL, VR = mixer(Vc, Vd, Vmax)
        return VL, VR, {"Vc": Vc, "Vd": Vd, **{f"c_{k}": val for k, val in dc.items()},
                        **{f"h_{k}": val for k, val in dh.items()}}


class ControllerSF:
    """Pengontrol alternatif: state feedback + integral (pole placement)."""

    name = "State Feedback + Integral"

    def __init__(self, speed: SpeedSF | None = None, heading: HeadingSF | None = None):
        self.speed = speed or SpeedSF()
        self.heading = heading or HeadingSF()

    def reset(self) -> None:
        self.speed.reset()
        self.heading.reset()

    def step(self, ref, meas, dt, Vmax):
        v_ref = ref.get("v", 0.0)
        a_ref = ref.get("a", 0.0)
        psi_ref = ref.get("psi", 0.0)
        Vc, dc = self.speed.step(v_ref, a_ref, meas["v"], dt, Vmax)
        Vd, dh = self.heading.step(psi_ref, meas["psi"], meas["w"], dt,
                                   Vmax - abs(Vc))
        VL, VR = mixer(Vc, Vd, Vmax)
        return VL, VR, {"Vc": Vc, "Vd": Vd, **{f"c_{k}": val for k, val in dc.items()},
                        **{f"h_{k}": val for k, val in dh.items()}}
