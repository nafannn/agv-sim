"""Parameter fisis AGV line-follower 4WD dan koefisien model.

Seluruh nilai dasar diambil dari Tabel 3.2 laporan (hasil CAD + datasheet
TT DC Gear Motor 48:1 + identifikasi). Koefisien turunan dihitung ulang di
sini dari besaran dasar supaya angka di laporan dan di kode selalu konsisten.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, replace


@dataclass(frozen=True)
class Plant:
    """Model fisik AGV: 4 roda, 2 motor per sisi (kiri/kanan paralel)."""

    # --- Parameter mekanik (Tabel 3.2)
    m: float = 2.5            # massa total AGV + payload [kg]
    r: float = 0.0318         # jari-jari roda penggerak [m]
    b: float = 0.15           # track width [m]
    Jz: float = 8.70e-3       # inersia yaw body terhadap CoG [kg m^2]
    Jw: float = 1.204e-5      # inersia satu roda [kg m^2]
    Jt: float = 2.0e-6        # inersia pulley/poros sisi roda [kg m^2]
    Jm: float = 1.0e-6        # inersia rotor motor [kg m^2]
    ig: float = 48.0          # rasio gearbox (omega_motor / omega_roda)
    eta: float = 0.85         # efisiensi transmisi
    mu_r: float = 0.025       # koefisien gesek gelinding
    g: float = 9.81           # [m/s^2]
    theta: float = 0.0        # kemiringan lintasan [rad]

    # --- Parameter motor DC (TT motor 48:1, hasil identifikasi)
    Ra: float = 4.0           # resistansi armatur [ohm]
    La: float = 2.0e-3        # induktansi armatur [H]
    Kt: float = 4.265e-3      # konstanta torsi [N m/A]
    Ke: float = 4.265e-3      # konstanta back-EMF [V s/rad]
    Bm: float = 6.5e-8        # gesek viskos motor+gearbox [N m s/rad]
    tau_c: float = 6.0e-4     # gesek Coulomb motor+gearbox [N m]
    V_supply: float = 6.0     # tegangan baterai [V]

    # ------------------------------------------------------------------
    # Koefisien turunan (Bab 3 laporan)
    # ------------------------------------------------------------------
    @property
    def J_eq(self) -> float:
        """Inersia ekuivalen satu sisi roda (dilihat di poros roda)."""
        return self.Jw + self.Jt + self.ig ** 2 * self.Jm

    @property
    def m_eq(self) -> float:
        """Massa ekuivalen translasi (massa body + inersia 4 roda)."""
        return self.m + 4.0 * self.J_eq / self.r ** 2

    @property
    def J_psi(self) -> float:
        """Inersia yaw ekuivalen terhadap pusat massa."""
        return self.Jz + self.J_eq * self.b ** 2 / self.r ** 2

    @property
    def c_F(self) -> float:
        """Gaya traksi per sisi per ampere (gabungan 2 motor). [N/A]"""
        return 2.0 * self.eta * self.ig * self.Kt / self.r

    @property
    def c_v(self) -> float:
        """Redaman viskos translasi total. [N s/m]"""
        return 4.0 * self.eta * self.ig ** 2 * self.Bm / self.r ** 2

    @property
    def F_C(self) -> float:
        """Gesek Coulomb total (4 motor direfleksikan ke roda). [N]"""
        return 4.0 * self.eta * self.ig * self.tau_c / self.r

    @property
    def F_roll(self) -> float:
        """Gesek gelinding total. [N]"""
        return self.mu_r * self.m * self.g * math.cos(self.theta)

    @property
    def k_v(self) -> float:
        """Back-EMF per m/s kecepatan linear. [V s/m]"""
        return self.Ke * self.ig / self.r

    @property
    def k_psi(self) -> float:
        """Back-EMF per rad/s laju yaw. [V s/rad]"""
        return self.Ke * self.ig * self.b / (2.0 * self.r)

    @property
    def c_T(self) -> float:
        """Torsi yaw per ampere diferensial. [N m/A]"""
        return self.Kt * self.eta * self.ig * self.b / self.r

    @property
    def c_psi(self) -> float:
        """Redaman viskos yaw. [N m s/rad]"""
        return 2.0 * self.eta * self.ig ** 2 * self.Bm * self.b ** 2 / self.r ** 2

    # ------------------------------------------------------------------
    # Model linear terefleksi (dipakai untuk tuning & verifikasi)
    # ------------------------------------------------------------------
    @property
    def b_v(self) -> float:
        """Gain kanal kecepatan dv/dt = -a_v v + b_v V_c. [1/s per V]"""
        return 2.0 * self.c_F / (self.m_eq * self.Ra)

    @property
    def a_v(self) -> float:
        """Konstanta waktu terbalik kanal kecepatan. [1/s]"""
        return (2.0 * self.c_F * self.k_v / self.Ra + self.c_v) / self.m_eq

    @property
    def b_w(self) -> float:
        """Gain kanal laju yaw dpsi_dot/dt = -a_w psi_dot + b_w V_d."""
        return 2.0 * self.c_T / (self.J_psi * self.Ra)

    @property
    def a_w(self) -> float:
        """Konstanta waktu terbalik kanal laju yaw. [1/s]"""
        return (2.0 * self.c_T * self.k_psi / self.Ra + self.c_psi) / self.J_psi

    # ------------------------------------------------------------------
    def coef(self):
        """Bekukan seluruh koefisien numerik menjadi objek Coef (cepat)."""
        from .plant import Coef

        return Coef(
            r=self.r, b=self.b, m=self.m, g=self.g, theta=self.theta,
            Ra=self.Ra, La=self.La, Ke=self.Ke, ig=self.ig,
            m_eq=self.m_eq, J_psi=self.J_psi, c_F=self.c_F, c_v=self.c_v,
            F_C=self.F_C, F_roll=self.F_roll, c_psi=self.c_psi,
        )

    def scaled(self, **kw) -> "Plant":
        """Salinan parameter dengan beberapa nilai diganti (uji robustness)."""
        return replace(self, **kw)

    def summary(self) -> dict:
        return {
            "m_eq": self.m_eq,
            "J_eq": self.J_eq,
            "J_psi": self.J_psi,
            "c_F": self.c_F,
            "c_v": self.c_v,
            "F_C": self.F_C,
            "F_roll": self.F_roll,
            "k_v": self.k_v,
            "k_psi": self.k_psi,
            "c_T": self.c_T,
            "c_psi": self.c_psi,
            "a_v": self.a_v,
            "b_v": self.b_v,
            "tau_v": 1.0 / self.a_v,
            "K_v": self.b_v / self.a_v,
            "a_w": self.a_w,
            "b_w": self.b_w,
            "K_w": self.b_w / self.a_w,
            "tau_w": 1.0 / self.a_w,
        }


# Motor pembanding pada Tahap 6-D: gearmotor DC 6 V 1:30 dengan rotor lebih
# berat (Kt lebih besar, Ra lebih tinggi) sebagai alternatif "torsi lebih besar,
# putaran lebih rendah" terhadap TT 48:1.
PLANT_N20 = Plant(
    ig=30.0,
    Kt=4.0e-3,
    Ke=4.0e-3,
    Ra=6.0,
    La=1.0e-3,
    Jm=1.2e-6,
    Bm=1.0e-7,
    tau_c=5.0e-4,
)
