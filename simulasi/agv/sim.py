"""Runner simulasi closed-loop: ZOH 10 ms, tunda transportasi 1.5*Ts, RK4."""

from __future__ import annotations

import math
from collections import deque
from dataclasses import dataclass, field
from typing import Callable, Dict, List, Optional

import numpy as np

from .params import Plant
from .plant import rk4_step


@dataclass
class Trace:
    t: np.ndarray
    v_ref: np.ndarray
    a_ref: np.ndarray
    psi_ref: np.ndarray
    v: np.ndarray
    psi: np.ndarray
    w: np.ndarray
    iL: np.ndarray
    iR: np.ndarray
    VL: np.ndarray
    VR: np.ndarray
    Vc: np.ndarray
    Vd: np.ndarray
    X: np.ndarray
    Y: np.ndarray

    def __getitem__(self, k: str) -> np.ndarray:
        return getattr(self, k)


def simulate(
    p: Plant,
    ctrl,
    t_end: float,
    ref_fn: Callable[[float], Dict[str, float]],
    dist_fn: Optional[Callable[[float], Dict[str, float]]] = None,
    Ts: float = 0.01,
    Td: float = 0.015,
    h: float = 1e-4,
    Vmax: Optional[float] = None,
    x0=(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
    noise: Optional[Dict[str, float]] = None,
    seed: int = 1,
) -> Trace:
    """Simulasikan sistem tertutup.

    ref_fn(t)  -> dict(v, a, psi, dpsi)
    dist_fn(t) -> dict(F_ext, tau_ext)
    noise      -> dict(v=sd, psi=sd, w=sd) derau sensor (opsional)
    """
    Vmax = p.V_supply if Vmax is None else Vmax
    cf = p.coef()
    ctrl.reset()
    st = list(x0)
    rng = np.random.default_rng(seed)

    n = int(round(t_end / Ts))
    nsub = max(1, int(round(Ts / h)))
    hh = Ts / nsub

    keys = ["t", "v_ref", "a_ref", "psi_ref", "v", "psi", "w", "iL", "iR",
            "VL", "VR", "Vc", "Vd", "X", "Y"]
    rec: Dict[str, List[float]] = {k: [] for k in keys}

    queue = deque()          # (waktu terpasang, VL, VR) -> model tunda ZOH
    VL_a = VR_a = 0.0

    for k in range(n + 1):
        t = k * Ts
        ref = ref_fn(t)
        dist = (dist_fn(t) if dist_fn else None) or {}

        meas = {"v": st[2], "psi": st[4], "w": st[3]}
        if noise:
            meas["v"] += rng.normal(0.0, noise.get("v", 0.0))
            meas["psi"] += rng.normal(0.0, noise.get("psi", 0.0))
            meas["w"] += rng.normal(0.0, noise.get("w", 0.0))

        VL, VR, diag = ctrl.step(ref, meas, Ts, Vmax)
        queue.append((t + Td, VL, VR))
        while queue and queue[0][0] <= t + 1e-12:
            _, VL_a, VR_a = queue.popleft()

        vals = [t, ref.get("v", 0.0), ref.get("a", 0.0), ref.get("psi", 0.0),
                st[2], st[4], st[3], st[0], st[1], VL_a, VR_a,
                diag["Vc"], diag["Vd"], st[5], st[6]]
        for kk, vv in zip(keys, vals):
            rec[kk].append(vv)

        if k == n:
            break
        F_ext = dist.get("F_ext", 0.0)
        tau_ext = dist.get("tau_ext", 0.0)
        for _ in range(nsub):
            st = rk4_step(st, VL_a, VR_a, cf, hh, F_ext, tau_ext)

    return Trace(**{k: np.asarray(v) for k, v in rec.items()})
