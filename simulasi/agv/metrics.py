"""Metrik performa respons waktu (rise/settling time, overshoot, IAE, RMSE)."""

from __future__ import annotations

import numpy as np


def _first_cross(tt: np.ndarray, yy: np.ndarray, level: float) -> float | None:
    idx = np.where(yy >= level)[0]
    return float(tt[idx[0]]) if idx.size else None


def step_metrics(t, y, ref, t_start: float, tol: float = 0.02,
                 t_stop: float | None = None) -> dict:
    """Metrik respons tangga dihitung sejak t_start (waktu lompatan referensi).

    t_stop dipakai untuk membatasi jendela evaluasi pada referensi bertingkat.
    """
    t = np.asarray(t, dtype=float)
    y = np.asarray(y, dtype=float)
    ref = np.asarray(ref, dtype=float)
    m = t >= t_start - 1e-12
    if t_stop is not None:
        m &= t < t_stop - 1e-12
    tt = t[m] - t_start
    yy = y[m]
    rf = ref[m]

    y0 = float(yy[0])
    yf = float(rf[-1])
    d = yf - y0
    out = {"y0": y0, "yf": yf}
    if abs(d) < 1e-9:
        return out

    ys = (yy - y0) / d
    t10 = _first_cross(tt, ys, 0.10)
    t90 = _first_cross(tt, ys, 0.90)
    out["rise_time"] = (t90 - t10) if (t10 is not None and t90 is not None) else None

    outside = np.where(np.abs(ys - 1.0) > tol)[0]
    if outside.size:
        j = outside[-1] + 1
        out["settling_time"] = float(tt[j]) if j < tt.size else float(tt[-1])
    else:
        out["settling_time"] = 0.0

    out["overshoot_pct"] = max(0.0, (float(np.max(ys)) - 1.0)) * 100.0
    out["undershoot_pct"] = max(0.0, (1.0 - float(np.min(ys)))) * 100.0
    out["error_final"] = float(abs(yf - yy[-1]))
    out["error_rel_pct"] = float(abs(yf - yy[-1]) / abs(yf) * 100.0) if abs(yf) > 1e-9 else 0.0
    out["iae"] = float(np.trapezoid(np.abs(yy - rf), tt))
    out["rmse"] = float(np.sqrt(np.mean((yy - rf) ** 2)))
    out["peak"] = float(np.max(yy))
    return out


def tracking_metrics(t, y, ref, t_start: float) -> dict:
    t = np.asarray(t, dtype=float)
    y = np.asarray(y, dtype=float)
    ref = np.asarray(ref, dtype=float)
    m = t >= t_start - 1e-12
    tt = t[m] - t_start
    e = y[m] - ref[m]
    return {
        "rmse": float(np.sqrt(np.mean(e ** 2))),
        "iae": float(np.trapezoid(np.abs(e), tt)),
        "max_abs_err": float(np.max(np.abs(e))),
    }


def effort_metrics(t, VL, VR) -> dict:
    VL = np.asarray(VL, dtype=float)
    VR = np.asarray(VR, dtype=float)
    V = np.vstack([VL, VR])
    return {
        "max_abs_V": float(np.max(np.abs(V))),
        "rms_V": float(np.sqrt(np.mean(V ** 2))),
    }
