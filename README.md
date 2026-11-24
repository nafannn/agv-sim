# AGV Line-Follower Simulation

Interactive simulation of a four-wheel differential-drive AGV: nonlinear plant,
discrete-time control, and a web front-end. Built for **TF4107 Mechatronics
Project 2** and used to produce the simulation chapters of the report.

The plant is derived from the CAD design (mass, inertia, DC-gearmotor, friction)
and integrated with a hand-written Runge-Kutta 4 solver. No control-toolbox
functions (`step`, `lsim`, `tf`, `ss`, `feedback`) are used anywhere.

## Features

- **Seven scenarios** — speed step, multi-step reference, ramp, load
  disturbance, heading step 90°/180°, and sinusoidal heading tracking.
- **Three controllers** — cascaded PI with feedforward (design baseline),
  single-loop PID, and state feedback with integral action.
- **Robustness controls** — scale mass/inertia and friction from the UI and
  re-run without touching code.
- **Per-run metrics** — rise time, 2 % settling time, overshoot, steady-state
  error, IAE/RMSE and peak voltage.

## Architecture

```
browser ──GET /api/simulate?skenario=step&kontroler=pid──► Vercel Python function
        ◄──────────── JSON: time series + metrics ────────  api/simulate.py
canvas renders the top view, v(t), ψ(t) and the metric cards
```

The server-side engine is stdlib-only Python: it holds the plant coefficients,
integrates with RK4 (`h = 0.1 ms`), runs the controller at `Ts = 10 ms` with a
`1.5·Ts` transport delay, and returns ~300–400 downsampled points (~20 KB).

## Model

| Quantity | Value | Note |
|---|---|---|
| equivalent mass `m_eq` | 11.67 kg | body + 4 wheel/rotor inertias |
| yaw inertia `J_ψ` | 0.0603 kg·m² | body + reflected wheels |
| traction per amp `c_F` | 10.94 N/A | per side, 2 motors |
| viscous damping `c_v` | 0.5035 N·s/m | transliteration of motor `B_m` |
| Coulomb friction `F_C` | 3.08 N | reflected to the ground |
| rolling friction `F_roll` | 0.613 N | `μ_r·m·g` |
| back-EMF `k_v` | 6.44 V·s/m | `K_e·i_g/r` |
| speed channel | `a_v = 3.06 s⁻¹`, `b_v = 0.469` | linearised |
| yaw channel | `a_ω = 3.38 s⁻¹`, `b_ω = 6.81` | linearised |

Sampling `Ts = 10 ms` · delay `15 ms` · integrator `h = 0.1 ms` · drive limit
`|V| ≤ 6 V` · gearbox `48:1`, `η = 0.85`.

## Getting started

```bash
python dev_server.py          # http://127.0.0.1:8099
```

Only Python 3.9+ is required — no packages for the web app.

Command-line simulation (stdlib only):

```bash
python simulasi/simulasi_agv.py                     # all scenarios, metric tables
python simulasi/simulasi_agv.py --bandingkan        # PID cascade vs PID-1-loop vs SF
python simulasi/simulasi_agv.py --skenario psi90 --csv out
python simulasi/simulasi_agv.py --plot              # needs matplotlib
```

## API

`GET /api/simulate`

| Parameter | Values | Default |
|---|---|---|
| `skenario` | `step`, `multi`, `ramp`, `gangguan`, `psi90`, `psi180`, `sinus` | `step` |
| `kontroler` | `pid` (cascade + feedforward), `pid1` (single loop), `sf` (state feedback) | `pid` |
| `m_scale` | mass/inertia multiplier, 0.7–1.6 | `1` |
| `mu_scale` | friction multiplier, 0.5–2.5 | `1` |

Response:

```json
{
  "skenario": "step", "nama": "...", "kontroler": "pid", "satuan": "m/s",
  "metrik": {"tr": 0.27, "ts": 0.65, "MP": 0.0, "ess": 8.5e-7, "IAE": 0.052},
  "t": [...], "v": [...], "v_ref": [...], "psi_deg": [...], "psi_ref_deg": [...],
  "VL": [...], "VR": [...], "x": [...], "y": [...],
  "params": {"m_eq": 11.669, "J_psi": 0.0603, "c_F": 10.944, "a_v": 3.062, "b_v": 0.4689}
}
```

Invalid parameters return HTTP 400 with `{"error": "..."}`.

## Results (nominal tuning)

| Scenario | Controller | `t_s` (2 %) | Overshoot | Steady-state error |
|---|---|---|---|---|
| speed step 0 → 0.35 m/s | PID cascade | 0.650 s | 0 % | 8·10⁻⁷ m/s |
| speed step 0 → 0.35 m/s | state feedback | 0.650 s | 0 % | < 10⁻⁶ m/s |
| heading 0 → 90° | PID cascade | 1.030 s | 1.02 % | 0.096° |
| heading 0 → 90° | state feedback | 1.240 s | 1.18 % | 0.175° |
| sinusoidal heading, 0.2 Hz | PID cascade | — | — | RMSE 8.7° |

## Repository layout

```
index.html            web UI (canvas, no external libraries)
api/simulate.py       Python serverless engine: plant, controllers, metrics
dev_server.py         local server (static files + /api/simulate)
simulasi/             desktop/CLI sources and report generators
  agv/                params, plant (RK4), control, sim, metrics
  simulasi_agv.py     single-file, dependency-free CLI simulation
  run_sims.py         full scenario matrix → figures + results.json
  make_report.py      builds the Word/PDF report from results.json
  make_flow.py        control-algorithm flowcharts
```

## Deployment

1. Import the repository at [vercel.com/new](https://vercel.com/new).
2. Framework preset **Other**; leave Build Command and Output Directory empty.
3. Deploy. `api/simulate.py` is detected as a Python function automatically;
   `simulasi/` is excluded from the deployment via `.vercelignore`.

## Notes and limitations

- One request runs one full simulation; there is no streaming or websocket.
- Execution takes ~0.5 s (cold start ~1 s), well inside the default function
  timeout.
- Physical parameters are fixed at the values extracted from the CAD model and
  the motor datasheet; only mass and friction are exposed in the UI.
- The heading channel shows a Coulomb deadband of ≈0.56 V, which explains the
  phase lag visible in sinusoidal tracking.

## License

Coursework, TF4107 Mechatronics Project 2. Code provided for academic review.
