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
- **Report figures on demand** — the eight document charts are re-rendered
  server-side with matplotlib from the current mass/friction settings, so the
  figure you download always belongs to the run you chose.
- **Run exports** — PNG (canvas composite), CSV time series and full JSON, named
  after the active scenario, controller and scaling.

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

The numeric endpoints (`/api/simulate` JSON) run on the standard library alone.
Rendering report figures (`?gambar=`) additionally needs numpy and matplotlib:

```bash
pip install -r requirements.txt
```

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
| `gambar` | `4_6_step`, `4_7_step90`, `4_7_step180`, `5_7_all`, `6_A_speed`, `6_B_pos`, `6_C_robust`, `6_D_motor` — returns a PNG instead of JSON | — |

Figure requests integrate at `h = 0.25 ms` (the report's convergence test shows
the same response to eight digits from 0.05 to 0.5 ms) to stay under the
function timeout; the JSON/CSV endpoints keep `h = 0.1 ms`.

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
requirements.txt      numpy + matplotlib, needed only by ?gambar= rendering
api/simulate.py       Python serverless engine: plant, controllers, metrics
dev_server.py         local server (static files + /api/simulate)
simulasi/             CLI simulation that produces every number in the report
  agv/                params, plant (RK4), control, sim, metrics
  simulasi_agv.py     single-file, dependency-free CLI simulation
  run_sims.py         full scenario matrix -> figures + results.json
```

## Running the simulation code only

The `simulasi/` folder is the code behind the report tables. It is excluded from
the deployment (`.vercelignore`) and needs no web stack:

```bash
cd simulasi
python simulasi_agv.py        # Python 3.8+, no packages, prints the metric tables
pip install -r requirements.txt
python run_sims.py            # ~60 s, writes figs/ and results/results.json
```

Downloading the repository as a ZIP and running `cd simulasi && python
simulasi_agv.py` reproduces the report numbers on any machine with Python.

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
