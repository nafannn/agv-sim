# AGV Line-Follower Simulation

Interactive simulation of a four-wheel differential-drive AGV: nonlinear plant,
discrete-time control, and a web front-end. Built for **TF4107 Mechatronics
Project 2** and used to produce the simulation chapters of the report.

The plant is derived from the CAD design (mass, inertia, DC-gearmotor, friction)
and integrated with a hand-written Runge-Kutta 4 solver. No control-toolbox
functions (`step`, `lsim`, `tf`, `ss`, `feedback`) are used anywhere.

## What you can do

- **Eight scenarios** — speed step, multi-step, ramp, load disturbance, 500 rpm
  (0.0347 m/s), heading step 90°/180°, and sinusoidal heading tracking.
- **Three controllers** — cascaded PI with feedforward (design baseline),
  single-loop PID, and state feedback with integral action.
- **Compare** — pick a second controller in *Bandingkan dengan*; the run is
  repeated and both curves are drawn in the same panels.
- **Robustness sliders** — scale mass/inertia (0.7–1.6×) and friction (0.5–2.5×)
  and re-run without touching code.
- **Exports** — `Simpan PNG` (report-style sheet: header with both metric sets,
  then the same two panels as the document), `CSV` (time series) and `JSON`
  (full response). Filenames carry scenario, controller and scaling.
- **Report figures on demand** — the eight document charts are re-rendered
  server-side with matplotlib from the current sliders; the figure is **shown in
  the page first**, and only then offered as a download.

## Architecture

```
browser ──GET /api/simulate?skenario=step&kontroler=pid──► Vercel Python function
        ◄──────────── JSON: time series + metrics ────────  api/simulate.py
        ◄──────────── PNG: report-style figure ───────────  (?gambar=name)
canvas draws the top view, v(t) or ψ(t), V_c or ψ̇(t) and the metric cards
```

The engine holds the plant coefficients, integrates with RK4 (`h = 0.1 ms`),
runs the controller at `Ts = 10 ms` with a `1.5·Ts` transport delay, and returns
~300–400 points (~20 KB). The JSON path uses the standard library only; the
figure path imports numpy and matplotlib on demand.

## Model

| Quantity | Value | Note |
|---|---|---|
| equivalent mass `m_eq` | 11.67 kg | body + 4 wheel/rotor inertias |
| yaw inertia `J_ψ` | 0.0603 kg·m² | body + reflected wheels |
| traction per amp `c_F` | 10.94 N/A | per side, 2 motors |
| viscous damping `c_v` | 0.5035 N·s/m | reflected motor `B_m` |
| Coulomb friction `F_C` | 3.08 N | reflected to the ground |
| rolling friction `F_roll` | 0.613 N | `μ_r·m·g` |
| back-EMF `k_v` | 6.44 V·s/m | `K_e·i_g/r` |
| speed channel | `a_v = 3.06 s⁻¹`, `b_v = 0.469` | linearised |
| yaw channel | `a_ω = 3.38 s⁻¹`, `b_ω = 6.81` | linearised |

Sampling `Ts = 10 ms` · delay `15 ms` · integrator `h = 0.1 ms` · drive limit
`|V| ≤ 6 V` · gearbox `48:1`, `η = 0.85`.

## Using the page

1. Pick **Skenario** and **Kontroler**; optionally set *Bandingkan dengan*.
2. **Jalankan simulasi** — charts, metric cards and readout update. Hovering a
   chart freezes the animation and locks both charts to the same instant.
3. **Simpan PNG / CSV / JSON** export the run that is on screen.
4. **Hasil simulasi pada laporan** renders one of the document figures with the
   current sliders, shows it inline, then offers *Simpan gambar*.

## API

`GET /api/simulate`

| Parameter | Values | Default |
|---|---|---|
| `skenario` | `step`, `multi`, `ramp`, `gangguan`, `rpm500`, `psi90`, `psi180`, `sinus` | `step` |
| `kontroler` | `pid` (cascade + feedforward), `pid1` (single loop), `sf` (state feedback) | `pid` |
| `m_scale` | mass/inertia multiplier, 0.7–1.6 | `1` |
| `mu_scale` | friction multiplier, 0.5–2.5 | `1` |
| `gambar` | `4_6_step`, `4_7_step90`, `4_7_step180`, `5_7_all`, `6_A_speed`, `6_B_pos`, `6_C_robust`, `6_D_motor` — returns a PNG instead of JSON | — |

Figure requests integrate at `h = 0.25 ms` (the convergence test in the report
shows the same response to eight digits from 0.05 to 0.5 ms) to stay inside the
function timeout; the JSON path keeps `h = 0.1 ms`.

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
| speed step to 500 rpm (0.0347 m/s) | PID cascade | 0.300 s | 1.15 % | < 10⁻⁶ m/s |
| heading 0 → 90° | PID cascade | 1.030 s | 1.02 % | 0.096° |
| heading 0 → 90° | state feedback | 1.240 s | 1.18 % | 0.175° |
| heading 0 → 90° | single-loop PID | 3.160 s | 3.60 % | 0.545° |
| sinusoidal heading, 0.2 Hz | PID cascade | — | — | RMSE 8.68° |

## Getting started

```bash
python dev_server.py          # http://127.0.0.1:8099
```

The JSON endpoints need nothing but Python. Figure rendering and the CLI matrix
need numpy and matplotlib:

```bash
pip install -r requirements.txt
```

## Simulation code only (no web stack)

`simulasi/` holds the code behind the report tables; it is excluded from the
deployment via `.vercelignore`.

```bash
cd simulasi
python simulasi_agv.py        # Python 3.8+, no packages, prints the metric tables
python run_sims.py            # ~60 s, writes figs/ and results/results.json
```

Downloading the repository as a ZIP and running `cd simulasi && python
simulasi_agv.py` reproduces the report numbers on any machine with Python.

## Repository layout

```
index.html            web UI (canvas, no external libraries)
api/simulate.py       Python function: plant, controllers, metrics, figure renderer
dev_server.py         local server (static files + /api/simulate), same router as Vercel
pyproject.toml        Vercel entrypoint + dependency declaration
requirements.txt      numpy, matplotlib
simulasi/             CLI simulation that produces every number in the report
  agv/                params, plant (RK4), control, sim, metrics
  simulasi_agv.py     single-file, dependency-free CLI simulation
  run_sims.py         full scenario matrix -> figures + results.json
```

## Deployment

1. Import the repository at [vercel.com/new](https://vercel.com/new).
2. Framework preset **Other**; leave Build Command and Output Directory empty.
3. Deploy. `api/simulate.py` is detected as a Python function; `simulasi/` is
   excluded from the deployment.

Dependencies must be declared under `[project].dependencies` in
`pyproject.toml`: this project is built from `pyproject.toml` (because of
`[tool.vercel] entrypoint`), so a bare `requirements.txt` is not installed and
the figure route fails with `No module named matplotlib`.

## Notes and limitations

- One request runs one full simulation; there is no streaming or websocket.
- JSON answers take ~0.5 s. Figure requests take 1–4 s, and a cold function needs
  a few seconds more while numpy and matplotlib are imported.
- Physical parameters are fixed at the values extracted from the CAD model and
  the motor datasheet; only mass and friction are exposed in the UI.
- The heading channel shows a Coulomb deadband of ≈0.56 V, which explains the
  phase lag visible in sinusoidal tracking.

## License

Coursework, TF4107 Mechatronics Project 2. Code provided for academic review.
