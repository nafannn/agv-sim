# AGV Line-Follower Simulation

Simulation of a four-wheel differential-drive AGV: nonlinear plant, discrete-time
control, and a web page to try it in a browser. Built for **TF4107 Robotics and
Automation — Project 2**; the numbers it produces are the ones used in the report.

The plant comes from the CAD design (mass, inertia, DC gearmotor, friction) and is
integrated with a hand-written Runge-Kutta 4 solver. No control-toolbox functions
(`step`, `lsim`, `tf`, `ss`, `feedback`) are used anywhere.

## What you can try

- **Eight scenarios** — speed step, multi-step, ramp, load disturbance, 500 rpm
  (0.0347 m/s), heading step 90°/180°, and sinusoidal heading tracking.
- **Three controllers** — cascaded PI with feedforward, single-loop PID, and state
  feedback with integral action.
- **Comparison** — choose a second controller in *Bandingkan dengan* and both
  curves are drawn in the same panels.
- **Parameter sliders** — mass/inertia 0.7–1.6× and friction 0.5–2.5×, no code
  editing needed.
- **Exports** — the run on screen as PNG, CSV or JSON; filenames carry scenario,
  controller and scaling.
- **Report figures** — the eight charts of the document, re-drawn with the current
  sliders, shown on the page and downloadable from there.

## Using the page

1. Pick **Skenario** and **Kontroler**; optionally set *Bandingkan dengan*.
2. **Jalankan simulasi** — charts, metric cards and the readout update. Hovering a
   chart freezes the animation and locks both charts to the same instant.
3. **Simpan PNG / CSV / JSON** export the run on screen.
4. **Hasil simulasi pada laporan** draws one of the document figures, displays it
   inline, then offers *Simpan gambar*.

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

Sampling `Ts = 10 ms` · command delay `15 ms` · integrator `h = 0.1 ms` · drive
limit `|V| ≤ 6 V` · gearbox `48:1`, `η = 0.85`.

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

## API

`GET /api/simulate`

| Parameter | Values | Default |
|---|---|---|
| `skenario` | `step`, `multi`, `ramp`, `gangguan`, `rpm500`, `psi90`, `psi180`, `sinus` | `step` |
| `kontroler` | `pid`, `pid1`, `sf` | `pid` |
| `m_scale` | mass/inertia multiplier, 0.7–1.6 | `1` |
| `mu_scale` | friction multiplier, 0.5–2.5 | `1` |
| `gambar` | `4_6_step`, `4_7_step90`, `4_7_step180`, `5_7_all`, `6_A_speed`, `6_B_pos`, `6_C_robust`, `6_D_motor` — returns a PNG instead of JSON | — |

Response (`skenario=step&kontroler=pid`):

```json
{
  "skenario": "step", "nama": "...", "kontroler": "pid", "satuan": "m/s",
  "metrik": {"tr": 0.27, "ts": 0.65, "MP": 0.0, "ess": 8.5e-7, "IAE": 0.052},
  "t": [...], "v": [...], "v_ref": [...], "psi_deg": [...], "psi_ref_deg": [...],
  "VL": [...], "VR": [...], "x": [...], "y": [...],
  "params": {"m_eq": 11.669, "J_psi": 0.0603, "c_F": 10.944, "a_v": 3.062, "b_v": 0.4689}
}
```

Unknown parameter values return HTTP 400 with `{"error": "..."}`.

## Running it locally

```bash
python dev_server.py            # http://127.0.0.1:8099
```

Nothing beyond Python is needed for the JSON endpoints. The figure buttons and the
command-line matrix also need numpy and matplotlib:

```bash
pip install -r requirements.txt
```

## Simulation code (no web)

`simulasi/` holds the code that produces every number in the report tables.

```bash
cd simulasi
python simulasi_agv.py          # Python 3.8+, no packages, prints the metric tables
python run_sims.py              # ~60 s, writes figs/ and results/results.json
```

Downloading the repository as a ZIP and running `cd simulasi && python
simulasi_agv.py` reproduces the report numbers on any machine with Python.

## What is in the repository

```
index.html            web page (canvas, no external libraries)
api/simulate.py       plant, controllers, metrics and the figure renderer
dev_server.py         local server, same routing as the hosted version
simulasi/             command-line simulation behind the report tables
  agv/                params, plant (RK4), control, sim, metrics
  simulasi_agv.py     single-file, dependency-free simulation
  run_sims.py         full scenario matrix -> figures + results.json
```

## Notes

- One request runs one complete simulation; there is no streaming.
- JSON answers take about half a second; a figure takes 1–4 s, plus a few seconds
  on the first request while numpy and matplotlib load.
- Physical parameters are fixed at the values extracted from the CAD model and the
  motor datasheet; only mass and friction are exposed on the page.
- The heading channel has a Coulomb deadband of about 0.56 V, which is why the
  sinusoid response lags.

## Credits and licence

Kalila Effie Abida (13323075) · Aliya Lathifa Putri (13723069) · Nurul Na'im Natifah (18223106)
TF4107 Robotics and Automation — Project 2. Code provided for academic review.
