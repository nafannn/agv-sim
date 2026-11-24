# Simulasi AGV Line Follower 4WD — TF4107 Tugas Proyek 2

Aplikasi web + pustaka simulasi untuk AGV line follower 4WD. Mesin simulasi
berjalan sebagai **Python serverless function** di Vercel; antarmuka hanya
HTML + Canvas tanpa library eksternal.

**Demo:** `https://<proyek>.vercel.app` — pilih skenario & kontroler, animasi
AGV tampak atas, grafik kecepatan/heading, dan kartu metrik.

## Struktur

```
index.html            antarmuka web (statis)
api/simulate.py       fungsi Python: dinamika + RK4 + PID/state feedback + metrik
dev_server.py         server lokal untuk pengembangan
simulasi/             sumber simulasi desktop/CLI dan generator laporan
  agv/                params, plant (RK4), control, sim, metrics
  simulasi_agv.py     versi satu berkas tanpa dependensi (untuk penilai)
  run_sims.py         seluruh skenario → grafik + results.json
  make_report.py      menyusun dokumen Word dari results.json
  make_flow.py        flowchart algoritma kontrol
```

## Menjalankan lokal

```bash
python dev_server.py            # http://127.0.0.1:8099
```

Butuh Python 3.9+ saja (tanpa numpy/matplotlib untuk versi web).

Versi CLI tanpa dependensi:

```bash
python simulasi/simulasi_agv.py                  # semua skenario, tabel metrik
python simulasi/simulasi_agv.py --bandingkan      # PID vs PID satu loop vs SF
python simulasi/simulasi_agv.py --csv hasil_csv   # ekspor CSV
```

## API

`GET /api/simulate?skenario=step&kontroler=pid&m_scale=1&mu_scale=1`

- `skenario`: `step`, `multi`, `ramp`, `gangguan`, `psi90`, `psi180`, `sinus`
- `kontroler`: `pid` (cascade + feedforward), `pid1` (satu loop), `sf` (state feedback)
- `m_scale`, `mu_scale`: pengali massa/inersia dan gesek (uji robustness)

Balasan JSON: `metrik{...}`, deret waktu `t, v, v_ref, psi_deg, psi_ref_deg,
VL, VR, x, y`, dan `params`.

## Deploy

1. Push repo ini ke GitHub.
2. Vercel → **Add New → Project → Import Git Repository**.
3. Framework Preset **Other**, Build Command kosong, Output Directory kosong.
4. Deploy. `api/simulate.py` dikenali otomatis sebagai Python function;
   `simulasi/` tidak ikut ter-deploy (lihat `.vercelignore`); laporan dan gambar
   disimpan di luar repo.

## Angka acuan

| Skenario | Kontroler | t_s | MP | galat tunak |
|---|---|---|---|---|
| kecepatan 0 → 0,35 m/s | PID | 0,650 s | 0 % | 8·10⁻⁷ m/s |
| kecepatan 0 → 0,35 m/s | SF | 0,650 s | 0 % | < 10⁻⁶ m/s |
| heading 0 → 90° | PID | 1,030 s | 1,02 % | 0,096° |
| heading 0 → 90° | SF | 1,240 s | 1,18 % | 0,175° |
| sinusoid 0,2 Hz | PID | — | — | RMSE 8,7° |

## Lisensi

Kode untuk keperluan akademik TF4107 Tugas Proyek 2.
