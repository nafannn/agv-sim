# Kode simulasi AGV (CLI)

Kode yang menghasilkan seluruh angka pada tabel 4.6, 4.7, 5.7, dan Bab 6.
Plant nonlinear 7 state dengan RK4 tulis sendiri, kontroler diskret 10 ms,
tunda 15 ms, saturasi 6 V. Tidak memakai `step()`, `lsim()`, `tf()`, `ss()`,
maupun `feedback()`.

## Tanpa memasang apa pun

```bash
python simulasi_agv.py
```

Python 3.8+ saja. Keluarannya tabel metrik semua skenario:

```
[gangguan]  t_r = 0.270 s | t_s (2%) = 0.650 s | MP = 0.00 % | IAE = 0.0524
[psi90]     t_r = 0.570 s | t_s (2%) = 1.030 s | MP = 1.02 % | e_ss = 0.09627 deg
```

Opsi: `--skenario psi90` · `--kontroler sf` · `--bandingkan` · `--csv folder` ·
`--plot` (butuh matplotlib). Di Windows bisa klik dua kali `JALANKAN.bat`.

## Pilih bagian yang dijalankan

Tidak harus semuanya. Ada tiga cara:

```bash
python run_sims.py --daftar            # lihat kode bagian yang tersedia
python run_sims.py --bagian 4.6,6A     # hanya dua bagian itu
python run_sims.py --pilih             # menu interaktif, pilih nomor
```

Kode bagian: `4.6` kontrol kecepatan, `4.7` kontrol posisi, `5.7` state feedback,
`6A` perbandingan kecepatan, `6B` perbandingan posisi, `6C` robustness,
`6D` variasi motor. Tanpa argumen = semua.

Beberapa bagian memakai hasil bagian lain, jadi ikut dijalankan otomatis dan
diberitahukan di layar (mis. `6A` butuh `4.6`, `4.7`, dan `5.7`). Bila yang
dijalankan hanya sebagian, angkanya disimpan ke `results/results_bagian_<kode>.json`
agar `results/results.json` tidak tertimpa.

## Semua skenario + grafik

```bash
pip install -r requirements.txt
python run_sims.py
```

±60 detik, menghasilkan `figs/` (13 grafik) dan `results/results.json`. Skrip
membuat kedua folder itu sendiri.

| Skenario | Kontroler | t_s (2%) | Lonjakan |
|---|---|---|---|
| Tangga 0 → 0,35 m/s | PID cascade | 0,650 s | 0,00 % |
| Tangga 0 → 0,35 m/s | State feedback | 0,650 s | 0,00 % |
| Heading 0 → 90° | PID cascade | 1,030 s | 1,02 % |
| Heading 0 → 90° | State feedback | 1,240 s | 1,18 % |
| Heading 0 → 90° | PID satu loop | 3,160 s | 3,60 % |

Sidik jari hasil: SHA-256 `results/results.json` diawali `2427257142cf673c`.

## Struktur

```
agv/params.py      parameter fisis + koefisien turunan
agv/plant.py       persamaan dinamik 7 state + RK4 manual
agv/control.py     PI kecepatan, cascade heading, PID 1 loop, state feedback
agv/sim.py         loop simulasi: ZOH 10 ms, tunda 15 ms, saturasi 6 V
agv/metrics.py     rise time, settling, lonjakan, IAE, RMSE
run_sims.py        seluruh skenario -> figs/ + results/results.json
simulasi_agv.py    versi satu berkas tanpa dependensi
demo.py            satu skenario cepat di terminal
```
