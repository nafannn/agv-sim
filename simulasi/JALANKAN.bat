@echo off
REM ============================================================
REM  Simulasi AGV line follower 4WD - TF4107 Tugas Proyek 2
REM  Klik dua kali berkas ini untuk menjalankan simulasi.
REM  Tidak perlu memasang apa pun selain Python 3.8+.
REM ============================================================
cd /d "%~dp0"

where py >nul 2>nul
if %errorlevel%==0 (
    py -3.12 simulasi_agv.py %*
    if errorlevel 1 py simulasi_agv.py %*
) else (
    python simulasi_agv.py %*
)

echo.
echo ============================================================
echo Selesai. Tekan tombol apa pun untuk menutup jendela ini.
pause >nul
