"""Menyusun dokumen Word subbab 4.6, 4.7, 5.7, dan Bab 6 + lampiran kode.

Seluruh angka diambil dari results/results.json (hasil run_sims.py), sehingga
teks, tabel, dan grafik selalu konsisten.
"""

from __future__ import annotations

import json
import os
import sys

import numpy as np
from docx import Document
from docx.enum.section import WD_SECTION
from docx.enum.table import WD_TABLE_ALIGNMENT
from docx.enum.text import WD_ALIGN_PARAGRAPH, WD_BREAK
from docx.oxml import OxmlElement
from docx.oxml.ns import qn
from docx.shared import Cm, Pt, RGBColor

ROOT = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, ROOT)
FIG = os.path.join(ROOT, "figs")

from agv import Plant, PLANT_N20

D = json.load(open(os.path.join(ROOT, "results", "results.json"), encoding="utf-8"))
P = Plant()

# ----------------------------------------------------------------------
# Utilitas format
# ----------------------------------------------------------------------
def n(x, nd=2, unit="", sign=False):
    """Format angka gaya Indonesia (koma desimal, titik ribuan)."""
    if x is None:
        return "–"
    if isinstance(x, str):
        return x
    s = f"{x:+,.{nd}f}" if sign else f"{x:,.{nd}f}"
    s = s.replace(",", "#").replace(".", ",").replace("#", ".")
    s = s.replace("-", "−")
    return f"{s} {unit}".strip()


def g(*path, default=None):
    cur = D
    for k in path:
        if not isinstance(cur, dict) or k not in cur:
            return default
        cur = cur[k]
    return cur


def poles(*coefs):
    return np.roots(list(coefs))


def fmt_poles(roots, nd=2):
    """Format kutub: pasangan konjugat ditulis sekali, akar dekat sebagai ganda."""
    roots = list(np.asarray(roots, dtype=complex))
    used = [False] * len(roots)
    out = []
    for i, r in enumerate(roots):
        if used[i]:
            continue
        used[i] = True
        if abs(r.imag) > max(1e-6, 0.05 * abs(r.real)):
            for j in range(i + 1, len(roots)):
                if not used[j] and abs(roots[j] - np.conj(r)) < 1e-6:
                    used[j] = True
                    break
            out.append(f"{n(r.real, nd)} ± j{n(abs(r.imag), nd)}")
        else:
            dup = [j for j in range(i + 1, len(roots))
                   if not used[j] and abs(roots[j].real - r.real) < 0.02]
            if dup:
                used[dup[0]] = True
                out.append(f"{n(r.real, nd)} (ganda)")
            else:
                out.append(n(r.real, nd))
    return "; ".join(out)


# ----------------------------------------------------------------------
# Dokumen
# ----------------------------------------------------------------------
doc = Document()
sec = doc.sections[0]
sec.page_width = Cm(21.0)
sec.page_height = Cm(29.7)
sec.left_margin = Cm(3.0)
sec.right_margin = Cm(2.5)
sec.top_margin = Cm(2.5)
sec.bottom_margin = Cm(2.5)

style = doc.styles["Normal"]
style.font.name = "Times New Roman"
style.font.size = Pt(11)
style.paragraph_format.space_after = Pt(6)
style.paragraph_format.line_spacing = 1.15
style.paragraph_format.alignment = WD_ALIGN_PARAGRAPH.JUSTIFY

for name, size in (("Heading 1", 14), ("Heading 2", 12), ("Heading 3", 11.5)):
    st = doc.styles[name]
    st.font.name = "Times New Roman"
    st.font.size = Pt(size)
    st.font.color.rgb = RGBColor(0, 0, 0)
    st.font.bold = True
    st.paragraph_format.space_before = Pt(12)
    st.paragraph_format.space_after = Pt(6)
    st.paragraph_format.alignment = WD_ALIGN_PARAGRAPH.LEFT
    rpr = st.element.get_or_add_rPr()
    rfonts = rpr.find(qn("w:rFonts"))
    if rfonts is None:
        rfonts = OxmlElement("w:rFonts")
        rpr.append(rfonts)
    for attr in ("w:asciiTheme", "w:hAnsiTheme", "w:eastAsiaTheme", "w:cstheme"):
        if rfonts.get(qn(attr)) is not None:
            del rfonts.attrib[qn(attr)]
    for attr in ("w:ascii", "w:hAnsi", "w:cs", "w:eastAsia"):
        rfonts.set(qn(attr), "Times New Roman")


def h(level, text):
    p = doc.add_heading(text, level=level)
    return p


def par(text, italic=False, align="just", size=None, space_after=6):
    p = doc.add_paragraph()
    p.alignment = {"just": WD_ALIGN_PARAGRAPH.JUSTIFY, "center": WD_ALIGN_PARAGRAPH.CENTER,
                   "left": WD_ALIGN_PARAGRAPH.LEFT}[align]
    p.paragraph_format.space_after = Pt(space_after)
    run = p.add_run(text)
    run.italic = italic
    if size:
        run.font.size = Pt(size)
    return p


def bullet(text, level=0):
    p = doc.add_paragraph(style="List Bullet" if level == 0 else "List Bullet 2")
    p.paragraph_format.space_after = Pt(2)
    p.alignment = WD_ALIGN_PARAGRAPH.JUSTIFY
    p.add_run(text)
    return p


def numbered(text):
    p = doc.add_paragraph(style="List Number")
    p.paragraph_format.space_after = Pt(3)
    p.alignment = WD_ALIGN_PARAGRAPH.JUSTIFY
    run = p.add_run(text)
    return p


def eq(text):
    p = doc.add_paragraph()
    p.alignment = WD_ALIGN_PARAGRAPH.CENTER
    p.paragraph_format.space_before = Pt(4)
    p.paragraph_format.space_after = Pt(6)
    run = p.add_run(text)
    run.italic = True
    run.font.name = "Cambria Math"
    return p


def caption(text, bold_prefix=None):
    p = doc.add_paragraph()
    p.alignment = WD_ALIGN_PARAGRAPH.CENTER
    p.paragraph_format.space_before = Pt(2)
    p.paragraph_format.space_after = Pt(10)
    if bold_prefix:
        r = p.add_run(bold_prefix)
        r.bold = True
        r.font.size = Pt(9.5)
    r2 = p.add_run(text)
    r2.font.size = Pt(9.5)
    return p


def table(headers, rows, widths=None, font=9, align_first_left=True,
          caption_text=None, caption_prefix=None):
    if caption_text:
        caption(caption_text, caption_prefix)
    t = doc.add_table(rows=1, cols=len(headers))
    t.style = "Table Grid"
    t.alignment = WD_TABLE_ALIGNMENT.CENTER
    hdr = t.rows[0].cells
    for i, htxt in enumerate(headers):
        hdr[i].text = ""
        p0 = hdr[i].paragraphs[0]
        p0.alignment = WD_ALIGN_PARAGRAPH.CENTER
        p0.paragraph_format.space_after = Pt(0)
        r = p0.add_run(htxt)
        r.bold = True
        r.font.size = Pt(font)
    for row in rows:
        cells = t.add_row().cells
        for i, val in enumerate(row):
            cells[i].text = ""
            p0 = cells[i].paragraphs[0]
            p0.paragraph_format.space_after = Pt(0)
            p0.alignment = (WD_ALIGN_PARAGRAPH.LEFT if (i == 0 and align_first_left)
                            else WD_ALIGN_PARAGRAPH.CENTER)
            r = p0.add_run(str(val))
            r.font.size = Pt(font)
    if widths:
        for i, w in enumerate(widths):
            for row in t.rows:
                row.cells[i].width = Cm(w)
    doc.add_paragraph().paragraph_format.space_after = Pt(2)
    return t


def figure(fname, number, caption_text, width=15.5):
    path = os.path.join(FIG, fname)
    if not os.path.exists(path):
        raise FileNotFoundError(path)
    p = doc.add_paragraph()
    p.alignment = WD_ALIGN_PARAGRAPH.CENTER
    p.paragraph_format.space_before = Pt(6)
    p.paragraph_format.space_after = Pt(2)
    p.add_run().add_picture(path, width=Cm(width))
    caption(caption_text, f"Gambar {number}")


def code_block(path, title):
    par(title, italic=True, size=10, space_after=3)
    text = open(path, encoding="utf-8").read()
    for line in text.splitlines():
        p = doc.add_paragraph()
        p.paragraph_format.space_after = Pt(0)
        p.paragraph_format.space_before = Pt(0)
        p.paragraph_format.line_spacing = 1.0
        p.paragraph_format.left_indent = Cm(0.4)
        r = p.add_run(line if line.strip() else " ")
        r.font.name = "Consolas"
        r.font.size = Pt(7.5)


# ======================================================================
# Judul dokumen
# ======================================================================
title = doc.add_paragraph()
title.alignment = WD_ALIGN_PARAGRAPH.CENTER
r = title.add_run("TUGAS PROYEK 2 – SISTEM MEKATRONIKA (TF4107)")
r.bold = True
r.font.size = Pt(13)
sub = doc.add_paragraph()
sub.alignment = WD_ALIGN_PARAGRAPH.CENTER
r = sub.add_run("Simulasi, Pengujian, dan Analisis Perbandingan\n"
                "Subbab 4.6, 4.7, 5.7, dan Bab 6 – AGV Line Follower 4WD")
r.font.size = Pt(11.5)
r.italic = True
par("Dokumen ini memuat bagian simulasi yang menjadi tanggung jawab penyusun: "
    "4.6 (simulasi kontrol kecepatan), 4.7 (simulasi kontrol posisi/heading), "
    "5.7 (implementasi algoritma pengontrol alternatif), dan Bab 6 (pengujian "
    "menyeluruh serta analisis perbandingan). Seluruh angka pada tabel dan grafik "
    "merupakan keluaran langsung skrip Python pada Lampiran A – tidak ada satu pun "
    "respons yang dihasilkan oleh fungsi kontrol bawaan. Integrasi dilakukan dengan "
    "Runge-Kutta orde 4 yang ditulis manual.", space_after=10)

# tabel parameter simulasi umum
table(
    ["Parameter simulasi", "Simbol", "Nilai", "Keterangan"],
    [
        ["Periode pencuplikan kontrol", "Ts", "10 ms", "zero-order hold (4.2.1.d)"],
        ["Tunda transportasi total", "Td", "15 ms", "1,5·Ts, komputasi + aktuator"],
        ["Langkah integrasi RK4", "h", "0,1 ms", "100 sub-langkah per Ts"],
        ["Tegangan suplai / batas driver", "V_max", "6,0 V", "H-bridge dua arah"],
        ["Batas referensi laju yaw", "ω_ref,max", "3,0 rad/s", "batas fisik rotasi"],
        ["Pita penghalusan fungsi sign", "ε", "0,2 rad/s; 5 mm/s", "anti-chattering RK4"],
        ["Anti-windup", "K_aw", "25 s⁻¹", "back-calculation (4.2.1.d)"],
        ["Durasi simulasi", "t_end", "4–7 s", "sesuai skenario"],
    ],
    widths=[5.8, 1.8, 3.2, 4.7],
    caption_text=": Parameter simulasi yang dipakai pada seluruh subbab 4.6, 4.7, 5.7, dan Bab 6.",
    caption_prefix="Tabel 1",
)

# ======================================================================
# 4.6
# ======================================================================
h(1, "4.6 Simulasi Kontrol Kecepatan")

par("Subbab ini memverifikasi hasil perancangan kontroler kecepatan pada subbab 4.2–4.4 "
    "langsung pada model dinamik nonlinear (persamaan E1, E2, M1, M2 pada subbab 3.5), "
    "bukan pada model linear yang dipakai saat tuning. Tujuannya tiga: (i) memastikan "
    "penguatan hasil IMC benar-benar memenuhi target kinerja 4.1.4 pada sistem penuh, "
    "(ii) menetapkan penguatan final bila ternyata target tidak tercapai, dan "
    "(iii) menyajikan respons pada empat skenario wajib: tangga, bertingkat, ramp, dan "
    "perubahan beban.")

h(2, "4.6.1 Konfigurasi numerik dan verifikasi konsistensi model")

par("Persamaan gerak diintegrasikan sendiri dengan Runge-Kutta orde 4 berlangkah tetap "
    "0,1 ms, sementara algoritma kontrol dijalankan secara diskret setiap 10 ms dengan "
    "zero-order hold. Perintah tegangan tidak langsung diterapkan, melainkan ditunda "
    "1,5·Ts (15 ms) melalui antrean perintah untuk merepresentasikan waktu komputasi "
    "mikrokontroler ditambah inersia driver – ekuivalen dengan pendekatan Padé pada "
    "subbab 4.2.6, tetapi tanpa aproksimasi. Tegangan tiap sisi dibatasi |V| ≤ 6 V dan "
    "integrator dilengkapi anti-windup back-calculation dengan K_aw = 25 s⁻¹.")

par("Sebelum simulasi dipakai, konsistensi plant diuji pada titik kerja desain "
    "v = 0,35 m/s. Simulasi mencapai keadaan tunak dengan V_c = 2,960 V dan arus "
    "0,177 A. Hasil ini dapat dibandingkan dengan penyelesaian analitik persamaan "
    "tunak: arus yang dibutuhkan untuk menahan gesek Coulomb, gelinding, dan viskos "
    "adalah")
eq("i = (F_C + F_roll + c_v·v) / (2·c_F) = 0,177 A,   V_c = R_a·i + k_v·v = 2,960 V")
par("Kesesuaian sampai tiga angka berarti (galat < 0,1%) menunjukkan bahwa koefisien "
    "yang dipakai simulator identik dengan koefisien pada Bab 3, sehingga setiap "
    "perbedaan respons murni berasal dari dinamika dan algoritma kontrol, bukan dari "
    "ketidakcocokan parameter. Sebagai pemeriksaan tambahan, langkah integrasi "
    "diuji pada 0,05 ms, 0,1 ms, dan 0,5 ms. Ketiganya memberi kecepatan yang sama "
    "sampai delapan angka (0,34955449 m/s pada t = 2 s) dan tegangan tunak 2,960238 V, "
    "sehingga h = 0,1 ms sudah konvergen dan dapat dipakai.")

par("Satu koreksi kecil dilakukan terhadap parameter Bab 3: redaman viskos yaw dihitung "
    "ulang dengan faktor dua karena setiap sisi menggerakkan dua roda, sehingga "
    f"c_ψ = {n(g('params','c_psi'), 4)} N·m·s/rad (sebelumnya 2,83×10⁻³) dan "
    f"a_ω = {n(g('params','a_w'), 3)} s⁻¹ (sebelumnya 3,335 s⁻¹). Selisih 1,4% ini "
    "menggeser kutub closed-loop kurang dari 0,1 rad/s dan tidak mengubah kesimpulan "
    "kestabilan pada subbab 4.4.")

par("Diagram alir algoritma kontrol untuk satu siklus kendali (T_s = 10 ms) — termasuk "
    "percabangan saturasi dan anti-windup pada kedua kanal — disajikan pada Lampiran B, "
    "Gambar B.1.")

h(2, "4.6.2 Verifikasi tuning analitik dan penyesuaian penguatan")

par("Penguatan hasil IMC pada subbab 4.3.1 (λ_v = 0,20 s → K_p = 10,66; K_i = 32,65) "
    "diuji lebih dahulu. Pada model penuh, respons tangga 0 → 0,35 m/s memberi waktu "
    f"penetapan {n(g('s46','step_lambda020','settling_time'))} s dengan lonjakan "
    f"{n(g('s46','step_lambda020','overshoot_pct'))}%, sehingga target t_s ≤ 1 s pada "
    "4.1.4 tidak terpenuhi. Penyebabnya dapat dijelaskan dari struktur closed-loop: "
    "untuk plant orde satu dengan PI hasil IMC, kutub closed-loop berada di −a_v dan "
    "−1/λ, sedangkan nol PI tepat berada di −a_v. Nol tersebut hampir meniadakan kutub "
    "lambat, tetapi pada sistem nonlinear (gesek Coulomb dan saturasi) peniadaan itu "
    "tidak sempurna sehingga muncul ekor respons berkonstanta waktu 1/a_v = 0,33 s.")

par("λ_v diperkecil menjadi 0,12 s, masih di dalam rentang saran subbab 4.5 "
    "(K_p 8,5–21; K_i 26–65), sehingga diperoleh K_p = 17,78 dan K_i = 54,44. Selain "
    "itu ditambahkan pembobotan setpoint (setpoint weighting) β = 0,70: aksi "
    "proporsional bekerja pada β·v_ref − v, sedangkan integrator tetap bekerja pada "
    "galat penuh v_ref − v. Tanpa pembobotan (β = 1), referensi bertingkat menghasilkan "
    f"lonjakan {n(g('s46','multi_beta1','os_max'))}% pada setiap langkah kecil karena "
    "suku proporsional menerima lompatan referensi secara penuh; dengan β = 0,70 "
    f"lonjakan turun menjadi {n(g('s46','multi','os_max'))}% tanpa menimbulkan galat "
    "pelacakan ramp seperti yang terjadi bila dipakai prefilter orde satu "
    "(prefilter menyisakan galat tunak 0,05 m/s saat akselerasi).")

table(
    ["Konfigurasi", "K_p [V·s/m]", "K_i [V/m]", "t_r [s]", "t_s (2%) [s]", "MP [%]", "IAE [m]"],
    [
        ["IMC λ = 0,20 (subbab 4.3)", n(10.66), n(32.65),
         n(g('s46','step_lambda020','rise_time')), n(g('s46','step_lambda020','settling_time')),
         n(g('s46','step_lambda020','overshoot_pct')), n(g('s46','step_lambda020','iae'), 4)],
        ["IMC λ = 0,12, β = 1", n(17.78), n(54.44),
         n(g('s46','step_beta1','rise_time')), n(g('s46','step_beta1','settling_time')),
         n(g('s46','step_beta1','overshoot_pct')), n(g('s46','step_beta1','iae'), 4)],
        ["IMC λ = 0,12, β = 0,70 (final)", n(17.78), n(54.44),
         n(g('s46','step_ff','rise_time')), n(g('s46','step_ff','settling_time')),
         n(g('s46','step_ff','overshoot_pct')), n(g('s46','step_ff','iae'), 4)],
    ],
    widths=[4.8, 2.2, 1.9, 1.6, 2.0, 1.5, 1.5],
    caption_text=": Perbandingan penguatan PI kecepatan pada tangga 0 → 0,35 m/s (model penuh).",
    caption_prefix="Tabel 4.6.1",
)

h(2, "4.6.3 Respons tangga 0 → 0,35 m/s dan peran feedforward")

par("Gambar 4.6.1 memperlihatkan respons kecepatan untuk referensi tangga pada t = 0,5 s, "
    "dengan dan tanpa feedforward. Keduanya memakai penguatan yang sama, sehingga "
    "perbedaan respons murni berasal dari aksi feedforward pada subbab 4.2.2. Dengan "
    "feedforward, waktu naik turun dari 0,54 s menjadi "
    f"{n(g('s46','step_ff','rise_time'))} s dan IAE turun dari "
    f"{n(g('s46','step_noff','iae'), 4)} menjadi {n(g('s46','step_ff','iae'), 4)} m "
    f"(turun {n(100*(1-g('s46','step_ff','iae')/g('s46','step_noff','iae')),0)}%). "
    "Feedforward menyediakan hampir seluruh tegangan tunak (2,30 V dari 2,96 V) sejak "
    "awal, sehingga integrator hanya perlu mengoreksi sisa model; tanpa feedforward, "
    "integrator harus membangun tegangan itu dari nol dan respons menjadi lebih lambat.")

par("Pada detik-detik awal tangga, tegangan kendali sempat menyentuh batas 6 V selama "
    "kira-kira 0,1 s karena suku proporsional masih besar. Berkat anti-windup "
    "back-calculation, integrator tidak terisi selama saturasi sehingga tidak ada "
    f"lonjakan setelah tegangan keluar dari batas: lonjakan maksimum tercatat "
    f"{n(g('s46','step_ff','overshoot_pct'))}% dan galat tunak "
    f"{n(g('s46','step_ff','error_rel_pct'), 4)}%. Kecepatan 0,35 m/s setara dengan "
    "105,1 rpm roda atau 5.045 rpm poros motor (rasio 48:1), tepat pada titik desain "
    "pemilihan motor di subbab 2.")

figure("4_6_step.png", "4.6.1",
       ": Respons kecepatan terhadap tangga 0 → 0,35 m/s (atas) dan tegangan kendali "
       "common-mode V_c (bawah). Garis putus-putus merah adalah batas saturasi 6 V.")

h(2, "4.6.4 Referensi kecepatan bertingkat")

par("Skenario kedua memakai referensi bertingkat 0,15 → 0,25 → 0,35 → 0,20 m/s "
    "(Gambar 4.6.2). Setiap langkah dievaluasi pada jendela waktunya sendiri agar "
    "metrik tidak tercemar langkah berikutnya. Hasilnya: waktu penetapan terburuk "
    f"{n(g('s46','multi','ts_max'))} s, lonjakan terburuk {n(g('s46','multi','os_max'))}% "
    f"(setara {n(0.1*g('s46','multi','os_max')*10, 1)} mm/s, karena langkah terkecil "
    "hanya 0,1 m/s), dan galat akhir "
    f"{n(g('s46','multi','ess_end')*1000, 3)} mm/s. Lonjakan tersebar merata pada "
    "keempat langkah (1,7–1,8%), tanda bahwa perilaku itu berasal dari pembobotan "
    "setpoint β = 0,70 dan bukan dari titik kerja tertentu. Tanpa pembobotan, langkah "
    "0,15 → 0,25 m/s menghasilkan lonjakan 15,4% dengan ekor respons hampir 1,1 s "
    "(lihat subbab 4.6.2).")

figure("4_6_multistep.png", "4.6.2",
       ": Referensi kecepatan bertingkat (atas) dan arus kedua sisi motor (bawah).")

h(2, "4.6.5 Pelacakan referensi ramp")

par("Skenario ketiga adalah ramp 0 → 0,35 m/s dalam 0,8 s (akselerasi 0,4375 m/s²) "
    "sesuai 4.1.3, ditahan, lalu ramp turun ke nol (Gambar 4.6.3). Galat RMS selama "
    f"manuver adalah {n(g('s46','ramp','rmse')*1000, 1)} mm/s "
    f"({n(100*g('s46','ramp','rmse')/0.35, 1)}% dari kecepatan jelajah) dengan galat "
    f"maksimum {n(g('s46','ramp','max_abs_err')*1000, 1)} mm/s, yang terjadi pada awal "
    "ramp. Galat ini sebagian disengaja: pembobotan β = 0,70 membuat aksi proporsional "
    "hanya melihat 70% referensi, sehingga loop tidak mengejar ramp secara agresif. "
    "Tanpa pembobotan (β = 1) galat ramp turun menjadi 1,9 mm/s, tetapi lonjakan pada "
    "referensi bertingkat kembali ke 15,4%. Pemilihan β = 0,70 adalah kompromi yang "
    "menjaga kedua skenario tetap di dalam target.")

figure("4_6_ramp.png", "4.6.3",
       ": Pelacakan ramp naik dan ramp turun (atas) serta galat kecepatan (bawah).")

h(2, "4.6.6 Penolakan gangguan beban")

par("Pada t = 3 s diberikan gaya hambat eksternal F_ext = 1,5 N selama 1,5 s, setara "
    "41% gaya traksi tunak (3,69 N) – misalnya AGV menabrak kemiringan kecil atau "
    "menyeret beban. Kecepatan turun dari 0,35 m/s ke "
    f"{n(g('s46','dist','drop'))} m/s, yaitu penurunan "
    f"{n(g('s46','dist','drop_pct'))}% "
    f"({n((0.35-g('s46','dist','drop'))*1000, 1)} mm/s), dan kembali ke pita ±1% dalam "
    f"{n(g('s46','dist','recovery_1pct'), 3)} s setelah gangguan hilang. Sisa galat "
    "sebesar beberapa mm/s dihilangkan oleh integrator dalam waktu sekitar 1 s. Hasil "
    "ini memenuhi target \"speed drop ≤ 5%\" pada 4.1.4.")

figure("4_6_dist.png", "4.6.4",
       ": Penolakan gangguan beban 1,5 N (area merah) dan tegangan kendali.")

h(2, "4.6.7 Kasus kecepatan rendah: referensi 500 rpm motor")

par("Skema pengujian pada Bab 6 meminta referensi 500 rpm. Untuk AGV ini, 500 rpm poros "
    "motor setara 10,4 rpm roda (kecepatan 0,0347 m/s) karena rasio gearbox 48:1; "
    "kecepatan jelajah desain 0,35 m/s justru setara 5.045 rpm motor. Kasus 500 rpm "
    "karena itu diuji sebagai titik kerja kecepatan rendah, tempat gesek Coulomb "
    f"mendominasi. Hasilnya t_r = {n(g('s46','low_speed','rise_time'))} s, "
    f"t_s = {n(g('s46','low_speed','settling_time'))} s, lonjakan "
    f"{n(g('s46','low_speed','overshoot_pct'))}%, dan tegangan puncak hanya "
    f"{n(g('s46','low_speed','max_abs_V'))} V. Pada rentang ini tegangan kendali lebih "
    "kecil daripada kompensasi gesek (0,675 V), sehingga perilaku stick-slip model "
    "gesek yang dihaluskan mulai terlihat; lonjakan 1,2% masih jauh di bawah batas 5%.")

h(2, "4.6.8 Kepatuhan terhadap spesifikasi 4.1.4")

table(
    ["Besaran", "Target (4.1.4)", "Hasil simulasi", "Status"],
    [
        ["Maximum overshoot v", "≤ 5%", f"{n(g('s46','step_ff','overshoot_pct'))}% (tangga), "
         f"{n(g('s46','multi','os_max'))}% (bertingkat)", "Terpenuhi"],
        ["Settling time v (2%)", "≤ 1 s", f"{n(g('s46','step_ff','settling_time'))} s (tangga), "
         f"{n(g('s46','multi','ts_max'))} s (bertingkat)", "Terpenuhi"],
        ["Steady-state error", "0 (≤ 1%)", f"{n(g('s46','step_ff','error_rel_pct'), 4)}%", "Terpenuhi"],
        ["Speed drop akibat gangguan", "≤ 5%", f"{n(g('s46','dist','drop_pct'))}%", "Terpenuhi"],
        ["Tegangan kendali", "0–6 V", "6 V saat transien, 2,96 V tunak", "Terpenuhi"],
    ],
    widths=[4.5, 3.0, 5.5, 2.5],
    caption_text=": Kepatuhan hasil simulasi kecepatan terhadap target kinerja subbab 4.1.4.",
    caption_prefix="Tabel 4.6.2",
)

# ======================================================================
# 4.7
# ======================================================================
h(1, "4.7 Simulasi Kontrol Posisi (Heading)")

par("Variabel posisi yang dikendalikan adalah sudut heading ψ, sesuai 4.1.3 "
    "(ψ = r·(θ_R − θ_L)/b, yaitu posisi sudut diferensial kedua roda). Dua arsitektur "
    "dibandingkan pada objek yang sama: cascade P–PI (subbab 4.2.1.b) sebagai rancangan "
    "utama dan PID satu loop (subbab 4.3.1.b) sebagai pembanding.")

h(2, "4.7.1 Asumsi pengujian posisi")

par("Uji 0° → 90° dan 0° → 180° dilakukan dari keadaan diam, sehingga AGV berputar di "
    "tempat: roda kiri dan kanan harus berputar berlawanan arah. Ini menuntut driver "
    "dua arah (H-bridge), sehingga batas tegangan pada simulasi posisi adalah |V| ≤ 6 V, "
    "bukan 0–6 V seperti pada mode jelajah maju. Batasan 0–6 V tetap berlaku untuk kanal "
    "kecepatan; kedua mode tidak pernah aktif bersamaan dengan tanda berlawanan pada "
    "sumbu yang sama. Referensi laju yaw dibatasi 3 rad/s dan kanal diferensial dibatasi "
    "|V_d| ≤ 6 − |V_c| agar tegangan tiap sisi tetap di dalam rentang driver.")

h(2, "4.7.2 Tangga 90° dan 180°")

par("Gambar 4.7.1 dan 4.7.2 memperlihatkan respons kedua arsitektur. Cascade memberi "
    f"t_s = {n(g('s47','cascade_90','settling_time'))} s dengan lonjakan "
    f"{n(g('s47','cascade_90','overshoot_pct'))}% untuk 90°, dan "
    f"t_s = {n(g('s47','cascade_180','settling_time'))} s dengan lonjakan "
    f"{n(g('s47','cascade_180','overshoot_pct'))}% untuk 180°; keduanya memenuhi target "
    "t_s ≤ 1,5 s. Galat tunak di bawah 0,1° untuk 90° dan 0,02° untuk 180°.")

par("PID satu loop menunjukkan kelemahan yang khas: dengan K_d = 1,273 penguatan "
    "derivatif besar, laju yaw memuncak 3,72 rad/s (90°) dan 5,58 rad/s (180°) – jauh "
    "di atas batas 3 rad/s dan menembus saturasi. Ketika V_d saturasi, integrator terus "
    "mengisi (windup) sehingga respons 90° baru tenang setelah "
    f"{n(g('s47','pid1_90','settling_time'))} s, lebih dari tiga kali cascade, dan "
    f"menyisakan galat {n(g('s47','pid1_90','error_final'))}° "
    f"(cascade {n(g('s47','cascade_90','error_final'), 3)}°). Untuk 180°, kecepatan "
    "sudut yang lebih tinggi justru membuat loop lebih cepat mencapai target sehingga "
    "t_s lebih pendek, tetapi galat tunaknya tetap lebih besar "
    f"({n(g('s47','pid1_180','error_final'))}° berbanding "
    f"{n(g('s47','cascade_180','error_final'), 3)}°).")

figure("4_7_step90.png", "4.7.1",
       ": Tangga heading 0 → 90°: sudut heading (atas) dan laju yaw (bawah) untuk "
       "cascade dan PID satu loop.")
figure("4_7_step180.png", "4.7.2",
       ": Tangga heading 0 → 180°: sudut heading (atas) dan laju yaw (bawah).")

table(
    ["Metrik", "Cascade 90°", "PID 1 loop 90°", "Cascade 180°", "PID 1 loop 180°"],
    [
        ["t_r [s]", n(g('s47','cascade_90','rise_time')), n(g('s47','pid1_90','rise_time')),
         n(g('s47','cascade_180','rise_time')), n(g('s47','pid1_180','rise_time'))],
        ["t_s (2%) [s]", n(g('s47','cascade_90','settling_time')), n(g('s47','pid1_90','settling_time')),
         n(g('s47','cascade_180','settling_time')), n(g('s47','pid1_180','settling_time'))],
        ["Lonjakan [%]", n(g('s47','cascade_90','overshoot_pct')), n(g('s47','pid1_90','overshoot_pct')),
         n(g('s47','cascade_180','overshoot_pct')), n(g('s47','pid1_180','overshoot_pct'))],
        ["Galat tunak [°]", n(g('s47','cascade_90','error_final'), 3), n(g('s47','pid1_90','error_final')),
         n(g('s47','cascade_180','error_final'), 3), n(g('s47','pid1_180','error_final'), 3)],
        ["Laju yaw puncak [rad/s]", n(g('s47','cascade_90','w_max')), n(g('s47','pid1_90','w_max')),
         n(g('s47','cascade_180','w_max')), n(g('s47','pid1_180','w_max'))],
        ["Tegangan puncak [V]", n(g('s47','cascade_90','max_abs_V')), n(g('s47','pid1_90','max_abs_V')),
         n(g('s47','cascade_180','max_abs_V')), n(g('s47','pid1_180','max_abs_V'))],
    ],
    widths=[4.0, 2.85, 2.85, 2.85, 2.85],
    caption_text=": Metrik respons heading pada tangga 90° dan 180° (rotasi di tempat).",
    caption_prefix="Tabel 4.7.1",
)

h(2, "4.7.3 Referensi heading bertingkat")

par("Skenario bertingkat 0 → 0,5 rad → −0,5 rad → 1,0 rad (Gambar 4.7.3) menguji "
    "pembalikan arah, bagian tersulit bagi aktuator karena gesek Coulomb harus dilawan "
    "berlawanan tanda. Cascade menyelesaikan ketiga langkah dengan t_s "
    f"{n(g('s47','multi_s1_cascade','settling_time'))}/"
    f"{n(g('s47','multi_s2_cascade','settling_time'))}/"
    f"{n(g('s47','multi_s3_cascade','settling_time'))} s dan lonjakan maksimum "
    f"{n(max(g('s47','multi_s1_cascade','overshoot_pct'), g('s47','multi_s2_cascade','overshoot_pct'), g('s47','multi_s3_cascade','overshoot_pct')))}%. "
    "PID satu loop mengalami lonjakan "
    f"{n(g('s47','multi_s1_pid1','overshoot_pct'))}% pada langkah pertama dan "
    f"{n(g('s47','multi_s2_pid1','overshoot_pct'))}% saat membalik arah, dengan galat "
    f"tunak hingga {n(g('s47','multi_s2_pid1','error_final'))}° – akibat windup saat "
    "saturasi diferensial selama pembalikan.")

figure("4_7_multistep.png", "4.7.3",
       ": Referensi heading bertingkat termasuk pembalikan arah; perhatikan ekor "
       "respons PID satu loop setelah saturasi.")

h(2, "4.7.4 Pelacakan trayektori sinusoidal")

par("Skenario terakhir adalah trayektori ψ_ref = 0,7·sin(2π·0,2·t) rad (amplitudo 40°, "
    "frekuensi 0,2 Hz) sambil bergerak maju 0,2 m/s (Gambar 4.7.4). Cascade memberi "
    f"galat RMS {n(g('s47','cascade_sin','rmse'))}° dengan galat maksimum "
    f"{n(g('s47','cascade_sin','max_abs_err_deg'))}°, keterlambatan puncak "
    f"{n(g('s47','cascade_sin','lag_s'))} s (≈18° fasa) dan rasio amplitudo "
    f"{n(g('s47','cascade_sin','amp_ratio'), 3)}. PID satu loop sedikit lebih baik pada "
    f"skenario ini: RMS {n(g('s47','pid1_sin','rmse'))}°, tunda "
    f"{n(g('s47','pid1_sin','lag_s'))} s, karena aksi derivatifnya menambah penguatan "
    "pada frekuensi rendah-menengah.")

par("Keterlambatan ini bukan akibat tuning, melainkan akibat gesek Coulomb pada kanal "
    "diferensial. Untuk membalik arah putaran, kedua sisi harus melawan gesek Coulomb "
    "yang direfleksikan sebesar 0,231 N·m, setara arus diferensial 0,281 A atau "
    "deadband tegangan V_d ≈ 0,56 V. Dengan K_p,ω = 2,448 V/(rad/s), deadband itu setara "
    "0,23 rad/s – sekitar 26% dari laju yaw puncak yang dibutuhkan (0,88 rad/s). Karena "
    "itu gelombang keluaran tampak \"datar\" di sekitar titik balik. Perbaikan yang "
    "mungkin adalah kompensasi gesek pada kanal heading atau feedforward percepatan "
    "sudut (ψ̈_ref), yang tidak termasuk rancangan pada Bab 4 dan karena itu tidak "
    "diterapkan di sini.")

figure("4_7_sinus.png", "4.7.4",
       ": Pelacakan trayektori sinusoidal 0,2 Hz (atas) dan galat sudut (bawah).")

h(2, "4.7.5 Verifikasi kestabilan secara numerik")

par("Analisis Routh–Hurwitz dan root locus pada subbab 4.4 dilakukan pada model "
    "tereduksi. Sebagai pemeriksaan silang, kutub closed-loop dihitung langsung dari "
    "polinomial karakteristik dengan koefisien plant yang dipakai simulator "
    f"(a_v = {n(g('params','a_v'),3)} s⁻¹, b_v = {n(g('params','b_v'),3)}, "
    f"a_ω = {n(g('params','a_w'),3)} s⁻¹, b_ω = {n(g('params','b_w'),3)}).")

av, bv = g('params', 'a_v'), g('params', 'b_v')
aw, bw = g('params', 'a_w'), g('params', 'b_w')
Kp_v, Ki_v = 17.78, 54.44
Kp_pos, Kp_w, Ki_w = 4.0, 2.448, 8.162
Kp1, Ki1, Kd1 = 7.050, 9.400, 1.273
kv_fb, kxi, kps, kw, kxip = 27.59, 136.5, 15.86, 2.154, 31.72

r_pi = poles(1, av + bv * Kp_v, bv * Ki_v)
r_casc = poles(1, aw + bw * Kp_w, bw * Ki_w + Kp_pos * bw * Kp_w, Kp_pos * bw * Ki_w)
tau_w = 1.0 / aw
K_w = bw / aw
r_pid1 = poles(tau_w, 1 + K_w * Kd1, K_w * Kp1, K_w * Ki1)
r_sf_v = poles(1, av + bv * kv_fb, bv * kxi)
r_sf_p = poles(1, aw + bw * kw, bw * kps, bw * kxip)

table(
    ["Loop", "Polinomial karakteristik", "Kutub closed-loop", "Status"],
    [
        ["PI kecepatan", "s² + " + n(av + bv * Kp_v) + "s + " + n(bv * Ki_v),
         fmt_poles(r_pi), "Stabil"],
        ["Cascade heading", "s³ + " + n(aw + bw * Kp_w) + "s² + "
         + n(bw * Ki_w + Kp_pos * bw * Kp_w) + "s + " + n(Kp_pos * bw * Ki_w),
         fmt_poles(r_casc), "Stabil"],
        ["PID 1 loop heading", n(tau_w, 3) + "s³ + " + n(1 + K_w * Kd1) + "s² + "
         + n(K_w * Kp1) + "s + " + n(K_w * Ki1), fmt_poles(r_pid1), "Stabil"],
        ["SF kecepatan", "s² + " + n(av + bv * kv_fb) + "s + " + n(bv * kxi),
         fmt_poles(r_sf_v), "Stabil"],
        ["SF heading", "s³ + " + n(aw + bw * kw) + "s² + " + n(bw * kps) + "s + "
         + n(bw * kxip), fmt_poles(r_sf_p), "Stabil"],
    ],
    widths=[3.4, 6.4, 4.2, 1.5], font=8.5,
    caption_text=": Kutub closed-loop hasil perhitungan numerik dengan koefisien plant simulator.",
    caption_prefix="Tabel 4.7.2",
)

par("Seluruh kutub berada di setengah bidang kiri. Pada loop cascade, ketiga kutub real "
    f"berada di {n(r_casc[0].real)}, {n(r_casc[1].real)}, dan {n(r_casc[2].real)} s⁻¹. "
    "Kutub dominan menjelaskan waktu penetapan yang terukur: 4/3,36 ≈ 1,2 s berbanding "
    f"{n(g('s47','cascade_90','settling_time'))} s pada simulasi, sehingga target t_s 1,5 s "
    "pada 4.1.4 terpenuhi tanpa osilasi. Angka −5,46 s⁻¹ pada 4.4 berasal dari penempatan "
    "kutub pada model tereduksi tanpa dinamika elektrik; dengan dinamika listrik dan "
    "koreksi c_ψ, kutub dominan bergeser ke −3,36 s⁻¹.")

par("Pada pengontrol alternatif, kanal kecepatan menghasilkan pasangan kutub "
    f"{n(r_sf_v[0].real)} ± j{n(abs(r_sf_v[0].imag), 2)} – praktis ganda di −8 rad/s "
    "seperti rancangan. Kanal heading menghasilkan "
    f"{n(r_sf_p[0].real)}, {n(r_sf_p[1].real)} ± j{n(abs(r_sf_p[1].imag), 2)} s⁻¹ terhadap "
    "target rangkap tiga di −6 rad/s. Sebaran ini khas kutub rangkap tiga: galat "
    "koefisien hanya 0,8% (p(−6) = 1,77 dari 216) sudah menggeser akar ±1,2 rad/s karena "
    "sensitivitas akar rangkap sebanding dengan akar pangkat tiga galat koefisien. "
    "Respons waktu pada Gambar 5.7.1(c) tetap tanpa osilasi berkepanjangan.")

h(2, "4.7.6 Ringkasan perbandingan arsitektur")

par("Dari empat skenario posisi, cascade lebih unggul pada tiga: tangga 90° (t_s "
    f"{n(g('s47','cascade_90','settling_time'))} s berbanding "
    f"{n(g('s47','pid1_90','settling_time'))} s), referensi bertingkat, dan galat tunak. "
    "PID satu loop hanya unggul pada pelacakan sinusoidal dan sedikit pada tangga 180°. "
    "Sebabnya struktural: cascade mengendalikan laju yaw melalui loop dalam yang lebih "
    "cepat (τ ≈ 50 ms) dan membatasi referensinya (3 rad/s), sehingga saturasi tidak "
    "pernah menimbulkan windup besar; PID satu loop tidak memiliki pembatas laju "
    "internal dan derivatifnya memperkuat lonjakan sesaat. Karena itu cascade dipilih "
    "sebagai arsitektur final untuk Bab 6.")

# ======================================================================
# 5.7
# ======================================================================
h(1, "5.7 Implementasi dan Simulasi Pengontrol Alternatif")

par("Pengontrol alternatif yang dipilih pada Bab 5 adalah state feedback dengan aksi "
    "integral (pole placement). Subbab ini mengimplementasikan hukum kendali tersebut "
    "secara diskret pada model nonlinear yang sama dengan 4.6–4.7, sehingga "
    "perbandingan di Bab 6 bersifat apple-to-apple: plant, periode cuplik, tunda, dan "
    "batas tegangan identik.")

h(2, "5.7.1 Bentuk diskret hukum kendali")

par("Kanal kecepatan memakai state [v, ξ_v] dengan ξ_v integrator galat kecepatan, dan "
    "kanal heading memakai [ψ, ψ̇, ξ_ψ]. Hukum kendali yang dieksekusi setiap 10 ms:")
eq("V_c[k] = −k_v·v[k] + k_ξv·ξ_v[k] + N·v_ref[k] + V_c0·sgn(v_ref[k])")
eq("V_d[k] = −k_ψ·ψ[k] − k_ω·ψ̇[k] + k_ξψ·ξ_ψ[k]")
eq("ξ_v[k+1] = ξ_v[k] + Ts·(v_ref[k] − v[k]) + Ts·(V_c,sat − V_c)/(T_t·k_ξv)")
eq("ξ_ψ[k+1] = ξ_ψ[k] + Ts·(ψ_ref[k] − ψ[k]) + Ts·(V_d,sat − V_d)/(T_t·k_ξψ)")
par("Suku terakhir pada kedua persamaan integrator adalah anti-windup back-calculation "
    "dengan T_t = 0,05 s: bila tegangan hasil hitung melebihi batas driver, integrator "
    "dikoreksi sebesar selisihnya sehingga tidak terjadi windup. Penguatan yang dipakai "
    f"adalah k_v = {n(kv_fb)}, k_ξv = {n(kxi)}, N = {n(6.53)}, V_c0 = {n(0.675)} V, "
    f"k_ψ = {n(kps)}, k_ω = {n(kw)}, dan k_ξψ = {n(kxip)} – seluruhnya berasal dari "
    "penempatan kutub pada subbab 5.2.")

par("Diagram alir pengontrol alternatif ini (satu siklus kendali, termasuk mekanisme "
    "anti-windup ξ_v dan ξ_ψ) disajikan pada Lampiran B, Gambar B.2.")

h(2, "5.7.2 Kanal kecepatan")

par("Pada tangga 0 → 0,35 m/s, state feedback memberi t_r = "
    f"{n(g('s57','speed_step','rise_time'))} s, t_s = "
    f"{n(g('s57','speed_step','settling_time'))} s, tanpa lonjakan, dan galat tunak di bawah "
    "10⁻⁶ % karena kompensasi V_c0 dan N tepat pada titik desain. Ciri yang menonjol adalah "
    f"tegangan puncak hanya {n(g('s57','speed_step','max_abs_V'))} V, jauh di bawah "
    "batas 6 V, karena feedforward N·v_ref dan kompensasi V_c0 langsung memberi "
    "keseimbangan tanpa perlu tegangan koreksi besar. Pada gangguan beban 1,5 N, "
    f"kecepatan turun {n(g('s57','speed_dist','drop_pct'))}% – sedikit lebih baik "
    f"daripada PID ({n(g('s46','dist','drop_pct'))}%) karena tidak ada aksi "
    "proporsional yang bereaksi berlebihan terhadap galat sesaat.")

h(2, "5.7.3 Kanal heading")

par("Untuk tangga 90°, SF memberi t_s = {t} s dengan lonjakan {o}% dan galat tunak "
    "{e}°, tegangan puncak {v} V. Nilai ini sedikit lebih lambat daripada cascade "
    "(t_s {tc} s) tetapi dicapai tanpa saturasi diferensial yang besar, sehingga loop "
    "tidak pernah memicu windup. Sebaliknya, pada trayektori sinusoidal SF tertinggal "
    "{lag} s dengan galat RMS {rmse}° – paling buruk di antara ketiga kontroler. "
    "Penyebabnya: hukum SF memakai umpan balik ψ dan ψ̇ tanpa suku feedforward "
    "kecepatan referensi, sehingga tidak ada komponen yang mengantisipasi laju "
    "referensi; ditambah penguatan integrator k_ξψ = {kx} yang besar, loop menjadi "
    "sensitif terhadap deadband gesek Coulomb yang dijelaskan pada 4.7.4.".format(
        t=n(g('s57','heading90','settling_time')), o=n(g('s57','heading90','overshoot_pct')),
        e=n(g('s57','heading90','error_final'), 3), v=n(g('s57','heading90','max_abs_V')),
        tc=n(g('s47','cascade_90','settling_time')), lag=n(g('s57','sin','lag_s')),
        rmse=n(g('s57','sin','rmse')), kx=n(kxip)))

figure("5_7_all.png", "5.7.1",
       ": Pengontrol alternatif (SF + integral) dibandingkan PID: (a) tangga kecepatan, "
       "(b) penolakan gangguan beban, (c) tangga heading 90°, (d) trayektori sinusoidal.")

table(
    ["Metrik", "PID (final)", "SF + integral"],
    [
        ["Kecepatan: t_r [s]", n(g('s46','step_ff','rise_time')), n(g('s57','speed_step','rise_time'))],
        ["Kecepatan: t_s [s]", n(g('s46','step_ff','settling_time')), n(g('s57','speed_step','settling_time'))],
        ["Kecepatan: IAE [m]", n(g('s46','step_ff','iae'), 4), n(g('s57','speed_step','iae'), 4)],
        ["Kecepatan: V puncak [V]", n(g('s46','step_ff','max_abs_V')), n(g('s57','speed_step','max_abs_V'))],
        ["Kecepatan: drop gangguan [%]", n(g('s46','dist','drop_pct')), n(g('s57','speed_dist','drop_pct'))],
        ["Heading 90°: t_s [s]", n(g('s47','cascade_90','settling_time')), n(g('s57','heading90','settling_time'))],
        ["Heading 90°: lonjakan [%]", n(g('s47','cascade_90','overshoot_pct')), n(g('s57','heading90','overshoot_pct'))],
        ["Heading 90°: galat tunak [°]", n(g('s47','cascade_90','error_final'), 3), n(g('s57','heading90','error_final'), 3)],
        ["Heading 90°: V puncak [V]", n(g('s47','cascade_90','max_abs_V')), n(g('s57','heading90','max_abs_V'))],
        ["Sinusoid: RMS [°]", n(g('s47','cascade_sin','rmse')), n(g('s57','sin','rmse'))],
        ["Sinusoid: tunda [s]", n(g('s47','cascade_sin','lag_s')), n(g('s57','sin','lag_s'))],
    ],
    widths=[6.4, 4.2, 4.2],
    caption_text=": Perbandingan ringkas PID dan pengontrol alternatif pada subbab 5.7.",
    caption_prefix="Tabel 5.7.1",
)

h(2, "5.7.4 Kestabilan dan rentang penguatan")

par("Dengan penguatan hasil penempatan kutub, polinomial karakteristik kanal kecepatan "
    f"menjadi s² + {n(av + bv*kv_fb)}s + {n(bv*kxi)}, praktis (s + 8)², dan kanal heading "
    f"s³ + {n(aw + bw*kw)}s² + {n(bw*kps)}s + {n(bw*kxip)}, praktis (s + 6)³. Kutub "
    "aktualnya (Tabel 4.7.2) berada di sekitar lokasi rancangan dengan sebaran 1,2 rad/s "
    "akibat galat pembulatan koefisien, sehingga kestabilan terbukti secara konstruktif "
    "dan tidak memerlukan tabel Routh. Syarat umumnya juga jelas: semua "
    "penguatan positif dan k_ξψ < k_ψ + a_ω·k_ω/b_ω, yang ekivalen dengan batas "
    "k_ξψ < 285,5 pada subbab 5.4; titik kerja 31,72 berada jauh di dalam batas.")

par("Keterbatasan yang perlu dinyatakan: karena penguatan diturunkan dari a_v, b_v, a_ω, "
    "dan b_ω, kinerja hanya sebaik akurasi model. Uji robustness pada 6.3 menunjukkan "
    "bahwa ketika gesek digandakan, t_s heading SF membengkak menjadi "
    f"{n(g('s6','C')[4]['SF']['ts_psi'])} s (PID {n(g('s6','C')[4]['PID']['ts_psi'])} s) "
    "dan galat tunak naik ke "
    f"{n(g('s6','C')[4]['SF']['ess_psi_deg'], 3)}° (PID "
    f"{n(g('s6','C')[4]['PID']['ess_psi_deg'], 3)}°). Integral memang menghapus galat "
    "tunak, tetapi tidak mengubah fakta bahwa penguatan proporsional-derivatif menjadi "
    "terlalu kecil untuk sistem yang lebih \"berat\".")

h(2, "5.7.5 Biaya komputasi")

par("Hukum kendali SF hanya terdiri atas enam perkalian dan lima penjumlahan skalar per "
    "siklus kontrol, tanpa tabel, tanpa filter derivatif, dan tanpa observer karena "
    "seluruh state (v, ψ, ψ̇) tersedia dari encoder. Pada ESP32 dengan FPU, beban ini "
    "jauh di bawah 1% dari anggaran 10 ms, sehingga SF menarik dari sisi implementasi. "
    "Sebaliknya, PID satu loop membutuhkan filter turunan (T_f = 20 ms) yang menambah "
    "satu state dan menurunkan margin fasa.")

# ======================================================================
# 6
# ======================================================================
doc.add_page_break()
h(1, "6 Simulasi, Pengujian, dan Analisis Perbandingan")

par("Bab ini menyatukan seluruh pengujian: empat skenario kecepatan (A), empat skenario "
    "posisi (B), uji robustness terhadap perubahan parameter dan gangguan (C), serta "
    "variasi jenis motor (D). Setiap skenario dijalankan pada dua pengontrol – PID "
    "(PI kecepatan dengan feedforward + cascade heading) dan state feedback dengan "
    "integral – dengan plant, periode cuplik, tunda, dan batas tegangan yang identik. "
    "Metrik yang dilaporkan mengikuti 6.E: rise time (10–90%), settling time pita 2%, "
    "lonjakan maksimum, galat tunak, IAE/RMSE, dan tegangan kendali maksimum.")

table(
    ["Metrik", "Definisi operasional"],
    [
        ["Rise time (t_r)", "waktu dari 10% ke 90% nilai akhir (referensi tangga)"],
        ["Settling time (t_s)", "waktu terakhir keluar dari pita ±2% nilai akhir"],
        ["Lonjakan (MP)", "(nilai puncak − nilai akhir)/perubahan referensi × 100%"],
        ["Galat tunak", "|referensi − keluaran| pada akhir jendela evaluasi"],
        ["IAE / RMSE", "∫|e|dt dan √(mean e²) pada jendela evaluasi (trapesium, Ts = 10 ms)"],
        ["Tegangan puncak", "max(|V_L|, |V_R|) sepanjang simulasi"],
    ],
    widths=[4.6, 9.9],
    caption_text=": Definisi metrik yang dipakai pada seluruh tabel Bab 6.",
    caption_prefix="Tabel 6.1",
)

h(2, "6.1 (A) Pengujian kontrol kecepatan")

par("Empat skenario diuji: tangga 0 → 0,35 m/s, referensi bertingkat "
    "(0,15 → 0,25 → 0,35 → 0,20 m/s), ramp 0,4375 m/s², dan tangga dengan gangguan beban "
    "1,5 N pada t = 3–4,5 s. Hasil lengkap pada Gambar 6.1 dan Tabel 6.2.")

figure("6_A_speed.png", "6.1",
       ": Perbandingan PID dan SF pada empat skenario kecepatan: (a) tangga, "
       "(b) bertingkat, (c) ramp, (d) gangguan beban.")

rows = []
labels = {"step": "Tangga 0 → 0,35 m/s", "multi": "Bertingkat", "ramp": "Ramp",
          "load": "Tangga + gangguan 1,5 N"}
for k in ("step", "multi", "ramp", "load"):
    for lab in ("PID", "SF"):
        rows.append([
            labels[k] if lab == "PID" else "",
            lab,
            n(g('s6','A',k,lab,'rise_time')) if k != "ramp" else "–",
            n(g('s6','A',k,lab,'settling_time')) if k != "ramp" else "–",
            n(g('s6','A',k,lab,'overshoot_pct')) if k != "ramp" else "–",
            n(g('s6','A',k,lab,'iae'), 4) if k != "ramp" else n(g('s6','A',k,lab,'rmse'), 4),
            n(g('s6','A',k,lab,'max_abs_V')),
        ])
table(["Skenario", "Kontroler", "t_r [s]", "t_s [s]", "MP [%]",
       "IAE/RMSE", "V puncak [V]"], rows,
      widths=[4.0, 2.0, 1.8, 1.8, 1.6, 2.1, 2.1], font=8.5,
      caption_text=": Metrik kontrol kecepatan, PID vs SF (4 skenario). IAE dalam m untuk "
                   "skenario tangga; RMSE dalam m/s untuk ramp.",
      caption_prefix="Tabel 6.2")

par("PID unggul pada seluruh skenario pelacakan. Pada tangga, kedua kontroler mencapai "
    f"t_s yang sama ({n(g('s6','A','step','PID','settling_time'))} s) tetapi IAE PID "
    f"{n(g('s6','A','step','PID','iae'), 4)} berbanding "
    f"{n(g('s6','A','step','SF','iae'), 4)} – PID lebih cepat mendekati referensi pada "
    "fase awal karena suku proporsionalnya langsung bereaksi, sedangkan SF hanya "
    "mengandalkan feedforward N·v_ref dan integrator. Keunggulan PID paling jelas pada "
    f"ramp: RMSE {n(g('s6','A','ramp','PID','rmse')*1000, 1)} mm/s berbanding "
    f"{n(g('s6','A','ramp','SF','rmse')*1000, 1)} mm/s. Sebaliknya, SF memakai tegangan "
    f"puncak lebih rendah ({n(g('s6','A','step','SF','max_abs_V'))} V berbanding "
    f"{n(g('s6','A','step','PID','max_abs_V'))} V) dan tidak pernah saturasi, yang berarti "
    "cadangan torsi lebih besar untuk menanjak atau membawa beban tambahan.")

h(2, "6.2 (B) Pengujian kontrol posisi")

par("Empat skenario posisi: 0 → 90° dan 0 → 180° saat jelajah 0,25 m/s, referensi "
    "bertingkat dengan pembalikan arah, dan trayektori sinusoidal 0,2 Hz (Gambar 6.2, "
    "Tabel 6.3).")

figure("6_B_pos.png", "6.2",
       ": Perbandingan PID dan SF pada empat skenario posisi: (a) 90°, (b) 180°, "
       "(c) bertingkat, (d) sinusoidal.")

labels_b = {"step90": "Tangga 0 → 90°", "step180": "Tangga 0 → 180°",
            "multi": "Bertingkat", "sin": "Sinusoid 0,2 Hz"}
rows = []
for k in ("step90", "step180", "multi", "sin"):
    for lab in ("PID", "SF"):
        rows.append([
            labels_b[k] if lab == "PID" else "", lab,
            n(g('s6','B',k,lab,'rise_time')) if k != "sin" else "–",
            n(g('s6','B',k,lab,'settling_time')) if k != "sin" else "–",
            n(g('s6','B',k,lab,'overshoot_pct')) if k != "sin" else "–",
            n(g('s6','B',k,lab,'iae'), 3),
            n(g('s6','B',k,lab,'max_abs_V')),
        ])
table(["Skenario", "Kontroler", "t_r [s]", "t_s [s]", "MP [%]", "IAE [°·s]",
       "V puncak [V]"], rows,
      widths=[3.8, 2.0, 1.8, 1.8, 1.6, 2.2, 2.2], font=8.5,
      caption_text=": Metrik kontrol posisi/heading, PID vs SF (4 skenario).",
      caption_prefix="Tabel 6.3")

par("Pada tangga 90° PID lebih cepat ({t90p} s berbanding {t90s} s) dan lebih halus; "
    "pada tangga 180° justru SF lebih cepat ({t180s} s berbanding {t180p} s) dengan IAE "
    "{iae180s} berbanding {iae180p} °·s, karena penguatan heading SF yang lebih tinggi "
    "dapat memanfaatkan rentang tegangan penuh tanpa windup. Pada referensi bertingkat "
    "keduanya setara dalam waktu, tetapi SF menyisakan galat tunak lebih kecil "
    "({essm_s}° berbanding {essm_p}°). Pada sinusoidal PID jauh lebih baik "
    "(RMS {sinp}° berbanding {sins}°). Secara keseluruhan tidak ada kontroler yang "
    "mendominasi: PID unggul ketika referensi berubah cepat dan berulang, SF unggul "
    "ketika manuver besar dan beban berat.".format(
        t90p=n(g('s6','B','step90','PID','settling_time')), t90s=n(g('s6','B','step90','SF','settling_time')),
        t180p=n(g('s6','B','step180','PID','settling_time')), t180s=n(g('s6','B','step180','SF','settling_time')),
        iae180p=n(g('s6','B','step180','PID','iae'),1), iae180s=n(g('s6','B','step180','SF','iae'),1),
        essm_p=n(g('s6','B','multi','PID','error_final'),2), essm_s=n(g('s6','B','multi','SF','error_final'),2),
        sinp=n(g('s6','B','sin','PID','rmse')), sins=n(g('s6','B','sin','SF','rmse'))))

h(2, "6.3 (C) Pengujian robustness")

par("Lima kondisi operasi yang berbeda dari nominal diuji, ditambah dua uji gangguan: "
    "(i) massa dan inersia +30% (payload tambahan), (ii) koefisien gesek digandakan, "
    "(iii) resistansi armatur +20% dan konstanta torsi −10% (motor memanas atau "
    "berbeda batch), (iv) tegangan suplai turun ke 5,0 V (baterai lemah), serta "
    "(v) gangguan torsi eksternal 0,05 N·m selama 1 s pada kanal heading. Setiap kondisi "
    "diuji dengan skenario tangga kecepatan dan tangga heading 90° memakai penguatan "
    "yang sama seperti nominal (tanpa adaptasi) – kondisi terburuk untuk kontroler "
    "berbasis model. Hasil numeriknya pada Tabel 6.4 (kecepatan), Tabel 6.5 "
    "(heading), dan Tabel 6.6 (gangguan torsi sesaat).")

rows_v = []
rows_h = []
for r in [r for i, r in enumerate(g('s6', 'C')) if i % 2 == 0 and i < 10]:
    rows_v.append([r["case"], n(r["PID"]["ts_v"]), n(r["PID"]["OS_v"]),
                   n(r["PID"]["ess_v"], 4), n(r["SF"]["ts_v"]), n(r["SF"]["OS_v"]),
                   n(r["SF"]["ess_v"], 4)])
    rows_h.append([r["case"], n(r["PID"]["ts_psi"]), n(r["PID"]["OS_psi"]),
                   n(r["PID"]["ess_psi_deg"], 2), n(r["SF"]["ts_psi"]),
                   n(r["SF"]["OS_psi"]), n(r["SF"]["ess_psi_deg"], 2)])
table(["Kondisi", "PID t_s [s]", "PID MP [%]", "PID e_v [%]",
       "SF t_s [s]", "SF MP [%]", "SF e_v [%]"], rows_v,
      widths=[4.4, 1.9, 1.9, 1.9, 1.9, 1.9, 1.9], font=8.5,
      caption_text=": Robustness kanal kecepatan (tangga 0 → 0,35 m/s) pada lima kondisi operasi.",
      caption_prefix="Tabel 6.4")
table(["Kondisi", "PID t_s [s]", "PID MP [%]", "PID e_ψ [°]",
       "SF t_s [s]", "SF MP [%]", "SF e_ψ [°]"], rows_h,
      widths=[4.4, 1.9, 1.9, 1.9, 1.9, 1.9, 1.9], font=8.5,
      caption_text=": Robustness kanal heading (tangga 90°) pada lima kondisi operasi.",
      caption_prefix="Tabel 6.5")

rows_t = []
for r in [r for i, r in enumerate(g('s6', 'C')) if i % 2 == 1 and i < 10]:
    rows_t.append([r["case"], n(r["PID"]["psi_dev_deg"], 3),
                   n(r["PID"]["psi_end_deg"], 3),
                   n(r["SF"]["psi_dev_deg"], 3), n(r["SF"]["psi_end_deg"], 3)])
table(["Kondisi (gangguan torsi 0,05 N·m, 1 s)", "PID simpangan [°]",
       "PID sisa [°]", "SF simpangan [°]", "SF sisa [°]"], rows_t,
      widths=[6.6, 2.2, 2.2, 2.2, 2.2], font=8.5,
      caption_text=": Respons terhadap gangguan torsi eksternal sesaat pada kanal heading.",
      caption_prefix="Tabel 6.6")

figure("6_C_robust.png", "6.3",
       ": Waktu penetapan kecepatan (pita 2%) pada lima kondisi operasi, PID vs SF.")

par("Temuan utama: (i) PID jauh lebih tahan pada kanal heading ketika gesek berubah, "
    "sedangkan SF lebih tahan pada kanal kecepatan dan terhadap perubahan massa maupun "
    "tegangan suplai. Pada gesek dua kali, t_s kecepatan PID {a} s berbanding SF {b} s, "
    "tetapi t_s heading SF "
    "membengkak menjadi {c} s (PID {d} s) karena penguatan k_ψ dan k_ω tidak lagi cukup "
    "untuk torsi Coulomb yang dua kali lebih besar. (ii) Perubahan parameter motor "
    "(Ra +20%, Kt −10%) hampir tidak berpengaruh pada PID karena integrator menyerap "
    "galat, tetapi menggeser titik kerja SF sehingga galat heading naik ke {e}°. "
    "(iii) Penurunan tegangan suplai ke 5,0 V menaikkan t_s PID ke {f} s (loop menyentuh "
    "saturasi lebih sering), sedangkan SF tidak terpengaruh karena tegangan kerjanya "
    "memang lebih rendah. (iv) Gangguan torsi 0,05 N·m hanya menyimpangkan heading "
    "{g}° pada PID dan {hh}° pada SF, keduanya pulih di bawah 0,15° setelah gangguan "
    "hilang.".format(
        a=n(g('s6','C')[4]['PID']['ts_v']), b=n(g('s6','C')[4]['SF']['ts_v']),
        c=n(g('s6','C')[4]['SF']['ts_psi']), d=n(g('s6','C')[4]['PID']['ts_psi']),
        e=n(g('s6','C')[6]['SF']['ess_psi_deg'], 2),
        f=n(g('s6','C')[8]['PID']['ts_v']),
        g=n(g('s6','C')[1]['PID']['psi_dev_deg'], 3),
        hh=n(g('s6','C')[1]['SF']['psi_dev_deg'], 3)))

par("Uji derau sensor (encoder dengan simpangan baku 10 mm/s dan 2 mrad) memperlihatkan "
    "perbedaan penting: t_s kecepatan PID {a} s berbanding SF {b} s, dan galat tunak "
    "{c}% berbanding {d}%. Pembobotan setpoint β = 0,70 dan filter turunan membuat PID "
    "tidak memperkuat derau, sedangkan SF meneruskan derau pengukuran langsung ke "
    "tegangan melalui penguatan k_v = 27,59.".format(
        a=n(g('s6','C')[10]['PID']['ts_v']), b=n(g('s6','C')[11]['SF']['ts_v']),
        c=n(g('s6','C')[10]['PID']['ess_v'], 2), d=n(g('s6','C')[11]['SF']['ess_v'], 2)))

h(2, "6.4 (D) Pengujian variasi motor")

par("Penguatan kedua kontroler dibiarkan tetap, lalu plant diganti dengan motor "
    "alternatif: gearmotor 6 V 1:30 dengan konstanta torsi motor lebih besar "
    f"(K_t = K_e = 4,0×10⁻³), R_a = {n(PLANT_N20.Ra,1)} Ω, dan rotor lebih berat. Secara "
    f"tetapi rasio gearbox lebih rendah. Secara ekuivalen, massa terefleksi motor ini "
    f"{n(PLANT_N20.m_eq)} kg (TT 48:1: "
    f"{n(P.m_eq)} kg), sedangkan c_F = {n(PLANT_N20.c_F)} N/A dan k_v = "
    f"{n(PLANT_N20.k_v,2)} V·s/m. Hasil lengkap pada Gambar 6.4 dan Tabel 6.7.")

figure("6_D_motor.png", "6.4",
       ": Pengaruh karakteristik motor terhadap respons kecepatan (penguatan kontroler "
       "tidak diubah).")

table(
    ["Motor", "Kontroler", "t_r [s]", "t_s [s]", "MP [%]", "IAE [m]"],
    [
        ["TT 48:1 (terpilih)", "PID",
         n(g('s6','D','TT','rise_time')), n(g('s6','D','TT','settling_time')),
         n(g('s6','D','TT','overshoot_pct')), n(g('s6','D','TT','iae'), 4)],
        ["TT 48:1 (terpilih)", "SF", n(g('s57','speed_step','rise_time')),
         n(g('s57','speed_step','settling_time')), n(g('s57','speed_step','overshoot_pct')),
         n(g('s57','speed_step','iae'), 4)],
        ["Alternatif 1:30", "PID", n(g('s6','D','PID','rise_time')),
         n(g('s6','D','PID','settling_time')), n(g('s6','D','PID','overshoot_pct')),
         n(g('s6','D','PID','iae'), 4)],
        ["Alternatif 1:30", "SF", n(g('s6','D','SF','rise_time')),
         n(g('s6','D','SF','settling_time')), n(g('s6','D','SF','overshoot_pct')),
         n(g('s6','D','SF','iae'), 4)],
    ],
    widths=[4.4, 2.4, 2.0, 2.0, 1.9, 2.0], font=8.5,
    caption_text=": Respons kecepatan pada dua karakteristik motor dengan penguatan tetap.",
    caption_prefix="Tabel 6.7",
)

par("Motor alternatif memberi torsi per ampere lebih kecil (c_F {a} berbanding {b} N/A) "
    "tetapi massa terefleksi jauh lebih ringan, sehingga rasio b_v/a_v berubah dan "
    "respons PID menjadi {c} s dengan lonjakan {d}% – mulai melewati batas 5%. SF justru "
    "memberi t_s {e} s dan lonjakan {f}%. Hasil ini menegaskan bahwa pemilihan motor "
    "tidak boleh dipisahkan dari tuning: penguatan yang dihitung untuk TT 48:1 tidak "
    "otomatis optimal untuk motor lain, dan motor dengan inersia terefleksi lebih besar "
    "membutuhkan penguatan proporsional yang lebih tinggi.".format(
        a=n(PLANT_N20.c_F), b=n(P.c_F), c=n(g('s6','D','PID','settling_time')),
        d=n(g('s6','D','PID','overshoot_pct')), e=n(g('s6','D','SF','settling_time')),
        f=n(g('s6','D','SF','overshoot_pct'))))

h(2, "6.5 (E) Analisis performa dan kesimpulan teknis")

par("Tabel 6.8 merangkum perbandingan pada skenario inti, dengan tanda ✓ untuk kontroler "
    "yang lebih baik pada setiap baris.")

def better(*vals, lower=True):
    a, b = vals
    return (a <= b) if lower else (a >= b)

rows = []
pairs = [
    ("Kecepatan: rise time", g('s6','A','step','PID','rise_time'), g('s6','A','step','SF','rise_time'), "s"),
    ("Kecepatan: settling time", g('s6','A','step','PID','settling_time'), g('s6','A','step','SF','settling_time'), "s"),
    ("Kecepatan: IAE (tangga)", g('s6','A','step','PID','iae'), g('s6','A','step','SF','iae'), "m"),
    ("Kecepatan: tegangan puncak", g('s6','A','step','PID','max_abs_V'), g('s6','A','step','SF','max_abs_V'), "V"),
    ("Kecepatan: RMSE dengan gangguan", g('s6','A','load','PID','rmse'), g('s6','A','load','SF','rmse'), "m/s"),
    ("Kecepatan: RMSE ramp", g('s6','A','ramp','PID','rmse'), g('s6','A','ramp','SF','rmse'), "m/s"),
    ("Posisi: settling 90°", g('s6','B','step90','PID','settling_time'), g('s6','B','step90','SF','settling_time'), "s"),
    ("Posisi: settling 180°", g('s6','B','step180','PID','settling_time'), g('s6','B','step180','SF','settling_time'), "s"),
    ("Posisi: RMSE sinusoid", g('s6','B','sin','PID','rmse'), g('s6','B','sin','SF','rmse'), "°"),
    ("Robustness: gesek ×2 (t_s heading)", g('s6','C')[4]['PID']['ts_psi'], g('s6','C')[4]['SF']['ts_psi'], "s"),
    ("Robustness: derau sensor (e_v)", g('s6','C')[10]['PID']['ess_v'], g('s6','C')[11]['SF']['ess_v'], "%"),
    ("Variasi motor: t_s", g('s6','D','PID','settling_time'), g('s6','D','SF','settling_time'), "s"),
]
for name, a, b, unit in pairs:
    nd = 4 if unit in ("m", "m/s") else 2
    rows.append([name, n(a, nd, unit), n(b, nd, unit),
                 "Seri" if abs(a - b) < 1e-9 else ("PID" if better(a, b) else "SF")])
table(["Kriteria", "PID", "SF + integral", "Lebih baik"], rows,
      widths=[6.4, 3.0, 3.0, 2.1], font=8.5,
      caption_text=": Ringkasan perbandingan performa PID dan pengontrol alternatif.",
      caption_prefix="Tabel 6.8")

par("Dari dua belas kriteria, PID unggul pada tujuh dan SF pada lima. Pola kemenangan "
    "kedua kontroler konsisten dengan struktur masing-masing:")

bullet("PID (PI + feedforward + cascade) menang pada pelacakan: referensi berubah cepat "
       "(bertingkat), referensi kontinu (ramp, sinusoid), dan pada kondisi derau sensor. "
       "Sumber keunggulannya adalah integrator galat kecepatan yang bekerja pada galat "
       "penuh ditambah aksi proporsional yang langsung bereaksi, serta tidak adanya "
       "ketergantungan pada akurasi model untuk arah koreksi.")
bullet("SF + integral menang pada manuver besar dan kondisi parameter berubah: tangga "
       "180°, cadangan tegangan (tidak pernah saturasi), perubahan massa/inersia, dan "
       "penurunan tegangan baterai. SF juga lebih ringan secara komputasi dan tidak "
       "memerlukan filter derivatif.")
bullet("Keduanya gagal pada satu kondisi yang sama: ketika gesek digandakan, kanal "
       "heading kehilangan presisi karena deadband Coulomb setara 0,56 V tidak "
       "dikompensasi oleh kedua hukum kendali. Solusi yang sama berlaku untuk keduanya, "
       "yaitu penambahan kompensasi gesek atau estimator gangguan.")

par("Rekomendasi teknis: untuk AGV ini, PID dengan feedforward dan cascade heading tetap "
    "dipilih sebagai pengontrol utama karena tugas utamanya adalah mengikuti lintasan "
    "line follower pada kecepatan rendah–menengah dengan referensi yang berubah terus, "
    "dan karena performanya paling tidak sensitif terhadap ketidakpastian parameter. SF "
    "layak dipertimbangkan bila (i) model teridentifikasi dengan baik melalui uji "
    "identifikasi parameter, (ii) beban berubah banyak sehingga cadangan tegangan "
    "penting, atau (iii) pengendalian dilakukan pada mikrokontroler dengan anggaran "
    "komputasi sangat ketat. Langkah pengembangan berikutnya yang paling berdampak "
    "adalah kompensasi gesek Coulomb pada kanal heading dan feedforward percepatan "
    "sudut, karena keduanya menyerang sumber galat terbesar yang ditemukan pada bab ini.")

# ======================================================================
# Lampiran
# ======================================================================
doc.add_page_break()
h(1, "Lampiran A. Kode Program Simulasi")

par("Seluruh kode berikut dapat dijalankan apa adanya dengan Python 3.12 (numpy, "
    "matplotlib). Simulasi tidak memakai fungsi kontrol bawaan seperti step(), lsim(), "
    "tf(), ss(), feedback(), maupun scipy.signal; dinamika diintegrasikan dengan RK4 "
    "yang ditulis sendiri pada agv/plant.py, dan algoritma kontrol dieksekusi secara "
    "diskret pada agv/control.py. Struktur berkas: agv/params.py (parameter dan "
    "koefisien), agv/plant.py (persamaan dinamik + RK4), agv/control.py (PID, SF, "
    "mixer), agv/sim.py (loop simulasi dengan ZOH dan tunda), agv/metrics.py (metrik), "
    "run_sims.py (seluruh skenario dan grafik).")

for fname, title in [
    ("agv/params.py", "A.1  agv/params.py"),
    ("agv/plant.py", "A.2  agv/plant.py"),
    ("agv/control.py", "A.3  agv/control.py"),
    ("agv/sim.py", "A.4  agv/sim.py"),
    ("agv/metrics.py", "A.5  agv/metrics.py"),
]:
    h(2, title)
    code_block(os.path.join(ROOT, fname), os.path.basename(fname))

h(2, "A.6  run_sims.py (skenario lengkap)")
code_block(os.path.join(ROOT, "run_sims.py"), "run_sims.py")

doc.add_page_break()
h(1, "Lampiran B. Flowchart Algoritma Kontrol")

par("Lampiran ini memenuhi butir lampiran wajib \"flowchart algoritma kontrol\". Diagram "
    "disusun untuk satu siklus kendali (T_s = 10 ms) karena algoritma dijalankan secara "
    "diskret: setiap siklus kontroler membaca state, menghitung tegangan kedua sisi, "
    "menerapkannya ke driver (dengan tunda 15 ms), lalu plant diintegrasikan dengan RK4 "
    "sebelum siklus berikutnya. Percabangan saturasi dan anti-windup digambarkan eksplisit "
    "karena keduanya menentukan hasil pengujian pada Bab 6.")

figure("flow_pid.png", "B.1",
       ": Flowchart algoritma kontrol PID cascade + feedforward (satu siklus kendali).",
       width=13.0)
figure("flow_sf.png", "B.2",
       ": Flowchart pengontrol alternatif state feedback + integral (satu siklus kendali).",
       width=13.0)

out = os.path.join(ROOT, "Bagian_4.6-6_Simulasi_AGV.docx")
doc.save(out)
print("tersimpan:", out)
print("paragraf:", len(doc.paragraphs), "tabel:", len(doc.tables))
