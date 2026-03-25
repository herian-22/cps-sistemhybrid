# CPS Hybrid System Simulator

Simulator berbasis **PySide6** untuk memodelkan hybrid automaton robot berkaki empat. Aplikasi ini menampilkan visualisasi 3D, sinyal sensor kontinu→diskret, serta kontrol mode manual/otomatis dengan pengelolaan rintangan kustom.

## Fitur
- **Live 3D viewport** dengan grid/axis/label yang dapat diaktifkan/nonaktifkan dan kontrol kamera (drag untuk orbit, scroll untuk zoom).
- **Panel analitik**: diagram state, plot sudut servo real-time, dan panel sinyal (raw, filtered, quantized).
- **Mode kontrol**: MANUAL (atur jarak sensor via slider) dan AUTO (robot maju dan menghindar otomatis).
- **Zeno mode** untuk mensimulasikan chattering dengan ambang tanpa hysteresis.
- **Pengaturan gait & kinematika**: panjang femur/tibia, amplitudo dan kecepatan langkah, toggle inverse kinematics.
- **Manajer rintangan kustom**: tambah/hapus obstacle statis, dengan highlight saat terdeteksi (jarak ≤25 cm di depan robot).
- **Reset simulasi & kamera** kapan saja.

## Prasyarat
- Python **3.10+** disarankan.
- Sistem dengan dukungan Qt (PySide6).

## Instalasi
```bash
# Opsional: buat virtualenv
python -m venv .venv
source .venv/bin/activate  # Windows: .venv\\Scripts\\activate

# Pasang dependensi
pip install -r requirment.txt
```

## Menjalankan Aplikasi
```bash
python main.py
```

### Kontrol Utama
- **START/PAUSE**: memulai atau menghentikan simulasi (timer tetap berjalan untuk rendering).
- **AUTO/MANUAL**:
  - AUTO: robot maju otomatis dan beralih ke EVASIVE saat obstacle dekat.
  - MANUAL: slider jarak sensor aktif untuk memberi input manual.
- **ZENO MODE**: menonaktifkan hysteresis guard dan mengatur slider jarak ke 25 cm (aktifkan/ matikan dengan tombol ⚡).
- **RESET**: mengatur ulang state hybrid (mode WALKING, jarak 100 cm, transisi 0).
- **Reset Camera**: mengembalikan sudut orbit & zoom default.

### Pengaturan & Tab
- **Sensor Distance (Manual)**: slider 5–100 cm, aktif hanya pada MANUAL.
- **View options**: toggle grid, axis gizmo, label.
- **Servo gauges**: indikator sudut keempat servo.
- **Analytics strip**: diagram state, plot servo, dan panel sinyal.
- **Global Configuration tab**:
  - **Inverse Kinematics**: aktif/nonaktif dan atur panjang femur/tibia.
  - **Gait Dynamics**: atur amplitude (°) dan kecepatan langkah (rad/s).
  - **Custom Obstacle Manager**: isi X/Z/Width/Height lalu klik **ADD**; pilih item dan **REMOVE** untuk menghapus.

## Struktur Proyek
```
main.py                   # Entry point aplikasi Qt
core/logic.py             # Logika hybrid automaton & fisika sederhana
ui/main_window.py         # Layout jendela utama dan wiring kontrol
ui/widgets/robot_3d.py    # Renderer 3D robot & obstacle
ui/widgets/analytics.py   # Diagram state, plot servo, panel sinyal
ui/widgets/gauges.py      # Widget gauge servo & jarak
```

## Pemecahan Masalah
- Pastikan PySide6 terpasang sesuai versi di `requirment.txt` (6.11.0).
- Jika GUI tidak muncul, jalankan dari terminal dan periksa error Qt yang tampil.
- Lingkungan virtual dianjurkan untuk menghindari konflik dependensi Qt.

## Lisensi
Belum ditentukan di repository ini.
