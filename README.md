# S3 Air

USB DAC + DSP berbasis **ESP32-S3 N16R8** dan DAC I2S **PCM5102**.
Komputer membaca alat ini sebagai soundcard USB **16-bit / 48 kHz**;
semua pengolahan suara dikerjakan task DSP khusus (FreeRTOS, Core 1)
dan diatur dari aplikasi Android **S3 KONSOL** — lewat WiFi di rumah
(API `mbahdac.local`) atau lewat **Bluetooth** di lapangan (sejak v1.1.0). Di alamat yang sama ada halaman darurat kecil
(status alat + ganti WiFi) untuk keadaan darurat, dan kontrol serial
tetap tersedia di port COM.

## Fitur DSP

- PEQ 10 band (Peak / Low Shelf / High Shelf / Notch / Band Pass) + Dynamic EQ per band
- GEQ 31 band 1/3 oktaf, Low Cut (HPF) 20-200 Hz slope 6/12/18/24 dB
- Noise gate, compressor (+ meter gain reduction), limiter brickwall + penghitung clip
- Trim L/R, balik fase, preamp, delay L/R 0-20 ms, stereo width, balance, cek mono
- Crossfeed, loudness Fletcher-Munson, auto-headroom, kunci volume maksimum, Dim -20 dB
- RTA spektrum 31 band (FFT 1024), meter Peak + RMS
- 5 preset pabrik (Flat, Speaker DIY, Vokal, Bass, Malam) + 10 slot preset user (NVS)
- Banding A/B, ekspor / impor setting JSON
- Semua fader meluncur halus (glide per-sampel, tanpa kresek)

## Mode DLMS (manajemen loudspeaker 2 arah)

- 2 input (A / B): aktif, gain, delay 0-100 ms
- 2 kanal universal (OUT 1 / OUT 2): sumber A / B / A+B, crossover
  HPF + LPF (Linkwitz-Riley / Butterworth sampai 48 dB/okt), PEQ 3 band,
  gain, delay 0-80 ms, peak limiter per kanal, balik fase
- Mode DLMS mati = bypass total ke rantai reguler; master tetap jalan

## Wiring (sesuai rakitan)

| Sinyal PCM5102 | GPIO ESP32-S3 |
| --- | --- |
| BCK | 42 |
| LRCK / WS | 21 |
| DIN | 2 |
| SCK | GND (mode PLL, tanpa MCLK) |

USB memakai port native ESP32-S3 (GPIO 19/20).

## Setting Arduino IDE (WAJIB, persis seperti ini)

- Board: **ESP32S3 Dev Module**
- USB Mode: **USB-OTG (TinyUSB)**
- USB CDC On Boot: **Disabled** ← penting; kalau Enabled, Windows menolak
  alat ini (Code 10) karena gabungan CDC + audio tidak didukung driver Windows
- PSRAM: **OPI PSRAM**
- Flash Size: **16 MB**
- Partition Scheme: **Huge APP (3MB No OTA/1MB SPIFFS)** — wajib sejak v1.1.0
- Library: **NimBLE-Arduino** by h2zero (pasang via Library Manager)

## Cara pakai

1. `git clone` repo ini (sekali saja), berikutnya cukup `git pull`.
2. Di folder `S3Air_USB`, salin `wifi_config.example.h` menjadi
   `wifi_config.h`, lalu isi nama & password WiFi rumah di dalamnya.
   File itu tidak ikut ter-commit, jadi password tidak akan bocor.
3. Buka `S3Air_USB.ino` di Arduino IDE, samakan setting di atas, upload.
4. Buka aplikasi S3 KONSOL di HP (WiFi yang sama) dan sambungkan ke alat; `http://mbahdac.local` kini menampilkan halaman darurat.

Panduan lengkap (API, serial, troubleshooting): [PANDUAN.md](PANDUAN.md).

Kredensial juga bisa diganti belakangan dari aplikasi S3 KONSOL
(Pengaturan -> Jaringan WiFi alat) atau dari halaman darurat —
tersimpan di NVS alat dan menimpa isi `wifi_config.h`.

## Kontrol serial (Serial0, 115200 baud)

```
set vol=0.8 b0_g=3 g12=4 loud=1
preset malam
upreset 2
save 2
state
```

## Alur kerja repo ini

- Rilis baru dari mbah → `git pull` di PC → upload via Arduino IDE.
- Edit lokal di PC → `git add -A && git commit -m "pesan" && git push`.

Versi firmware saat ini: **v1.1.0 S3 Air — Bluetooth** (2026-11-14).
Server BLE (NimBLE) mencerminkan seluruh permukaan kontrol untuk mode
lapangan: perintah teks (state/set/preset/save/wifi) + meter & spektrum
biner 55 byte / 50 ms; bonding wajib dengan jendela pairing 60 detik
(setelah boot / tombol BOOT). Fondasinya tetap rilis final WiFi v1.0.0
(= v5.2.5: halaman darurat, peak limiter DLMS, delay 0-80 ms, meter+RTA
gabungan, persistensi penuh, keluaran 24-bit berdither TPDF, transisi
bebas klik, lapisan gagal-diam). Syarat build: library NimBLE-Arduino +
Partition Scheme Huge APP. (Proyek ini sebelumnya bernama MbahDAC;
identitas USB & alamat masih `MbahDAC USB` / `mbahdac.local`).
