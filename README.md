# S3 Air

USB DAC + DSP berbasis **ESP32-S3 N16R8** dan DAC I2S **PCM5102**.
Komputer membaca alat ini sebagai soundcard USB **24-bit / 48 kHz**;
semua pengolahan suara dikerjakan task DSP khusus (FreeRTOS, Core 1)
dan diatur dari panel web di jaringan rumah (`http://mbahdac.local`)
atau lewat perintah serial di port COM.

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

## Cara pakai

1. `git clone` repo ini (sekali saja), berikutnya cukup `git pull`.
2. Di folder `S3Air_USB`, salin `wifi_config.example.h` menjadi
   `wifi_config.h`, lalu isi nama & password WiFi rumah di dalamnya.
   File itu tidak ikut ter-commit, jadi password tidak akan bocor.
3. Buka `S3Air_USB.ino` di Arduino IDE, samakan setting di atas, upload.
4. Buka `http://mbahdac.local` dari HP / PC di WiFi yang sama.

Kredensial juga bisa diganti belakangan dari tab Sistem di panel web
(tersimpan di NVS alat dan menimpa isi `wifi_config.h`).

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

Versi firmware saat ini: **v3.1 S3 Air** — panel web gaya X Air (simple, satu fokus per tab). (Proyek ini sebelumnya bernama MbahDAC;
identitas USB & alamat panel masih `MbahDAC USB` / `mbahdac.local`).
