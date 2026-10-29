# Panduan S3 Air

USB DAC + DSP berbasis ESP32-S3 (N16R8) dan DAC I2S PCM5102.
Windows membaca alat ini sebagai soundcard USB **16-bit / 48 kHz** bernama
`MbahDAC USB`. Semua pengolahan suara dikerjakan task DSP khusus (FreeRTOS,
Core 1); USB dan DSP bertukar sampel lewat ring buffer.

## 1. Setting Arduino IDE (WAJIB)

| Menu | Nilai |
| --- | --- |
| Board | ESP32S3 Dev Module |
| USB Mode | USB-OTG (TinyUSB) |
| USB CDC On Boot | **Disabled** |
| PSRAM | OPI PSRAM |
| Flash Size | 16 MB (N16R8) |
| Core esp32 | 3.3.12 |

`USB CDC On Boot` harus Disabled: gabungan USB CDC + audio membuat driver
Windows gagal start (Code 10). Debug & kontrol serial lewat Serial0
(port COM satunya).

## 2. Wiring

| Sinyal PCM5102 | GPIO ESP32-S3 |
| --- | --- |
| BCK | 42 |
| LRCK / WS | 21 |
| DIN | 2 |
| SCK | GND (mode PLL, tanpa MCLK) |

USB memakai port native ESP32-S3 (GPIO 19/20) - bukan port UART.

## 3. Kredensial WiFi

1. Di folder `S3Air_USB`, salin `wifi_config.example.h` menjadi `wifi_config.h`.
2. Isi nama & password WiFi rumah di file itu.
3. `wifi_config.h` masuk `.gitignore` - tidak akan pernah ter-commit.

Tanpa file itu sketch tetap compile (memakai placeholder di `.ino`):
audio tetap jalan, panel web nonaktif. Kredensial juga bisa diganti dari
tab Sistem di panel web; nilai panel tersimpan di NVS alat dan menimpa
`wifi_config.h`.

## 4. Upload & pemakaian pertama

1. Samakan setting IDE di atas, pilih port COM alat, Upload.
2. Cek Windows: Sound -> `MbahDAC USB` -> Properties -> Advanced harus
   terbaca **16 bit, 48000 Hz**.
3. Buka panel: `http://mbahdac.local` (HP/PC di WiFi yang sama), atau
   alamat IP alat (tertera di tab Sistem & Serial Monitor).

## 5. Panel web

- **EQ** - grafik + RTA; titik band bisa digeser (geser = frekuensi+gain,
  roda mouse = Q, klik ganda = on/off). Tombol mode: PEQ 10 / GEQ 31,
  Bypass, Loudness, Auto-Headroom. Strip fader di bawah sinkron dengan
  titik grafik (sama-sama gain band); strip MAIN = volume master + Mute.
- **Gate** - noise gate (threshold/attack/release) + kurva transfer.
- **Comp** - compressor (threshold/ratio/attack/release/makeup) + meter
  gain reduction + limiter & ceiling.
- **Main** - preamp, trim L/R, fase, Low Cut (HPF), delay L/R, stereo
  width, balance, cek mono, crossfeed, kunci volume maksimum, Dim.
- **Preset** - 5 preset pabrik (Flat, Speaker DIY, Vokal, Bass, Malam),
  10 slot preset sendiri, banding A/B, ekspor/impor JSON.
- **Sistem** - info firmware & jaringan, statistik ring buffer DSP
  (underrun/overflow), reset hitung clip, ganti kredensial WiFi.

Semua fader meluncur halus (glide) - menggeser saat musik jalan tidak
menimbulkan kresek atau putus.

## 6. Kontrol serial (Serial0, 115200 baud)

Satu perintah per baris:

```
set vol=0.8 mute=0
set b0_g=3 b0_f=100 b0_on=1
set g12=4
preset Bass
upreset 2
save 2
state
```

Kunci `set` sama dengan panel: `vol mute dim volMax preamp trimL trimR
polL polR hpfOn hpfFreq hpfSlope bypass loud autoHr eqMode gateOn gateThr
gateAtk gateRel compOn compThr compRatio compAtk compRel compMake limiter
ceiling xfOn xfAmt width mono balance dlyL dlyR`, per band
`b<i>_on b<i>_t b<i>_f b<i>_g b<i>_q b<i>_dyn b<i>_dthr b<i>_drng`
(i = 0..9), GEQ `g<i>` (i = 0..30), dan `clipReset`.

## 7. Troubleshooting

- **Windows tanda seru / Code 10** - USB CDC On Boot belum Disabled,
  atau kabel/port bukan port native S3. Samakan setting bab 1, upload ulang.
- **Panel tidak terbuka** - cek alat & HP satu WiFi; cek Serial Monitor
  untuk alamat IP; pastikan kredensial benar (bab 3).
- **Dengung saat panel dibuka** - gangguan RF WiFi masuk jalur analog;
  turunkan daya pancar WiFi atau perbaiki grounding/power PCM5102.
- **Putus-putus setelah lama** - lihat underrun (`ur`) di tab Sistem;
  catat angkanya sebelum/sesudah sesi panjang untuk diagnosis.

## 8. Riwayat versi

- **v3.3 S3 Air** - spektrum (RTA) dibuat hidup: balistik analyzer beneran (serangan seketika, jatuh +-22 dB/detik, puncak per band), bar jatuh ke lantai saat audio berhenti (tidak membeku lagi), panel mengambil data 150 ms + interpolasi gerak halus di layar.
- **v3.2 S3 Air** - perbaikan bunyi: jalur audio dikembalikan ke 16-bit/48 kHz penuh (UAC + I2S + DSP). Jalur 24-bit v3.0-v3.1 terbukti tidak mengalirkan audio di ESP32-S3 (core hanya mendukung UAC 16-bit di S3; 24-bit khusus ESP32-P4, sesuai contoh resmi core).
- **v3.1 S3 Air** - panel web gaya X Air versi simple (satu fokus per tab,
  grafik meregang penuh, fader panjang), nama proyek jadi S3 Air,
  kode & komentar dirapikan, panduan dipindah ke file ini.
- **v3.0 Gabungan** - audio 24-bit/48 kHz (ditarik di v3.2), DSP pindah ke task RTOS
  (Core 1) + ring buffer SPSC + statistik underrun/overflow,
  kontrol serial Serial0.
- **v2.0** - DSP lengkap (PEQ 10 band + Dynamic EQ, GEQ 31, gate,
  compressor, limiter, delay, width, crossfeed, loudness, RTA FFT),
  preset NVS, WiFi selalu nyala, semua fader glide halus.
