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

- **v4.3.1 S3 Air** - perbaikan posisi fader: rel & kap kini lurus di tengah strip (sebelumnya kap miring +21px ke kanan dari rel karena kap menengahi kotak input, bukan rel; diukur dari piksel screenshot alat user).
- **v4.3 S3 Air** - tema warna SOFT di atas basis v4.1 (pilihan final user setelah rancangan VoiceMeeter oranye & arang ditolak): cyan dilembutkan (#63c7de), pendaran dikurangi, warna band pastel, pink menara dilembutkan; fader diganti kap mixer (rel slot gelap + kap grafit bergerigi bergaris penanda, master bergaris merah muda) dengan garis penanda tengah rel; PERBAIKAN BUG: spektrum RTA kini bergerak di semua tab bergrafik (sebelumnya cuma bergerak di tab EQ lalu beku di tab lain karena loop animasi cuma menggambar ulang pada tab EQ).
- **v4.2 S3 Air** - fader master menara dipindah: meter L/R kini berkelompok di kiri, fader di kanan (tidak lagi di tengah VU, sesuai gambar rancangan); pengerasan stabilitas panel: tab Sistem menyegarkan data tiap 2 detik, panel memuat ulang data otomatis bila koneksi awal gagal, seret titik grafik anti-nyangkut (pointercancel), hitungan meter GR tidak bisa negatif, koordinat grafik aman dari pembagian nol.
- **v4.1 S3 Air** - poles sesuai rancangan: navbar menjadi pil teks ramping (tab aktif = pil cyan menyala, ikon disembunyikan) dan menara master diselaraskan ke gambar rancangan (rel fader merah, meter L/R bersegmen ala LED); tata letak & fungsi sama persis dengan v4.0.
- **v4.0 S3 Air** - tata letak baru MENARA MASTER / L3 (dipilih user dari 3 rancangan tata letak): strip MAIN keluar dari bank fader menjadi menara sendiri di kanan tab EQ (desktop) yang menjulang dua lantai - meter L/R sendiri, fader ekstra panjang, angka dB besar, tombol MUTE; display dan kotak editor menyatu jadi satu konsol; di tab lain dan di HP tampilannya seperti v3.9 (MAIN tetap di bank fader).
- **v3.9 S3 Air** - tema tampilan baru BROADCAST NEON (dipilih user dari 3 rancangan; kerangka v3.8 tidak berubah): latar nyaris hitam kebiruan dengan pendaran cyan dari atas, panel kaca gelap garis #22344d, sudut lebih membulat (10-12px), aksen cyan elektrik #22d3ee + pink #f471b5, kurva EQ & spektrum RTA cyan (dulu oranye/biru), tombol aktif bercahaya, bank fader bertepi atas cyan menyala, strip MAIN berbingkai pink, slider mengikuti aksen sistem.
- **v3.8 S3 Air** - bank fader menempel (fixed) di bawah layar dengan padding bawah agar tidak menutupi slot Preset; baris tab Main bisa membungkus sehingga pilihan slope HPF tidak kepotong; kotak Preset tinggi alami; input angka tab Main diperlebar.
- **v3.8 S3 Air** - dari screenshot asli user: bank fader sticky menimpa daftar slot di tab Preset (konten tertutup bank) - diganti fixed + ruang cadangan di bawah halaman setinggi bank, jadi semua konten bisa di-scroll bebas tanpa tertutup; pilihan slope Low Cut yang terpotong kotak tetangga dibebaskan (baris kontrol kini membungkus); kotak tab Preset tidak lagi dipaksa sama tinggi (blok kosong di kotak Ekspor/CLIP hilang); input angka tab Main dipersempit.
- **v3.7 S3 Air** - perbaikan bank fader dari screenshot asli user: wadah strip (#strips) tidak pernah diberi flex-grow sehingga bank hanya selebar isi dan menyisakan blok kosong besar di kanan; kini #strips mengisi sisa lebar bank dan strip melebar merata sampai strip MAIN. Slider di kotak editor EQ ikut melebar mengikuti kotaknya.
- **v3.6 S3 Air** - perapian simetri panel: bank fader kini mengisi penuh lebar layar (strip band melebar merata, tidak ada lagi celah kosong sebelum strip MAIN), semua kotak editor/parameter menjadi grid kolom sama lebar, preset pabrik - **v3.5 S3 Air** slot user jadi grid rapi, tinggi minimum paksa editor dicabut, dan pada mode GEQ chip + editor PEQ disembunyikan (bukan wilayahnya) agar tidak ada blok sisa.
- **v3.5 S3 Air** - perbaikan meter VU: kolom meter L/R di kanan spektrum akhirnya tampil (rantai tinggi CSS-nya putus sehingga kolom menyusut ~0px dan isinya bergerak di ruang tak terlihat; kini kolom mengisi penuh tinggi display). Bukti uji: data /api/meter sehat - **v3.4 S3 Air** kode panel terbukti menerapkan nilai dengan benar di pengujian lokal. Di HP, pill info WiFi disembunyikan agar pill CLIP tidak terdesak keluar layar.
- **v3.4 S3 Air** - tata letak HP dirapikan total: halaman jadi satu kolom yang bisa di-scroll wajar (tidak lagi dipaksa seukuran layar), header - **v3.3 S3 Air** chip band satu baris bisa digeser, grafik 290px sehingga meter VU L/R di kanan spektrum tampil penuh - **v3.3 S3 Air** bergerak, bank fader lengket di bawah layar dengan strip MAIN selalu terjangkau di kanan.
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
