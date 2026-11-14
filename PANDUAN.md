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
| Partition Scheme | **Huge APP (3MB No OTA)** — wajib sejak v1.1.0 |
| Core esp32 | 3.3.12 |
| Library tambahan | **NimBLE-Arduino** by h2zero (Library Manager; terverifikasi 2.5.2) |

Sejak v1.1.0 (Bluetooth) ukuran sketch ±1,31 MB — TIDAK muat di skema
partisi bawaan (slot aplikasi 1,3 MB), jadi Partition Scheme wajib
**Huge APP**. Mengganti skema memindahkan partisi NVS: upload pertama
dengan skema baru membuat setelan & preset user di alat kembali ke
bawaan SATU KALI. Kredensial WiFi aman (alat jatuh ke `wifi_config.h`
bila NVS kosong); ekspor JSON dulu dari aplikasi bila ada tuning yang
berharga, impor lagi sesudahnya.

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
audio tetap jalan, API & halaman darurat nonaktif. Kredensial juga bisa
diganti dari aplikasi S3 KONSOL (Pengaturan -> Jaringan WiFi alat) atau
halaman darurat; nilai tersimpan di NVS alat dan menimpa `wifi_config.h`.

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
- **DLMS** - mode manajemen speaker per kanal (lihat bawah).

Semua fader meluncur halus (glide) - menggeser saat musik jalan tidak
menimbulkan kresek atau putus.

### Mode DLMS (tab DLMS)

Saklar **DLMS ON** mengganti seluruh rantai stereo reguler dengan
rantai per kanal; OFF mengembalikan panel reguler persis seperti
semula. Master volume, Mute & Dim tetap jalan di kedua mode.

- **Input A / B** - gain (-60..+12 dB) dan delay 0..100 ms per input
  (delay input untuk penyelarasan sistem), masing-masing bisa
  dimatikan. A = kanal kiri USB, B = kanal kanan USB.
- **Kanal L / R** - bebas sendiri-sendiri: sumber (IN A / IN B / A+B),
  HPF dan LPF masing-masing dengan frekuensi & saklar sendiri, tipe
  filter gabungan (LR12/24/48, BW12/24/48), PEQ 3 band (100 Hz,
  1 kHz, 8 kHz), gain, delay 0..20 ms, limiter per kanal, balik
  fase 0°/180°, dan saklar aktif per kanal.
- Crossover adalah gabungan setelan biasa: misalnya L diberi
  HPF 2,5 kHz dan R diberi LPF 2,5 kHz - atau dibalik, semua dari
  kartu kanalnya masing-masing, tidak ada peran yang dikunci.
- Grafik menggambar kurva gabungan tiap kanal (filter + PEQ) di
  atas spektrum RTA yang tetap berjalan.
- Setelan DLMS tersimpan otomatis (tahan mati listrik) dan ikut
  tersimpan di 10 slot preset user.
- Selama tab DLMS terbuka, bank fader menampilkan strip L dan R
  (fader = gain kanal, tombol ON = aktif kanal) di samping strip
  MAIN; area tab bisa digulir di dalam konsol bila isinya tinggi.

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

Kunci DLMS: `dlmsOn inOnA inOnB inGainA inGainB inDlyA inDlyB`, per
kanal `c<i>_*` (i = 0 untuk L, 1 untuk R): `on src hpfOn hpfF lpfOn
lpfF tf p0 p1 p2 peqOn gain dly lim pol`. Contoh:
`set dlmsOn=1 c0_hpfF=200 c1_lpfF=200`.

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

- **v1.1.0 S3 Air — BLUETOOTH (mode lapangan)** (2026-11-14): server BLE NimBLE-Arduino 2.5.2 di Core 0 (audio Core 1 tidak disentuh). GATT: satu service khusus dengan kar. CMD (perintah teks sebaris: state / set k=v&.. / preset / save / wifi — logika set BERBAGI fungsi applySetList dengan /api/set), kar. RESP (jawaban JSON mengalir sebagai notifikasi, diakhiri notifikasi satu byte '\n'), kar. METER (paket biner 55 byte / 50 ms: peak/RMS/GR float + bendera + clip + flt + 31 byte RTA). Perintah dirakit dari ring buffer dan dijalankan di loop() — callback NimBLE hanya menyalin byte. Keamanan: bonding wajib, kar. CMD/RESP terenkripsi; ikatan BARU hanya diterima selama jendela pairing terbuka (60 dtk setelah boot, atau tekan BOOT >= 250 ms kapan pun untuk membuka 60 dtk); HP yang sudah terikat menyambung lagi otomatis. Aplikasi: fallback otomatis WiFi -> Bluetooth + pilihan manual di layar sambut. SYARAT BUILD BARU: library NimBLE-Arduino + Partition Scheme Huge APP (sketch 1.312.367 byte = 41% slot 3 MB; RAM 75%).
- **v1.0.0 S3 Air — RILIS FINAL set fitur WiFi** (2026-11-14): penomoran diulang ke 1.0.0 mengiringi aplikasi S3 KONSOL 1.0. Isi = v5.2.5 (panel web penuh 74 KB diturunkan jadi halaman darurat 2,8 KB — setelan full aplikasi, server/API tetap hidup; ganti WiFi kini dari aplikasi + halaman darurat) + v5.2.4 (RTA ikut dalam respons /api/meter) + v5.2.3 (delay kanal DLMS 0-80 ms, buffer delay 4096 sampel) + v5.2.2 (limiter kanal DLMS diganti dari saturator tanh mentah tanpa oversampling menjadi peak limiter beneran — obat bunyi noise saat ambang diturunkan). Bluetooth untuk mode lapangan = tonggak 1.1, belum termasuk di rilis ini.
- **v5.2.1 S3 Air** - satu-satunya perubahan: `server.enableCORS(true)` sebelum `server.begin()` — API kini membawa header izin lintas-origin, jadi aplikasi Flutter Web (debug di Chrome) bisa membaca /api/state|meter|rta dan mengirim /api/set dari origin localhost. Terbukti perlu 2026-11-13: panel di 192.168.1.26 terbuka normal di browser, tetapi aplikasi web debug ditolak browser (CORS) sebelum firmware memberi izin. Aplikasi native (Android/Windows) tidak terpengaruh; fungsi & DSP identik v5.2.
- **v5.2 S3 Air** - panel web diselaraskan ke rancangan aplikasi + perbaikan tata letak HP; DSP, persistensi, dan lapisan keamanan identik v5.1 (panel ditanam dari berkas pratinjau yang disetujui user lebih dulu). TEMA panel diganti tema aplikasi: latar #17191d, panel #1d2026/#2a2e36, tab aktif balok biru #346ea5, aksen pilihan cyan #38c7e0, garis penanda kap master merah #f47064 — menggantikan SOFT HIJAU v5.x di panel. Perbaikan HP (terukur di 393x873 dari laporan & screenshot user): (1) strip band kini bergeser di jalur gulirnya sendiri dan terpotong rapi di tepi strip MAIN — sebelumnya strip B5 tertumpuk DI BALIK strip master sampai kap putihnya terlihat di dalam master (master tampak bercap dua); meter master yang sempat menyusut kini tampil penuh. (2) Navbar HP satu baris bisa digeser (tinggi 133px/3 baris -> 49px). (3) Tombol ON strip mepet ke bawah sejajar MUTE master (celah 69px -> 8px; rel fader memanjang menyerap ruang slack, 165px -> 226px). (4) Halaman HP jadi satu guliran utuh: cangkang flex 100dvh dilepas pada <=900px dan body diberi ruang ekor 347px — ujung halaman (catatan di dasar tab) yang sebelumnya nyangkut permanen di balik bank fader tetap kini bisa naik bersih di atasnya.
- **v5.1 S3 Air** - RILIS BESAR: persistensi + kualitas + keamanan. (1) PERSISTENSI DIPERBAIKI: di v5.0.8 parameter reguler TIDAK pernah tersimpan otomatis (hanya blob DLMS yang punya dirty-flag) — tuning hilang setiap alat mati; terbukti dari laporan user + audit kode. Kini seluruh keadaan (reguler + DLMS) tersimpan sebagai key "state"+"dlms2": tombol SIMPAN di panel (serial `saveState=1`) menyimpan seketika; auto-save +-2 dtk setelah perubahan terakhir dengan toggle ON/OFF di tab Sistem (bawaan ON, ikut tersimpan); titik oranye di panel menyala selama ada perubahan belum tersimpan (bidang `dirty` di /api/state dan /api/meter). Reset dua tingkat: Reset Setelan (parameter ke bawaan, preset user utuh; serial `resetSoft=1`) dan Reset Pabrik (seluruh namespace dihapus, kredensial WiFi panel dipertahankan; serial `resetFactory=1`), plus jalur fisik: tahan tombol BOOT >=2,5 dtk dalam 6 dtk pertama setelah alat menyala. (2) KELUARAN 24-BIT + DITHER: slot I2S 32-bit membawa data 24-bit rata-kiri (PCM5102 menerima bingkai 32-bit) dengan dither TPDF +-1 LSB24 dari xorshift32 — tanpa transendental per sampel; jalur float dipertahankan sampai kuantisasi akhir (di volume master lazim -12,5 dB, +-2 bit yang dulu hilang kini utuh). (3) TRANSISI BEBAS KLIK: semua delay (reguler L/R, input DLMS A/B, kanal DLMS) membaca dua tap dengan crossfade 5 ms setiap kali tap bergeser — bunyi "ciitt" saat delay digeser hilang; celup transisi preset/perubahan struktur diganti dua fase smoothstep 2x2,5 ms dengan pergantian koefisien tepat di titik nol (sebelumnya celup satu arah 24 sampel). (4) LAPISAN GAGAL-DIAM: boot mulai dari hening + ramp 50 ms; sanitasi tahap akhir (NaN/Inf -> nol + flush seluruh state filter, penahan DC ~3 Hz, jepit keras); pengawas tugas DSP dari loop (macet -> flush state, macet 2 dtk berturut -> restart pengaman); USB berhenti >300 ms -> DMA diumpan hening agar tak ada sisa bunyi berulang. Jumlah flush tampil sebagai `flt` di /api/meter. Verifikasi build: arduino-cli core 3.3.12 bersih — flash 89%, RAM 65%. Uji di alat: `dlmstest` untuk waktu blok + soak `ur`/`ov`/`flt`.
- **v5.0.8 S3 Air** - AKAR MASALAH DLMS DITEMUKAN & DIPERBAIKI. Angka dlmstest v5.0.7 membuktikan: blok DLMS butuh 11,1 ms padahal anggaran realtime 512 sampel @48kHz cuma 10,67 ms (reguler 5,6 ms) — mode DLMS memang mustahil realtime, bukan crash. Penyebabnya: penghalus parameter DLMS memanggil db2lin() (powf) PER SAMPEL (6x per sampel: gain input A/B, gain kanal L/R, ceiling L/R) sementara rantai reguler menghitung targetnya sekali per blok. Perbaikan: semua target db2lin DLMS dihitung sekali per blok di prolog processBlock; per sampel tinggal lerps biasa. Penanda [dbg] v5.0.5 dicabut; dlmstest pengukur waktu dipertahankan sebagai alat verifikasi. Kursor area spektrum diganti dari crosshair (nyaris tak terlihat di latar gelap pada Windows) menjadi `move` (panah empat arah besar, kontras tinggi) atas laporan user bahwa kursor hilang di area spektrum.
- **v5.0.7 S3 Air** - perbaikan DSP + diagnostik pengukur waktu. (1) PEQ DLMS dengan gain ±0,05 dB kini di-bypass total (coefIdentity): biquad peaking RBJ pada gain tepat 0 dB punya kutub persis di lingkaran satuan (resonator marginal, state tak pernah meluruh — cacat DSP yang hanya ada di jalur DLMS karena gain bawaannya 0). (2) `dlmstest=1` kini mengukur waktu proses per fase (keras → meluruh → hening) untuk DLMS dan reguler sebagai kontrol: fase hening yang jauh lebih lambat membuktikan pelambatan denormal FPU sebagai penyebab serak + spektrum beku saat streaming.
- **v5.0.6 S3 Air** - build DIAGNOSTIK lanjutan: keterangan user memperjelas pola aslinya — dengan audio mengalir (kabel soundcard), menyalakan DLMS membuat audio seketika serak, panel tak bisa diubah, spektrum mati (= task DSP crash/halt saat cabang DLMS pertama kali mengeksekusi sampel; di kabel COM tanpa audio cabang itu tak pernah jalan, cocok dengan log v5.0.5 yang bersih). Build ini menambah perintah serial `dlmstest=1`: cabang DLMS dieksekusi pada 200 blok x 512 sampel sintetik memakai setelan DLMS tersimpan, tanpa USB — panic (Guru Meditation + backtrace) tercetak lengkap di Serial0 bila cabang itu penyebabnya.
- **v5.0.5 S3 Air** - build DIAGNOSTIK (sementara): uji user membuktikan hang DLMS juga terjadi dengan kabel di port COM tanpa audio mengalir, jadi cabang audio DSP bebas tuduhan; tersangka menyempit ke sisi Core 0 (proses setelan DLMS / simpan NVS). Build ini memasang penanda Serial0 `[dbg]` di handler /api/set DLMS dan di tiap langkah saveDlmsNvs (begin/putBytes/end) — baris terakhir yang tercetak sebelum hang menunjuk tahap penyebabnya; sesudah vonis, penanda dicabut di rilis perbaikan.
- **v5.0.4 S3 Air** - pengaman boot menanggapi laporan user bahwa menyalakan mode DLMS membuat alat hang total (panel mati, pulih hanya dengan cabut-pasang): mode DLMS kini selalu mulai OFF saat alat menyala dan sesudah memuat preset, walau state tersimpan di NVS berbendera ON — setelan DLMS lain utuh, mode dinyalakan manual dari saklarnya. Alat tidak bisa lagi terkunci dalam state hang saat boot. Penyebab hang sendiri masih diselidiki lewat log Serial0 (panic/watchdog/brownout) yang terbaca saat DLMS dinyalakan.
- **v5.0.3 S3 Air** - strip fader L/R di tab DLMS kini ramping (84px, rapat berdampingan, tidak melebar memenuhi bank) sesuai koreksi user. Pengerasan jalur audio: seluruh kerja DLMS di task audio kini digerbang — selama mode DLMS mati dan tidak ada setelan DLMS yang disentuh, task audio menjalankan NOL instruksi DLMS (rantai reguler identik v4.3.2); koefisien DLMS disiapkan sekali saat ada setelan berubah atau mode dinyalakan.
- **v5.0.2 S3 Air** - tiga perbaikan dari uji alat user atas v5.0.1: (1) mode DLMS kini menyala dalam keadaan NETRAL — HPF/LPF bawaan yang tadinya aktif sebagai contoh silang (L=HPF, R=LPF @2,5 kHz) dimatikan, jadi menyalakan DLMS tidak lagi mengubah suara; setelan DLMS lama dibuang sekali lewat kunci simpan baru. (2) Area tab kini bisa digulir di dalam konsol — isi tab yang tinggi (terutama DLMS) tidak lagi terpotong di balik bank fader. (3) Bank fader di tab DLMS kini menampilkan strip L & R individual (gain kanal + ON) di samping MAIN, sesuai rancangan.
- **v5.0.1 S3 Air** - tema hijau dilembutkan & digelapkan atas koreksi user ("terlalu kontras, kurang gelap soft"): aksen #6fcf97 -> #3f8a5f di seluruh panel (tab aktif, tombol, fader, kurva, spektrum) dan pendaran dikurangi. Fungsi identik dengan v5.0.
- **v5.0 S3 Air** - MODE DLMS ditanam ke firmware (sebelumnya hanya rancangan pratinjau): tab DLMS dengan saklar mode ON/OFF (bypass penuh, setelan tersimpan), kartu INPUT A/B (aktif, gain, delay 0..100 ms) dan dua kartu kanal L/R yang universal & bebas sendiri-sendiri - sumber A/B/A+B, HPF & LPF per kanal (frekuensi + saklar sendiri), tipe filter LR12/24/48 & BW12/24/48, PEQ 3 band per kanal, gain, delay 0..20 ms, limiter per kanal, balik fase; grafik menggambar kurva gabungan tiap kanal di atas spektrum RTA. Setelan DLMS tersimpan otomatis di NVS dan ikut di 10 slot preset user; kunci serial DLMS ditambahkan. TEMA WARNA diganti SOFT HIJAU ala VoiceMeeter (aksen hijau soft #6fcf97 menggantikan cyan; strip master & kanal R jadi mawar soft; kurva & spektrum ikut hijau). RAM naik ke 55% (buffer delay input DLMS).
- **v4.3.2 S3 Air** - spektrum RTA kini aktif di SEMUA tab: tab Gate & Comp sekarang juga menggambar spektrum di belakang kurva transfernya (di v4.3 dua tab ini masih dikecualikan), dan loop animasi menggambar ulang di semua tab tanpa kecuali.
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
