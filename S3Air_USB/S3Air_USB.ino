// ============================================================
//  S3 Air - USB DAC + DSP Console
//  ESP32-S3 (N16R8) + PCM5102 | USB Audio 16-bit / 48 kHz
//  (core ESP32-S3 hanya mendukung UAC 16-bit; jalur 24-bit khusus ESP32-P4)
//  DSP di task FreeRTOS (Core 1); kontrol aplikasi S3 KONSOL via API HTTP,
//  halaman darurat, dan perintah serial.
//  Versi firmware: FW_VERSION di bawah. Panduan lengkap,
//  setting IDE & wiring: lihat PANDUAN.md di repo ini.
// ============================================================

#include <Arduino.h>
#include "ESP_I2S.h"
#include "USB.h"
#include "USBAudioCard.h"
#include <WiFi.h>
#include <WebServer.h>
#include <ESPmDNS.h>
#include <Preferences.h>
#include <NimBLEDevice.h>
#include <math.h>
#include "dsp_types.h"

// ---- Kredensial WiFi rumah ----
// Taruh di file wifi_config.h (salin dari wifi_config.example.h lalu isi).
// wifi_config.h TIDAK ikut ter-commit (.gitignore) - password tidak bocor ke repo.
// Tanpa file itu sketch tetap compile memakai placeholder di bawah
// (audio tetap jalan, panel web nonaktif sampai kredensial diisi).
#if __has_include("wifi_config.h")
#include "wifi_config.h"
#else
#define WIFI_SSID "ISI_NAMA_WIFI_RUMAH"
#define WIFI_PASS "ISI_PASSWORD_WIFI"
#endif
#define DAC_HOSTNAME "mbahdac"
#define FW_VERSION "v1.1.0 S3 Air"

static const int PIN_BCK = 42, PIN_LRCK = 21, PIN_DIN = 2;
static const float SAMPLE_RATE = 48000.0f;
static const float GEQ_FREQ[NUM_GEQ] = {20,25,31.5f,40,50,63,80,100,125,160,200,250,315,400,500,630,800,1000,1250,1600,2000,2500,3150,4000,5000,6300,8000,10000,12500,16000,20000};
static const float GEQ_Q = 4.318f; // 1/3 oktaf

// ============================================================
//  Parameter & koefisien
// ============================================================
DspParams params;
static float dynEffGain[NUM_BANDS];     // gain efektif band dinamis (dihitung runtime)
static float dynEnv[NUM_BANDS];         // envelope detektor per band

// Bank filter utama: 0..3 HPF cascade, 4..13 PEQ, 14..15 loudness shelf
#define NMAIN 16
static BiquadCoef  mainBank[2][NMAIN];
static BiquadState mainState[NMAIN];
static BiquadCoef  geqBank[2][NUM_GEQ];
static BiquadState geqState[NUM_GEQ];
static volatile uint8_t activeBank = 0;
static volatile bool coefDirty = true;
// Celup transisi dua fase (v5.1 tahap 3): turun smoothstep 2,5 ms -> tahan
// di nol sampai koefisien tertukar -> naik smoothstep 2,5 ms. Menggantikan
// celup satu arah 24 sampel v5.0 yang masih menyisakan tepi tajam.
static const uint16_t DIP_HALF = 120;
static uint16_t dipRemain = 0;
static uint8_t dipPhase = 0;        // 0 diam, 1 turun, 3 tahan-di-nol, 2 naik
static bool dipDoSwap = false;
// Crossfade tap delay: tiap tap bergeser, dua tap (lama+baru) dibaca dan
// dicampur 5 ms — membunuh bunyi "ciitt"/klik saat delay digeser/dipindah.
struct DlyTap { uint16_t cur = 0, from = 0, fade = 0; };
static const uint16_t DLY_XFADE = 240;
static DlyTap tapInA, tapInB, tapCh0, tapCh1, tapRegL, tapRegR;
static inline float dlyRead(const float *buf, uint16_t wr, DlyTap &t, uint16_t target, uint16_t size) {
  if (target != t.cur) { t.from = t.cur; t.cur = target; t.fade = DLY_XFADE; }
  float y = buf[(wr + size - t.cur) % size];
  if (t.fade) {
    float g = 1.0f - (float)t.fade / (float)DLY_XFADE;
    float yo = buf[(wr + size - t.from) % size];
    y += (yo - y) * (1.0f - g);
    t.fade--;
  }
  return y;
}

// Keadaan dinamika & output (jalan di callback audio)
static float gateEnv = 0, gateGain = 1;
static float compEnv = 0;
static volatile float meterPeakL = 0, meterPeakR = 0, meterRmsL = 0, meterRmsR = 0;
static volatile float grMeterDb = 0;
static volatile uint32_t clipCount = 0;
static float smoothVol = 0;
static float smoothPreamp = 0;
static float smoothTrimL = 1.0f, smoothTrimR = 1.0f;
static float smoothXfAmt = 0;
static float smoothWidth = 1.0f;
static float smoothBalance = 0;
static float smoothDlyL = 0, smoothDlyR = 0;
static float smoothCeil = 1.0f;
static float xfLpL = 0, xfLpR = 0;
static const float SMOOTH_COEF = 0.002f;
// Delay line bersama (reguler + kanal DLMS). v5.2.3: diperbesar ke 4096
// sampel (85 ms) agar delay kanal DLMS bisa 0..80 ms; reguler tetap 0..20.
static float dlyBufL[4096], dlyBufR[4096];
static uint16_t dlyWr = 0;
// DLMS: delay input maks 100 ms (4800 sampel) + penghalus khusus mode DLMS
static float inDlyBufA[4800], inDlyBufB[4800];
static uint16_t inDlyWr = 0;
static float smInGainA = 1.0f, smInGainB = 1.0f;
static float smChGain[2] = {1.0f, 1.0f}, smChCeil[2] = {1.0f, 1.0f};
static float limEnv[2] = {0.0f, 0.0f}, limGainSm[2] = {1.0f, 1.0f}; // state peak-limiter kanal DLMS (v5.2.2)
static float smChDly[2] = {0, 0}, smInDlyA = 0, smInDlyB = 0;
// Tangkap sampel untuk RTA (mono mix, ring 1024)
static float rtaBuf[1024];
static uint16_t rtaWr = 0;
// RTA smoothing
static float rtaSmooth[NUM_GEQ];
// --- Parameter filter terhalus (glide) + snapshot nilai terbangun ---
static float smBandGain[NUM_BANDS], smBandFreq[NUM_BANDS], smBandQ[NUM_BANDS];
static float smGeqGain[NUM_GEQ];
static float smHpfFreq = 30.0f, smVolForLoud = 0.8f;
static float smGateThr = 0.0f, smCompThrDb = -18.0f, smCompRatio = 3.0f, smMakeup = 1.0f;
static bool builtOn[NUM_BANDS]; static uint8_t builtType[NUM_BANDS];
static float builtGain[NUM_BANDS], builtFreq[NUM_BANDS], builtQ[NUM_BANDS], builtGeq[NUM_GEQ];
static float builtHpfFreq = -1; static bool builtHpfOn = false;
static uint8_t builtSlope = 0, builtMode = 0; static bool builtBypass = false;
static float builtLs = 999, builtHs = 999;
static uint16_t ctrlCount = 0;
static volatile bool armDip = false; // celup singkat HANYA untuk perubahan struktur

// ============================================================
//  Helper biquad (RBJ Audio EQ Cookbook)
// ============================================================
static inline float db2lin(float db) { return powf(10.0f, db * 0.05f); }

static void coefIdentity(BiquadCoef &c) { c.b0 = 1; c.b1 = c.b2 = c.a1 = c.a2 = 0; c.bypass = true; }

static void coefBiquad(BiquadCoef &c, uint8_t type, float fs, float f0, float gainDb, float q) {
  if (type == BT_PEAKING && fabsf(gainDb) < 0.01f) { coefIdentity(c); return; }
  if ((type == BT_LOWSHELF || type == BT_HIGHSHELF) && fabsf(gainDb) < 0.01f) { coefIdentity(c); return; }
  f0 = constrain(f0, 10.0f, fs * 0.49f);
  q = constrain(q, 0.1f, 20.0f);
  float w = 2.0f * PI * f0 / fs, cw = cosf(w), sw = sinf(w);
  float A, alpha, a0;
  switch (type) {
    case BT_LOWSHELF: case BT_HIGHSHELF: {
      A = db2lin(gainDb * 0.5f);
      alpha = sw * 0.5f * sqrtf((A + 1.0f / A) * (1.0f / 0.9f - 1.0f) + 2.0f);
      float sA = sqrtf(A), t = 2.0f * sA * alpha;
      if (type == BT_LOWSHELF) {
        c.b0 = A * ((A + 1) - (A - 1) * cw + t); c.b1 = 2 * A * ((A - 1) - (A + 1) * cw); c.b2 = A * ((A + 1) - (A - 1) * cw - t);
        a0 = (A + 1) + (A - 1) * cw + t; c.a1 = -2 * ((A - 1) + (A + 1) * cw); c.a2 = (A + 1) + (A - 1) * cw - t;
      } else {
        c.b0 = A * ((A + 1) + (A - 1) * cw + t); c.b1 = -2 * A * ((A - 1) + (A + 1) * cw); c.b2 = A * ((A + 1) + (A - 1) * cw - t);
        a0 = (A + 1) - (A - 1) * cw + t; c.a1 = 2 * ((A - 1) - (A + 1) * cw); c.a2 = (A + 1) - (A - 1) * cw - t;
      }
      break;
    }
    case BT_NOTCH:
      alpha = sw / (2.0f * q);
      c.b0 = 1; c.b1 = -2 * cw; c.b2 = 1; a0 = 1 + alpha; c.a1 = -2 * cw; c.a2 = 1 - alpha;
      break;
    case BT_BANDPASS:
      alpha = sw / (2.0f * q);
      c.b0 = alpha; c.b1 = 0; c.b2 = -alpha; a0 = 1 + alpha; c.a1 = -2 * cw; c.a2 = 1 - alpha;
      break;
    default: // PEAKING
      A = db2lin(gainDb * 0.5f);
      alpha = sw / (2.0f * q);
      c.b0 = 1 + alpha * A; c.b1 = -2 * cw; c.b2 = 1 - alpha * A;
      a0 = 1 + alpha / A; c.a1 = -2 * cw; c.a2 = 1 - alpha / A;
      break;
  }
  c.b0 /= a0; c.b1 /= a0; c.b2 /= a0; c.a1 /= a0; c.a2 /= a0;
  c.bypass = false;
}

static void coefHighpass2(BiquadCoef &c, float fs, float f0, bool on) {
  if (!on) { coefIdentity(c); return; }
  f0 = constrain(f0, 10.0f, fs * 0.49f);
  float w = 2.0f * PI * f0 / fs, cw = cosf(w), sw = sinf(w);
  float alpha = sw / (2.0f * 0.7071f), a0 = 1 + alpha;
  c.b0 = (1 + cw) * 0.5f / a0; c.b1 = -(1 + cw) / a0; c.b2 = c.b0;
  c.a1 = -2 * cw / a0; c.a2 = (1 - alpha) / a0; c.bypass = false;
}

// HPF satu kutub dinyatakan sebagai biquad (b2 = a2 = 0)
static void coefHighpass1(BiquadCoef &c, float fs, float f0, bool on) {
  if (!on) { coefIdentity(c); return; }
  float rc = 1.0f / (2.0f * PI * f0), dt = 1.0f / fs, a = rc / (rc + dt);
  c.b0 = a; c.b1 = -a; c.b2 = 0; c.a1 = -a; c.a2 = 0; c.bypass = false;
}

// ============================================================
//  Preset dasar
// ============================================================
static void setFlat(DspParams &p) {
  memset(&p, 0, sizeof(p));
  p.preampDb = 0; p.trimLDb = 0; p.trimRDb = 0;
  p.hpfOn = true; p.hpfFreq = 30; p.hpfSlope = 12;
  p.eqMode = 0;
  const float defF[NUM_BANDS] = {31, 62, 125, 250, 500, 1000, 2000, 4000, 8000, 16000};
  for (int i = 0; i < NUM_BANDS; i++) {
    p.band[i].on = true; p.band[i].type = BT_PEAKING;
    p.band[i].freq = defF[i]; p.band[i].gainDb = 0; p.band[i].q = 1.0f;
    p.band[i].dynOn = false; p.band[i].dynThrDb = -24; p.band[i].dynRangeDb = 0;
  }
  p.gateThrDb = -55; p.gateAttackMs = 1; p.gateReleaseMs = 120;
  p.compThrDb = -18; p.compRatio = 3; p.compAttackMs = 10; p.compReleaseMs = 180; p.compMakeupDb = 0;
  p.limiterOn = true; p.ceilingDb = -1;
  p.crossfeedAmt = 0.25f; p.width = 1.0f;
  p.volume = 0.8f; p.volMax = 1.0f;
}

static bool setFactoryPreset(const char *name, DspParams &p) {
  setFlat(p);
  if (!strcmp(name, "flat")) return true;
  if (!strcmp(name, "speaker")) { // Speaker DIY: angkat ujung bawah & atas
    p.band[1].gainDb = 2; p.band[8].gainDb = 3; p.band[9].gainDb = 5; p.preampDb = -3; return true;
  }
  if (!strcmp(name, "vokal")) {
    p.band[3].gainDb = -2; p.band[6].gainDb = 3; p.band[7].gainDb = 2; p.preampDb = -2; return true;
  }
  if (!strcmp(name, "bass")) {
    p.band[0].gainDb = 4; p.band[1].gainDb = 4; p.band[2].gainDb = 1; p.preampDb = -4; return true;
  }
  if (!strcmp(name, "malam")) {
    p.preampDb = -6; p.band[1].gainDb = -2; p.band[9].gainDb = -3; p.ceilingDb = -3;
    p.compOn = true; p.loudnessOn = true; return true;
  }
  return false;
}

// ============================================================
//  Bangun ulang koefisien (bank ganda, ditukar di batas chunk)
// ============================================================
static float bandEffGain(int i) { // gain band termasuk sumbangan dynamic EQ terakhir
  return params.band[i].gainDb + dynEffGain[i];
}

static bool filtersChanged() {
  if (builtMode != params.eqMode || builtBypass != params.bypass) return true;
  bool hpf = params.hpfOn && !params.bypass;
  if (builtHpfOn != hpf || builtSlope != params.hpfSlope) return true;
  if (fabsf(builtHpfFreq - smHpfFreq) > 0.01f * smHpfFreq + 0.05f) return true;
  for (int i = 0; i < NUM_BANDS; i++) {
    if (builtOn[i] != params.band[i].on || builtType[i] != params.band[i].type) return true;
    if (fabsf(builtGain[i] - smBandGain[i]) > 0.03f) return true;
    if (fabsf(builtFreq[i] - smBandFreq[i]) > 0.01f * smBandFreq[i] + 0.05f) return true;
    if (fabsf(builtQ[i] - smBandQ[i]) > 0.02f) return true;
  }
  for (int i = 0; i < NUM_GEQ; i++) if (fabsf(builtGeq[i] - smGeqGain[i]) > 0.03f) return true;
  float d = (params.loudnessOn && !params.bypass) ? constrain(0.85f - smVolForLoud, 0.0f, 0.85f) : 0.0f;
  if (fabsf(builtLs - d * 9.0f) > 0.05f || fabsf(builtHs - d * 6.0f) > 0.05f) return true;
  return false;
}
static void snapSmoothToParams() {
  for (int i = 0; i < NUM_BANDS; i++) { smBandGain[i] = params.band[i].gainDb; smBandFreq[i] = params.band[i].freq; smBandQ[i] = params.band[i].q; }
  for (int i = 0; i < NUM_GEQ; i++) smGeqGain[i] = params.geq[i];
  smHpfFreq = params.hpfFreq; smVolForLoud = params.volume;
  smGateThr = db2lin(params.gateThrDb); smCompThrDb = params.compThrDb;
  smCompRatio = params.compRatio; smMakeup = db2lin(params.compMakeupDb);
}

static void rebuildCoefficients() {
  int nb = activeBank ^ 1;
  BiquadCoef *mb = mainBank[nb];
  bool bp = params.bypass;
  float fs = SAMPLE_RATE;
  bool hpf = params.hpfOn && !bp;
  // HPF cascade sesuai slope: 6 = 1 kutub, 12 = BW2, 18 = BW2+1 kutub, 24 = 2x BW2 (LR4)
  coefIdentity(mb[0]); coefIdentity(mb[1]); coefIdentity(mb[2]); coefIdentity(mb[3]);
  if (hpf) {
    if (params.hpfSlope <= 6) { coefHighpass1(mb[0], fs, smHpfFreq, true); }
    else if (params.hpfSlope <= 12) { coefHighpass2(mb[0], fs, smHpfFreq, true); }
    else if (params.hpfSlope <= 18) { coefHighpass2(mb[0], fs, smHpfFreq, true); coefHighpass1(mb[1], fs, smHpfFreq, true); }
    else { coefHighpass2(mb[0], fs, smHpfFreq, true); coefHighpass2(mb[1], fs, smHpfFreq, true); }
  }
  // PEQ 10 band
  for (int i = 0; i < NUM_BANDS; i++) {
    if (!bp && params.band[i].on && params.eqMode == 0)
      coefBiquad(mb[4 + i], params.band[i].type, fs, smBandFreq[i], smBandGain[i], smBandQ[i]);
    else coefIdentity(mb[4 + i]);
  }
  // Loudness: shelf bawah/atas mengikuti kekurangan volume (Fletcher-Munson kasar)
  float ls = 0, hs = 0;
  if (params.loudnessOn && !bp) {
    float deficit = constrain(0.85f - smVolForLoud, 0.0f, 0.85f);
    ls = deficit * 9.0f; hs = deficit * 6.0f;
  }
  coefBiquad(mb[14], BT_LOWSHELF, fs, 120, ls, 0.9f);
  coefBiquad(mb[15], BT_HIGHSHELF, fs, 9000, hs, 0.9f);
  // GEQ 31 band
  for (int i = 0; i < NUM_GEQ; i++) {
    if (!bp && params.eqMode == 1)
      coefBiquad(geqBank[nb][i], BT_PEAKING, fs, GEQ_FREQ[i], smGeqGain[i], GEQ_Q);
    else coefIdentity(geqBank[nb][i]);
  }
  activeBank = nb;
  coefDirty = false;
  // snapshot nilai yang baru dibangun (acuan deteksi perubahan berikutnya)
  builtMode = params.eqMode; builtBypass = bp; builtHpfOn = hpf; builtSlope = params.hpfSlope; builtHpfFreq = smHpfFreq;
  for (int i = 0; i < NUM_BANDS; i++) { builtOn[i] = params.band[i].on; builtType[i] = params.band[i].type; builtGain[i] = smBandGain[i]; builtFreq[i] = smBandFreq[i]; builtQ[i] = smBandQ[i]; }
  for (int i = 0; i < NUM_GEQ; i++) builtGeq[i] = smGeqGain[i];
  { float d = (params.loudnessOn && !bp) ? constrain(0.85f - smVolForLoud, 0.0f, 0.85f) : 0.0f; builtLs = d * 9.0f; builtHs = d * 6.0f; }
}

// Perkiraan puncak kurva EQ (buat auto-headroom), dihitung di konteks web
static float estimateEqPeakDb() {
  float peak = -60;
  BiquadCoef *mb = mainBank[activeBank];
  for (int k = 0; k < 48; k++) {
    float f = 20.0f * powf(1000.0f, k / 47.0f);
    float w = 2.0f * PI * f / SAMPLE_RATE, c1 = cosf(w), c2 = cosf(2 * w), tot = 1.0f;
    auto mag = [&](const BiquadCoef &c) {
      if (c.bypass) return;
      float n = c.b0*c.b0 + c.b1*c.b1 + c.b2*c.b2 + 2*(c.b0*c.b1 + c.b1*c.b2)*c1 + 2*c.b0*c.b2*c2;
      float d = 1 + c.a1*c.a1 + c.a2*c.a2 + 2*(c.a1 + c.a1*c.a2)*c1 + 2*c.a2*c2;
      if (d > 1e-9f) tot *= n / d;
    };
    for (int i = 0; i < NMAIN; i++) mag(mb[i]);
    if (params.eqMode == 1) for (int i = 0; i < NUM_GEQ; i++) mag(geqBank[activeBank][i]);
    float db = 10.0f * log10f(fmaxf(tot, 1e-9f));
    if (db > peak) peak = db;
  }
  return peak;
}

// ============================================================
//  Pemrosesan audio (callback USBAudioCard)
// ============================================================
USBAudioCard uac(48000, UAC_BPS_16, UAC_SPK_STEREO, UAC_MIC_NONE); // 16-bit/48kHz - format UAC yang didukung core di S3
I2SClass i2s;

static inline float runBiquad(BiquadCoef &c, BiquadState &s, float x, bool left) {
  if (c.bypass) return x;
  if (left) {
    float y = c.b0 * x + c.b1 * s.x1L + c.b2 * s.x2L - c.a1 * s.y1L - c.a2 * s.y2L;
    s.x2L = s.x1L; s.x1L = x; s.y2L = s.y1L; s.y1L = y; return y;
  }
  float y = c.b0 * x + c.b1 * s.x1R + c.b2 * s.x2R - c.a1 * s.y1R - c.a2 * s.y2R;
  s.x2R = s.x1R; s.x1R = x; s.y2R = s.y1R; s.y1R = y; return y;
}

// Koefisien kelancaran dari waktu (ms) pada laju sampel
static inline float msCoef(float ms) {
  if (ms <= 0.05f) return 1.0f;
  return 1.0f - expf(-1.0f / (SAMPLE_RATE * ms * 0.001f));
}

// Detektor band dinamis: biquad bandpass lepas (keadaan sendiri per band)
static BiquadCoef  dynCoef[NUM_BANDS];
static BiquadState dynState[NUM_BANDS];
static void rebuildDynCoefs() {
  for (int i = 0; i < NUM_BANDS; i++)
    coefBiquad(dynCoef[i], BT_BANDPASS, SAMPLE_RATE, params.band[i].freq, 0, params.band[i].q);
}

// ============================================================
//  DLMS — koefisien, keadaan & pengatur per kanal
// ============================================================
DlmsParams dlms;
static BiquadCoef  dlmsCoef[2][11];   // 0-3 HPF, 4-7 LPF, 8-10 PEQ
static BiquadState dlmsState[2][11];
static const float DLMS_PEQ_F[3] = {100, 1000, 8000};
static float smDlmsHpfF[2] = {2500, 2500}, smDlmsLpfF[2] = {2500, 2500};
static float smDlmsPeq[2][3] = {};
static float bDlmsHpfF[2] = {-1, -1}, bDlmsLpfF[2] = {-1, -1};
static float bDlmsPeq[2][3] = {};
static uint16_t bDlmsBits[2] = {0xFFFF, 0xFFFF};
static volatile bool dlmsSaveDirty = false;
static volatile bool stateDirty = false;   // ada perubahan state belum tersimpan (reguler/DLMS)
static bool sysAutoSave = true;            // toggle auto-save (tersimpan di NVS "autosave")
static uint32_t lastChangeMs = 0;          // kapan perubahan terakhir (debounce simpan)
static uint32_t lastStateSave = 0;         // kapan simpan terakhir
static bool serActed = false;              // penanda aksi (simpan/reset) dari serial
static volatile bool dlmsTouched = true;  // ada setelan DLMS baru: task audio menyiapkan koefisiennya sekali

static void setDlmsDefaults() {
  memset(&dlms, 0, sizeof(dlms));
  dlms.on = false;
  dlms.inOnA = dlms.inOnB = true;
  for (int c = 0; c < 2; c++) {
    DlmsCh &h = dlms.ch[c];
    h.on = true; h.src = (uint8_t)c;              // L <- IN A, R <- IN B
    h.hpfF = 2500; h.lpfF = 2500; h.tf = 1;       // LR24
    h.peqOn = true;
    h.limDb = (c == 0) ? -3.0f : -1.0f;
  }
  // HPF/LPF bawaan MATI: mode DLMS menyala dalam keadaan netral (full-range),
  // crossover adalah setelan yang dipilih user sendiri, bukan jebakan bawaan.
}

static void coefHP2q(BiquadCoef &c, float fs, float f0, float q, bool on) {
  if (!on) { coefIdentity(c); return; }
  f0 = constrain(f0, 10.0f, fs * 0.49f);
  float w = 2.0f * PI * f0 / fs, cw = cosf(w), sw = sinf(w);
  float alpha = sw / (2.0f * q), a0 = 1 + alpha;
  c.b0 = (1 + cw) * 0.5f / a0; c.b1 = -(1 + cw) / a0; c.b2 = c.b0;
  c.a1 = -2 * cw / a0; c.a2 = (1 - alpha) / a0; c.bypass = false;
}
static void coefLP2q(BiquadCoef &c, float fs, float f0, float q, bool on) {
  if (!on) { coefIdentity(c); return; }
  f0 = constrain(f0, 10.0f, fs * 0.49f);
  float w = 2.0f * PI * f0 / fs, cw = cosf(w), sw = sinf(w);
  float alpha = sw / (2.0f * q), a0 = 1 + alpha;
  c.b0 = (1 - cw) * 0.5f / a0; c.b1 = (1 - cw) / a0; c.b2 = c.b0;
  c.a1 = -2 * cw / a0; c.a2 = (1 - alpha) / a0; c.bypass = false;
}
static void coefLP1(BiquadCoef &c, float fs, float f0, bool on) {
  if (!on) { coefIdentity(c); return; }
  float rc = 1.0f / (2.0f * PI * f0), dt = 1.0f / fs, a = dt / (rc + dt);
  c.b0 = a; c.b1 = 0; c.b2 = 0; c.a1 = -(1.0f - a); c.a2 = 0; c.bypass = false;
}

// Isi 4 seksi biquad untuk satu filter (HP/LP) sesuai tipe gabungan tf
static void buildFilterSet(BiquadCoef *dst, bool lowpass, float f, uint8_t tf, bool on) {
  for (int i = 0; i < 4; i++) coefIdentity(dst[i]);
  if (!on) return;
  auto S2 = [&](int i, float q) { if (lowpass) coefLP2q(dst[i], SAMPLE_RATE, f, q, true); else coefHP2q(dst[i], SAMPLE_RATE, f, q, true); };
  auto S1 = [&](int i) { if (lowpass) coefLP1(dst[i], SAMPLE_RATE, f, true); else coefHighpass1(dst[i], SAMPLE_RATE, f, true); };
  switch (tf) {
    case 0: S1(0); S1(1); break;                                                      // LR12
    case 1: S2(0, 0.7071f); S2(1, 0.7071f); break;                                    // LR24
    case 2: S2(0, 0.7071f); S2(1, 0.7071f); S2(2, 0.7071f); S2(3, 0.7071f); break;    // LR48
    case 3: S2(0, 0.7071f); break;                                                    // BW12
    case 4: S2(0, 0.5412f); S2(1, 1.3065f); break;                                    // BW24
    default: S2(0, 0.3896f); S2(1, 0.6266f); S2(2, 1.1759f); S2(3, 2.5629f); break;   // BW48
  }
}

static void rebuildDlmsCoefs() {
  for (int c = 0; c < 2; c++) {
    DlmsCh &h = dlms.ch[c];
    buildFilterSet(&dlmsCoef[c][0], false, smDlmsHpfF[c], h.tf, h.hpfOn);
    buildFilterSet(&dlmsCoef[c][4], true, smDlmsLpfF[c], h.tf, h.lpfOn);
    for (int j = 0; j < 3; j++) {
      if (h.peqOn && fabsf(smDlmsPeq[c][j]) > 0.05f) coefBiquad(dlmsCoef[c][8 + j], BT_PEAKING, SAMPLE_RATE, DLMS_PEQ_F[j], smDlmsPeq[c][j], 1.0f);
      else coefIdentity(dlmsCoef[c][8 + j]);
    }
    bDlmsHpfF[c] = smDlmsHpfF[c]; bDlmsLpfF[c] = smDlmsLpfF[c];
    for (int j = 0; j < 3; j++) bDlmsPeq[c][j] = smDlmsPeq[c][j];
    bDlmsBits[c] = (uint16_t)((h.on ? 1 : 0) | (h.hpfOn ? 2 : 0) | (h.lpfOn ? 4 : 0) |
                              (h.peqOn ? 8 : 0) | (h.src << 4) | (h.tf << 6));
  }
}
static bool dlmsChanged() {
  for (int c = 0; c < 2; c++) {
    DlmsCh &h = dlms.ch[c];
    uint16_t bits = (uint16_t)((h.on ? 1 : 0) | (h.hpfOn ? 2 : 0) | (h.lpfOn ? 4 : 0) |
                               (h.peqOn ? 8 : 0) | (h.src << 4) | (h.tf << 6));
    if (bits != bDlmsBits[c]) return true;
    if (fabsf(bDlmsHpfF[c] - smDlmsHpfF[c]) > 0.01f * smDlmsHpfF[c] + 0.05f) return true;
    if (fabsf(bDlmsLpfF[c] - smDlmsLpfF[c]) > 0.01f * smDlmsLpfF[c] + 0.05f) return true;
    for (int j = 0; j < 3; j++) if (fabsf(bDlmsPeq[c][j] - smDlmsPeq[c][j]) > 0.03f) return true;
  }
  return false;
}
static void snapDlmsSmooth() {
  for (int c = 0; c < 2; c++) {
    smDlmsHpfF[c] = dlms.ch[c].hpfF; smDlmsLpfF[c] = dlms.ch[c].lpfF;
    for (int j = 0; j < 3; j++) smDlmsPeq[c][j] = dlms.ch[c].peq[j];
  }
}

// Satu kunci setelan DLMS (dipakai panel web & serial bersama)
static bool dlmsSetKeyImpl(const String &k, float v, bool b) {
  if (k == "dlmsOn") { dlms.on = b; dlmsSaveDirty = true; return true; }
  if (k == "inOnA") { dlms.inOnA = b; dlmsSaveDirty = true; return true; }
  if (k == "inOnB") { dlms.inOnB = b; dlmsSaveDirty = true; return true; }
  if (k == "inGainA") { dlms.inGainA = constrain(v, -60.0f, 12.0f); dlmsSaveDirty = true; return true; }
  if (k == "inGainB") { dlms.inGainB = constrain(v, -60.0f, 12.0f); dlmsSaveDirty = true; return true; }
  if (k == "inDlyA") { dlms.inDlyA = constrain(v, 0.0f, 100.0f); dlmsSaveDirty = true; return true; }
  if (k == "inDlyB") { dlms.inDlyB = constrain(v, 0.0f, 100.0f); dlmsSaveDirty = true; return true; }
  if (k.length() >= 4 && k[0] == 'c' && (k[1] == '0' || k[1] == '1') && k[2] == '_') {
    DlmsCh &h = dlms.ch[k[1] - '0'];
    String f = k.substring(3);
    if (f == "on") h.on = b;
    else if (f == "src") h.src = (uint8_t)constrain((int)v, 0, 2);
    else if (f == "hpfOn") { h.hpfOn = b; armDip = true; }
    else if (f == "hpfF") h.hpfF = constrain(v, 20.0f, 20000.0f);
    else if (f == "lpfOn") { h.lpfOn = b; armDip = true; }
    else if (f == "lpfF") h.lpfF = constrain(v, 20.0f, 20000.0f);
    else if (f == "tf") { h.tf = (uint8_t)constrain((int)v, 0, 5); armDip = true; }
    else if (f == "peqOn") h.peqOn = b;
    else if (f == "p0") h.peq[0] = constrain(v, -12.0f, 12.0f);
    else if (f == "p1") h.peq[1] = constrain(v, -12.0f, 12.0f);
    else if (f == "p2") h.peq[2] = constrain(v, -12.0f, 12.0f);
    else if (f == "gain") h.gainDb = constrain(v, -60.0f, 12.0f);
    else if (f == "dly") h.dlyMs = constrain(v, 0.0f, 80.0f);
    else if (f == "lim") h.limDb = constrain(v, -24.0f, 0.0f);
    else if (f == "pol") h.pol = b;
    else return false;
    dlmsSaveDirty = true; return true;
  }
  return false;
}
static bool dlmsSetKey(const String &k, float v, bool b) {
  bool ok = dlmsSetKeyImpl(k, v, b);
  if (ok) dlmsTouched = true;
  return ok;
}


// ============================================================
//  RTOS: ring SPSC — callback USB ringan, DSP di task khusus Core 1
// ============================================================
#define RING_FRAMES 4096
static int16_t ringIn[RING_FRAMES * 2];
static int32_t ringOut[RING_FRAMES * 2];   // v5.1: keluaran 24-bit rata-kiri dalam kata 32-bit
static volatile uint32_t ringInW = 0, ringInR = 0, ringOutW = 0, ringOutR = 0;
static volatile uint32_t statUnderrun = 0, statOverflow = 0;
static TaskHandle_t dspTaskHandle = NULL;
static int16_t dspBlock[512 * 2];
static int32_t dspOut[512 * 2];

// --- Kuantisasi keluaran 24-bit + dither TPDF (v5.1 tahap 2) ---
// Slot I2S 32-bit (PCM5102 menerima bingkai 32-bit); data 24-bit rata-kiri.
// Dither segitiga +-1 LSB24 dari xorshift32 — tanpa transendental per sampel.
static uint32_t dithState = 0x2545F491u;
static inline float dithU() {
  dithState ^= dithState << 13; dithState ^= dithState >> 17; dithState ^= dithState << 5;
  return (float)(dithState >> 8) * (1.0f / 16777216.0f);
}
static inline int32_t quant24(float v) {
  float q = v * 8388608.0f + (dithU() - dithU());
  if (q > 8388607.0f) q = 8388607.0f;
  else if (q < -8388608.0f) q = -8388608.0f;
  int32_t iv = (int32_t)(q >= 0.0f ? q + 0.5f : q - 0.5f);
  return iv << 8;
}

// --- Lapisan gagal-diam (v5.1 tahap 4) ---
static volatile uint32_t dspHeartbeat = 0;    // detak tugas DSP untuk pengawas
static volatile uint32_t lastSpkMs = 0;       // kapan callback USB terakhir jalan
static volatile bool stateFlushNeeded = false;
static volatile uint32_t statFlush = 0;
static float dcxL = 0, dcyL = 0, dcxR = 0, dcyR = 0;  // state penahan DC keluaran
static uint32_t bootRamp = 0;                 // ramp naik 50 ms pertama (dalam sampel)
static void flushDspStates() {
  memset(mainState, 0, sizeof(mainState));
  memset(geqState, 0, sizeof(geqState));
  memset(dlmsState, 0, sizeof(dlmsState));
  memset(dynState, 0, sizeof(dynState));
  memset(dynEnv, 0, sizeof(dynEnv));
  memset(dlyBufL, 0, sizeof(dlyBufL)); memset(dlyBufR, 0, sizeof(dlyBufR));
  memset(inDlyBufA, 0, sizeof(inDlyBufA)); memset(inDlyBufB, 0, sizeof(inDlyBufB));
  gateEnv = 0; gateGain = 1; compEnv = 0; xfLpL = xfLpR = 0;
  dcxL = dcyL = dcxR = dcyR = 0;
  statFlush++;
}

static void processBlock(int16_t *s, int frames) {
  if (stateFlushNeeded) { stateFlushNeeded = false; flushDspStates(); }
  BiquadCoef *mb = mainBank[activeBank];
  BiquadCoef *gb = geqBank[activeBank];

  bool dynMoved = false;
  if (params.eqMode == 0 && !params.bypass) {
    for (int i = 0; i < NUM_BANDS; i++) {
      if (!params.band[i].dynOn || !params.band[i].on) { if (dynEffGain[i] != 0) { dynEffGain[i] = 0; dynMoved = true; } continue; }
      float envDb = 20.0f * log10f(dynEnv[i] + 1e-7f);
      float over = constrain((envDb - params.band[i].dynThrDb) / 12.0f, 0.0f, 1.0f);
      float target = params.band[i].dynRangeDb * over;
      if (fabsf(target - dynEffGain[i]) > 0.25f) { dynEffGain[i] = target; dynMoved = true; }
    }
  }
  (void)dynMoved; // gerakan dynEQ kini mengalir lewat glide gain terhalus di loop sampel
  if (coefDirty) { rebuildCoefficients(); mb = mainBank[activeBank]; gb = geqBank[activeBank]; }

  float targetPreamp = db2lin(params.preampDb);
  if (params.autoHeadroom) {
    float boost = 0;
    if (params.eqMode == 0) { for (int i = 0; i < NUM_BANDS; i++) if (params.band[i].on && bandEffGain(i) > 0) boost += bandEffGain(i) * 0.5f; }
    else { for (int i = 0; i < NUM_GEQ; i++) if (params.geq[i] > 0) boost += params.geq[i] * 0.25f; }
    targetPreamp *= db2lin(-fmaxf(0.0f, boost - 1.0f));
  }
  float targetTrimL = db2lin(params.trimLDb) * (params.polL ? -1 : 1);
  float targetTrimR = db2lin(params.trimRDb) * (params.polR ? -1 : 1);
  float targetCeil = db2lin(params.ceilingDb);
  float targetVol = params.mute ? 0.0f : fminf(params.volume, params.volMax) * (params.dim ? db2lin(-20.0f) : 1.0f);
  float gateThr = db2lin(params.gateThrDb);
  float gAtk = msCoef(params.gateAttackMs), gRel = msCoef(params.gateReleaseMs);
  float cAtk = msCoef(params.compAttackMs), cRel = msCoef(params.compReleaseMs);
  float makeup = db2lin(params.compMakeupDb);
  float xfCoef = 1.0f - expf(-2.0f * PI * 700.0f / SAMPLE_RATE);
  float targetXfAmt = params.crossfeedAmt;
  float targetWidth = params.monoCheck ? 0.0f : params.width;
  float targetBal = params.balance;
  float targetGateThr = gateThr, targetCompThrDb = params.compThrDb, targetCompRatio = params.compRatio, targetMakeup = makeup;
  float targetDlyL = params.delayLms;
  float targetDlyR = params.delayRms;
  float sumL = 0, sumR = 0;
  bool dlmsLive = dlms.on || dlmsTouched; // gerbang kerja DLMS di task audio
  // Target penghalus DLMS dihitung SEKALI per blok — db2lin() adalah powf() yang mahal;
  // memanggilnya per sampel (v5.0-v5.0.7) membuat blok DLMS 11,1 ms > anggaran 10,67 ms.
  float tgtInGainA = dlms.inOnA ? db2lin(dlms.inGainA) : 0.0f;
  float tgtInGainB = dlms.inOnB ? db2lin(dlms.inGainB) : 0.0f;
  float tgtChGain[2], tgtChCeil[2];
  for (int c = 0; c < 2; c++) {
    DlmsCh &hc = dlms.ch[c];
    tgtChGain[c] = hc.on ? db2lin(hc.gainDb) * (hc.pol ? -1.0f : 1.0f) : 0.0f;
    tgtChCeil[c] = db2lin(hc.limDb);
  }

  for (int n = 0; n < frames; n++) {
    smoothPreamp += (targetPreamp - smoothPreamp) * SMOOTH_COEF;
    smoothTrimL  += (targetTrimL  - smoothTrimL)  * SMOOTH_COEF;
    smoothTrimR  += (targetTrimR  - smoothTrimR)  * SMOOTH_COEF;
    smoothVol    += (targetVol    - smoothVol)     * SMOOTH_COEF;
    smoothXfAmt  += (targetXfAmt  - smoothXfAmt)  * SMOOTH_COEF;
    smoothWidth  += (targetWidth  - smoothWidth)   * SMOOTH_COEF;
    smoothBalance+= (targetBal    - smoothBalance) * SMOOTH_COEF;
    smoothDlyL   += (targetDlyL   - smoothDlyL)    * SMOOTH_COEF;
    smoothDlyR   += (targetDlyR   - smoothDlyR)    * SMOOTH_COEF;
    smoothCeil   += (targetCeil   - smoothCeil)    * SMOOTH_COEF;
    // glide parameter filter & dinamika (semua fader bergerak tanpa zipper)
    for (int i = 0; i < NUM_BANDS; i++) smBandGain[i] += ((params.band[i].gainDb + dynEffGain[i]) - smBandGain[i]) * 0.004f;
    for (int i = 0; i < NUM_GEQ; i++) smGeqGain[i] += (params.geq[i] - smGeqGain[i]) * 0.004f;
    smVolForLoud += (params.volume - smVolForLoud) * 0.002f;
    smGateThr += (targetGateThr - smGateThr) * SMOOTH_COEF;
    smCompThrDb += (targetCompThrDb - smCompThrDb) * SMOOTH_COEF;
    smCompRatio += (targetCompRatio - smCompRatio) * SMOOTH_COEF;
    smMakeup += (targetMakeup - smMakeup) * SMOOTH_COEF;
    if (dlmsLive) {
      smInGainA += (tgtInGainA - smInGainA) * SMOOTH_COEF;
      smInGainB += (tgtInGainB - smInGainB) * SMOOTH_COEF;
      smInDlyA += (dlms.inDlyA - smInDlyA) * SMOOTH_COEF;
      smInDlyB += (dlms.inDlyB - smInDlyB) * SMOOTH_COEF;
      for (int c = 0; c < 2; c++) {
        smChGain[c] += (tgtChGain[c] - smChGain[c]) * SMOOTH_COEF;
        smChCeil[c] += (tgtChCeil[c] - smChCeil[c]) * SMOOTH_COEF;
        smChDly[c] += (dlms.ch[c].dlyMs - smChDly[c]) * SMOOTH_COEF;
      }
    }
    // laju kontrol (tiap 48 sampel): luncurkan frekuensi/Q/HPF secara rasio, bangun ulang bila berubah
    if (++ctrlCount >= 48) {
      ctrlCount = 0;
      for (int i = 0; i < NUM_BANDS; i++) {
        smBandFreq[i] *= 1.0f + 0.09f * (params.band[i].freq / smBandFreq[i] - 1.0f);
        smBandQ[i] += (params.band[i].q - smBandQ[i]) * 0.09f;
      }
      smHpfFreq *= 1.0f + 0.09f * (params.hpfFreq / smHpfFreq - 1.0f);
      if (dlmsLive) {
        for (int c = 0; c < 2; c++) {
          smDlmsHpfF[c] *= 1.0f + 0.09f * (dlms.ch[c].hpfF / smDlmsHpfF[c] - 1.0f);
          smDlmsLpfF[c] *= 1.0f + 0.09f * (dlms.ch[c].lpfF / smDlmsLpfF[c] - 1.0f);
          for (int j = 0; j < 3; j++) smDlmsPeq[c][j] += (dlms.ch[c].peq[j] - smDlmsPeq[c][j]) * 0.09f;
        }
        if (dlmsChanged()) rebuildDlmsCoefs();
        if (!dlms.on) dlmsTouched = false;
      }
      if (dipDoSwap) {
        rebuildCoefficients();
        mb = mainBank[activeBank]; gb = geqBank[activeBank];
        dipDoSwap = false; dipPhase = 2; dipRemain = DIP_HALF;
      } else if (filtersChanged()) {
        if (armDip && dipPhase == 0) { dipPhase = 1; dipRemain = DIP_HALF; armDip = false; }
        else if (!armDip && dipPhase != 1 && dipPhase != 3) { rebuildCoefficients(); mb = mainBank[activeBank]; gb = geqBank[activeBank]; }
      }
    }

    float dipMul = 1.0f;
    if (dipPhase == 1) {
      float t = 1.0f - (float)dipRemain / (float)DIP_HALF;
      dipMul = 1.0f - t * t * (3.0f - 2.0f * t);
      if (--dipRemain == 0) { dipPhase = 3; dipDoSwap = true; }
    } else if (dipPhase == 3) {
      dipMul = 0.0f;
    } else if (dipPhase == 2) {
      float t = 1.0f - (float)dipRemain / (float)DIP_HALF;
      dipMul = t * t * (3.0f - 2.0f * t);
      if (--dipRemain == 0) dipPhase = 0;
    }

    float L = s[2 * n] * (1.0f / 32768.0f), R = s[2 * n + 1] * (1.0f / 32768.0f);
    if (dlms.on) {
      // --- jalur DLMS: input -> routing -> filter per kanal -> limiter kanal -> master ---
      grMeterDb = 0;
      float a = L * smInGainA, b = R * smInGainB;
      inDlyBufA[inDlyWr] = a; inDlyBufB[inDlyWr] = b;
      uint16_t dA = (uint16_t)(smInDlyA * 48.0f), dB = (uint16_t)(smInDlyB * 48.0f);
      a = dlyRead(inDlyBufA, inDlyWr, tapInA, dA, 4800);
      b = dlyRead(inDlyBufB, inDlyWr, tapInB, dB, 4800);
      inDlyWr = (inDlyWr + 1) % 4800;
      float xs[2];
      for (int c = 0; c < 2; c++) {
        DlmsCh &h = dlms.ch[c];
        float x = h.src == 0 ? a : (h.src == 1 ? b : (a + b) * 0.5f);
        for (int i = 0; i < 11; i++) x = runBiquad(dlmsCoef[c][i], dlmsState[c][i], x, true);
        xs[c] = x * smChGain[c];
      }
      uint16_t dd0 = (uint16_t)(smChDly[0] * 48.0f), dd1 = (uint16_t)(smChDly[1] * 48.0f);
      dlyBufL[dlyWr] = xs[0]; dlyBufR[dlyWr] = xs[1];
      L = dlyRead(dlyBufL, dlyWr, tapCh0, dd0, 4096); R = dlyRead(dlyBufR, dlyWr, tapCh1, dd1, 4096);
      dlyWr = (dlyWr + 1) & 4095;
      // Limiter kanal DLMS = PEAK LIMITER beneran (gain-riding) sejak
      // v5.2.2. Dulu saturator tanh mentah c*tanhf(x/c) TANPA oversampling:
      // gelombang diukir ulang, harmoniknya aliasing jadi bunyi NOISE —
      // makin rendah ambang, makin dalam saturasi, makin noise (laporan
      // user 2026-11-13 19:57). Apalagi limiter ini duduk SEBELUM volume
      // master dan memakan sinyal yang sudah di-boost gain kanal. Peak
      // limiter hanya menurunkan gain sesaat bila puncak melewati ambang:
      // bentuk gelombang utuh, tanpa harmonik baru. Per sampel hanya
      // fabsf + mul-add (tanpa transendental), koefisien konstanta:
      // detektor attack ~0,1 ms / release ~120 ms; gain turun cepat,
      // naik pelan ~60 ms.
      {
        float xs2[2] = {L, R};
        for (int c = 0; c < 2; c++) {
          float det = fabsf(xs2[c]);
          float e = limEnv[c];
          e += (det - e) * (det > e ? 0.19f : 0.000174f);
          limEnv[c] = e;
          float g = e > smChCeil[c] ? smChCeil[c] / e : 1.0f;
          float gs = limGainSm[c];
          gs += (g - gs) * (g < gs ? 0.5f : 0.000347f);
          limGainSm[c] = gs;
          xs2[c] *= gs;
        }
        L = xs2[0]; R = xs2[1];
      }
      L *= dipMul; R *= dipMul;
      L *= smoothVol; R *= smoothVol;
    } else {
    L *= smoothPreamp * smoothTrimL; R *= smoothPreamp * smoothTrimR;

    if (params.eqMode == 0) {
      for (int i = 0; i < NUM_BANDS; i++) {
        if (!params.band[i].dynOn) continue;
        float d = runBiquad(dynCoef[i], dynState[i], (L + R) * 0.5f, true);
        float a = fabsf(d);
        dynEnv[i] += (a - dynEnv[i]) * (a > dynEnv[i] ? 0.06f : 0.004f);
      }
    }

    for (int i = 0; i < 4; i++) { L = runBiquad(mb[i], mainState[i], L, true); R = runBiquad(mb[i], mainState[i], R, false); }
    if (params.eqMode == 0) {
      for (int i = 0; i < NUM_BANDS; i++) { L = runBiquad(mb[4 + i], mainState[4 + i], L, true); R = runBiquad(mb[4 + i], mainState[4 + i], R, false); }
    } else {
      for (int i = 0; i < NUM_GEQ; i++) { L = runBiquad(gb[i], geqState[i], L, true); R = runBiquad(gb[i], geqState[i], R, false); }
    }
    L = runBiquad(mb[14], mainState[14], L, true); R = runBiquad(mb[14], mainState[14], R, false);
    L = runBiquad(mb[15], mainState[15], L, true); R = runBiquad(mb[15], mainState[15], R, false);

    L *= dipMul; R *= dipMul;

    float det = fmaxf(fabsf(L), fabsf(R));
    if (params.gateOn) {
      gateEnv += (det - gateEnv) * (det > gateEnv ? gAtk : gRel);
      float tgt = gateEnv > smGateThr ? 1.0f : 0.02f;
      gateGain += (tgt - gateGain) * (tgt > gateGain ? gAtk : gRel * 0.5f);
      L *= gateGain; R *= gateGain;
    }
    float gr = 1.0f;
    if (params.compOn) {
      compEnv += (det - compEnv) * (det > compEnv ? cAtk : cRel);
      float envDb = 20.0f * log10f(compEnv + 1e-7f);
      float over = envDb - smCompThrDb;
      float grDb = over > 0 ? -over * (1.0f - 1.0f / fmaxf(smCompRatio, 1.0f)) : 0.0f;
      grMeterDb = grDb;
      gr = db2lin(grDb) * smMakeup;
      L *= gr; R *= gr;
    } else grMeterDb = 0;

    if (params.crossfeedOn) {
      xfLpL += (L - xfLpL) * xfCoef; xfLpR += (R - xfLpR) * xfCoef;
      float nL = L + xfLpR * smoothXfAmt, nR = R + xfLpL * smoothXfAmt;
      L = nL; R = nR;
    }
    if (smoothWidth != 1.0f) {
      float M = (L + R) * 0.5f, S = (L - R) * 0.5f;
      L = M + S * smoothWidth; R = M - S * smoothWidth;
    }
    if (smoothBalance < -0.001f) R *= 1.0f + smoothBalance;
    else if (smoothBalance > 0.001f) L *= 1.0f - smoothBalance;

    uint16_t dlyL = (uint16_t)(smoothDlyL * 48.0f), dlyR = (uint16_t)(smoothDlyR * 48.0f);
    dlyBufL[dlyWr] = L; dlyBufR[dlyWr] = R;
    L = dlyRead(dlyBufL, dlyWr, tapRegL, dlyL, 4096); R = dlyRead(dlyBufR, dlyWr, tapRegR, dlyR, 4096);
    dlyWr = (dlyWr + 1) & 4095;

    L *= smoothVol; R *= smoothVol;
    if (params.limiterOn) {
      L = smoothCeil * tanhf(L / (smoothCeil + 1e-9f));
      R = smoothCeil * tanhf(R / (smoothCeil + 1e-9f));
    }
    } // akhir cabang reguler

    float aL = fabsf(L), aR = fabsf(R);
    if (aL > meterPeakL) meterPeakL = aL;
    if (aR > meterPeakR) meterPeakR = aR;
    sumL += L * L; sumR += R * R;
    if (aL >= 0.999f || aR >= 0.999f) clipCount++;
    rtaBuf[rtaWr] = (L + R) * 0.5f; rtaWr = (rtaWr + 1) & 1023;

    // sanitasi akhir: NaN/Inf -> nol + minta flush state; penahan DC ~3 Hz;
    // jepit keras; ramp boot 50 ms. Prinsipnya: gagal = diam.
    float oL = L, oR = R;
    if (!isfinite(oL) || !isfinite(oR)) { oL = 0.0f; oR = 0.0f; stateFlushNeeded = true; }
    float yL = oL - dcxL + 0.99961f * dcyL; dcxL = oL; dcyL = yL; oL = yL;
    float yR = oR - dcxR + 0.99961f * dcyR; dcxR = oR; dcyR = yR; oR = yR;
    oL = constrain(oL, -1.0f, 1.0f); oR = constrain(oR, -1.0f, 1.0f);
    float bootMul = 1.0f;
    if (bootRamp < 2400) bootMul = (float)(bootRamp++) * (1.0f / 2400.0f);
    dspOut[2 * n] = quant24(oL * bootMul);
    dspOut[2 * n + 1] = quant24(oR * bootMul);
  }
  if (frames > 0) {
    meterRmsL = sqrtf(sumL / frames); meterRmsR = sqrtf(sumR / frames);
  }
}

static void onSpkData(void *data, uint16_t len) {
  lastSpkMs = millis();
  uac.applyVolume(data, len);
  int16_t *s = (int16_t *)data;
  int frames = len / 4;
  // dorong input mentah ke ringIn (buang + hitung bila penuh)
  uint32_t w = ringInW;
  for (int n = 0; n < frames; n++) {
    if ((w - ringInR) >= RING_FRAMES - 1) { statOverflow++; break; }
    uint32_t idx = (w++) & (RING_FRAMES - 1);
    ringIn[idx * 2] = s[n * 2]; ringIn[idx * 2 + 1] = s[n * 2 + 1];
  }
  ringInW = w;
  // tarik hasil olahan 24-bit dari ringOut ke buffer kirim I2S (kata 32-bit)
  static int32_t i2sTx[1024 * 2];
  int txFrames = frames > 1024 ? 1024 : frames;
  uint32_t r = ringOutR;
  uint32_t avail = ringOutW - r;
  for (int n = 0; n < txFrames; n++) {
    if (n < (int)avail) {
      uint32_t idx = (r + n) & (RING_FRAMES - 1);
      i2sTx[n * 2] = ringOut[idx * 2]; i2sTx[n * 2 + 1] = ringOut[idx * 2 + 1];
    } else { i2sTx[n * 2] = 0; i2sTx[n * 2 + 1] = 0; statUnderrun++; }
  }
  ringOutR = r + (txFrames < (int)avail ? txFrames : (int)avail);
  if (dspTaskHandle && (ringInW - ringInR) >= 64) xTaskNotifyGive(dspTaskHandle);
  i2s.write((uint8_t *)i2sTx, txFrames * 8);
}

// Uji mandiri diagnostik (serial: dlmstest=1) — ukur waktu proses per fase sinyal.
// Fase hening/meluruh memaksa state biquad ke rentang denormal: bila fase itu jauh
// lebih lambat dari fase keras, itulah penyebab serak (FPU melambat pada denormal).
static int16_t dlmsTb[512 * 2];
static void dlmsFillSine(float amp) {
  for (int i = 0; i < 512; i++) {
    float v = amp * sinf(2.0f * PI * 440.0f * i / 48000.0f);
    int16_t q = (int16_t)(v * 32767.0f);
    dlmsTb[2 * i] = q; dlmsTb[2 * i + 1] = (int16_t)(q / 2);
  }
}
static void dlmsRunPhase(const char *nm, int blocks) {
  uint32_t t0 = micros();
  for (int r = 0; r < blocks; r++) processBlock(dlmsTb, 512);
  Serial0.printf("[test] %s: %lu us / %d blok\n", nm, (unsigned long)(micros() - t0), blocks);
}
static void dlmsSelfTest() {
  Serial0.println("[test] mulai: ukur waktu DLMS vs reguler (setelan tersimpan)");
  bool wasOn = dlms.on;
  dlms.on = true; dlmsTouched = true;
  dlmsFillSine(0.5f);  dlmsRunPhase("DLMS keras ", 100);
  float amp = 0.5f;
  uint32_t t0 = micros();
  for (int r = 0; r < 100; r++) { dlmsFillSine(amp); processBlock(dlmsTb, 512); amp *= 0.85f; }
  Serial0.printf("[test] DLMS luruh : %lu us / 100 blok\n", (unsigned long)(micros() - t0));
  dlmsFillSine(0.0f);  dlmsRunPhase("DLMS hening", 200);
  dlms.on = false;
  dlmsFillSine(0.5f);  dlmsRunPhase("REG  keras ", 100);
  dlmsFillSine(0.0f);  dlmsRunPhase("REG  hening", 200);
  dlms.on = wasOn;
  Serial0.println("[test] SELESAI");
}

static void dspTask(void *arg) {
  (void)arg;
  for (;;) {
    dspHeartbeat++;
    uint32_t avail = ringInW - ringInR;
    if (avail < 64) {
      ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(2));
      avail = ringInW - ringInR;
      if (!avail) continue;
    }
    int n = avail > 512 ? 512 : (int)avail;
    uint32_t r = ringInR;
    for (int i = 0; i < n; i++) {
      uint32_t idx = (r + i) & (RING_FRAMES - 1);
      dspBlock[i * 2] = ringIn[idx * 2]; dspBlock[i * 2 + 1] = ringIn[idx * 2 + 1];
    }
    ringInR = r + n;
    processBlock(dspBlock, n);
    uint32_t w = ringOutW;
    uint32_t space = RING_FRAMES - 1 - (w - ringOutR);
    int m = n < (int)space ? n : (int)space;
    for (int i = 0; i < m; i++) {
      uint32_t idx = (w + i) & (RING_FRAMES - 1);
      ringOut[idx * 2] = dspOut[i * 2]; ringOut[idx * 2 + 1] = dspOut[i * 2 + 1];
    }
    ringOutW = w + m;
  }
}


static void setupAudio() {
  i2s.setPins(PIN_BCK, PIN_LRCK, PIN_DIN, -1);
  // v5.1: slot 32-bit membawa data 24-bit rata-kiri + dither (lihat quant24).
  i2s.begin(I2S_MODE_STD, 48000, I2S_DATA_BIT_WIDTH_32BIT, I2S_SLOT_MODE_STEREO, I2S_STD_SLOT_BOTH);
  // Identitas USB sendiri (nama tampil di Windows sebagai perangkat audio).
  // WAJIB tanpa CDC (CDCOnBoot Disabled di menu IDE) — lihat catatan di atas.
  USB.productName("MbahDAC USB");
  USB.manufacturerName("mbah");
  USB.PID(0xDA01);
  USB.serialNumber("MBDAC-001");
  uac.onData(onSpkData);
  uac.begin();
  USB.begin();
}

// ============================================================
//  RTA — FFT 1024 titik (radix-2 iteratif), 31 band 1/3 oktaf
// ============================================================
static float rtaLevels[NUM_GEQ];
static void computeRTA() {
  static float re[1024], im[1024];
  static uint32_t lastWr = 0xFFFFFFFF;
  bool rtaStale = (rtaWr == lastWr); // buffer tidak maju = audio berhenti -> bar harus jatuh, bukan membeku
  lastWr = rtaWr;
  for (int i = 0; i < 1024; i++) {
    float v = rtaBuf[(rtaWr + i) & 1023];
    float win = 0.5f - 0.5f * cosf(2.0f * PI * i / 1023.0f);
    re[i] = v * win; im[i] = 0;
  }
  for (int i = 1, j = 0; i < 1024; i++) {
    int bit = 512; for (; j & bit; bit >>= 1) j ^= bit; j ^= bit;
    if (i < j) { float t; t = re[i]; re[i] = re[j]; re[j] = t; t = im[i]; im[i] = im[j]; im[j] = t; }
  }
  for (int len = 2; len <= 1024; len <<= 1) {
    float ang = -2.0f * PI / len;
    float wr = cosf(ang), wi = sinf(ang);
    for (int i = 0; i < 1024; i += len) {
      float cwr = 1, cwi = 0;
      for (int k = 0; k < len / 2; k++) {
        int u = i + k, v = i + k + len / 2;
        float vr = re[v] * cwr - im[v] * cwi, vi = re[v] * cwi + im[v] * cwr;
        re[v] = re[u] - vr; im[v] = im[u] - vi;
        re[u] += vr; im[u] += vi;
        float nwr = cwr * wr - cwi * wi; cwi = cwr * wi + cwi * wr; cwr = nwr;
      }
    }
  }
  for (int b = 0; b < NUM_GEQ; b++) {
    float fLo = GEQ_FREQ[b] * 0.89f, fHi = GEQ_FREQ[b] * 1.12f;
    int kLo = (int)(fLo * 1024 / SAMPLE_RATE), kHi = (int)(fHi * 1024 / SAMPLE_RATE);
    if (kLo < 1) kLo = 1; if (kHi <= kLo) kHi = kLo + 1; if (kHi > 511) kHi = 511;
    float peak = 0;
    for (int k = kLo; k < kHi; k++) { float m = sqrtf(re[k] * re[k] + im[k] * im[k]); if (m > peak) peak = m; }
    float db = 20.0f * log10f(peak / 256.0f + 1e-6f);
    float newLevel = rtaStale ? -60.0f : constrain(db, -60.0f, 0.0f);
    // balistik analyzer: serangan seketika, jatuh +-22 dB/detik (per hitungan 100 ms)
    if (newLevel > rtaSmooth[b]) rtaSmooth[b] = newLevel;
    else { rtaSmooth[b] -= 2.2f; if (rtaSmooth[b] < newLevel) rtaSmooth[b] = newLevel; }
    rtaLevels[b] = rtaSmooth[b];
  }
}

// ============================================================
//  Penyimpanan preset user di NVS (10 slot)
// ============================================================
Preferences prefs;
static bool saveUserPreset(int slot) {
  if (slot < 0 || slot >= NUM_USER_PRESETS) return false;
  char key[8]; snprintf(key, sizeof(key), "u%d", slot);
  char dkey[8]; snprintf(dkey, sizeof(dkey), "d%d", slot);
  prefs.begin("mbahdac2", false);
  size_t w = prefs.putBytes(key, &params, sizeof(params));
  prefs.putBytes(dkey, &dlms, sizeof(dlms));
  prefs.end();
  return w == sizeof(params);
}
static bool loadUserPreset(int slot) {
  if (slot < 0 || slot >= NUM_USER_PRESETS) return false;
  char key[8]; snprintf(key, sizeof(key), "u%d", slot);
  prefs.begin("mbahdac2", true);
  DspParams tmp;
  size_t r = prefs.getBytes(key, &tmp, sizeof(tmp));
  prefs.end();
  if (r != sizeof(tmp)) return false;
  params = tmp; coefDirty = true; rebuildDynCoefs();
  char dkey[8]; snprintf(dkey, sizeof(dkey), "d%d", slot);
  prefs.begin("mbahdac2", true);
  DlmsParams dtmp;
  size_t dr = prefs.getBytes(dkey, &dtmp, sizeof(dtmp));
  prefs.end();
  if (dr == sizeof(dtmp)) { dlms = dtmp; dlms.on = false; snapDlmsSmooth(); rebuildDlmsCoefs(); }
  return true;
}

static void saveDlmsNvs() {
  prefs.begin("mbahdac2", false);
  prefs.putBytes("dlms2", &dlms, sizeof(dlms));
  prefs.end();
}
static void loadDlmsNvs() {
  prefs.begin("mbahdac2", true);
  DlmsParams tmp;
  size_t r = prefs.getBytes("dlms2", &tmp, sizeof(tmp));
  prefs.end();
  if (r == sizeof(tmp)) dlms = tmp;
}
static void saveStateNvs() {
  prefs.begin("mbahdac2", false);
  prefs.putBytes("state", &params, sizeof(params));
  prefs.putBytes("dlms2", &dlms, sizeof(dlms));
  prefs.end();
  stateDirty = false; dlmsSaveDirty = false;
}
static bool loadStateNvs() {
  prefs.begin("mbahdac2", true);
  DspParams tmp;
  size_t r = prefs.getBytes("state", &tmp, sizeof(tmp));
  prefs.end();
  if (r != sizeof(tmp)) return false;
  params = tmp;
  return true;
}
static void loadSysPrefs() {
  prefs.begin("mbahdac2", true);
  sysAutoSave = prefs.getBool("autosave", true);
  prefs.end();
}
static void softResetNow() {
  setFlat(params);
  setDlmsDefaults(); dlms.on = false;
  snapSmoothToParams(); snapDlmsSmooth();
  rebuildCoefficients(); rebuildDlmsCoefs(); rebuildDynCoefs();
  coefDirty = true;
  saveStateNvs();
}
static void factoryResetNow() {
  // Hapus seluruh state audio + preset; kredensial WiFi panel DIPERTAHANKAN
  // (identitas jaringan bukan urusan reset setelan; fallback-nya header bila kosong).
  String ssid, pass;
  prefs.begin("mbahdac2", true);
  ssid = prefs.getString("ssid", ""); pass = prefs.getString("pass", "");
  prefs.end();
  prefs.begin("mbahdac2", false);
  prefs.clear();
  if (ssid.length()) { prefs.putString("ssid", ssid); prefs.putString("pass", pass); }
  prefs.end();
  delay(300);
  ESP.restart();
}
static void bootBtnPoll() {
  // Reset pabrik fisik: tahan BOOT (GPIO0) >= 2,5 detik dalam 6 detik pertama setelah boot.
  static uint32_t lowSince = 0;
  if (millis() > 6000) return;
  if (digitalRead(0) == LOW) {
    if (!lowSince) lowSince = millis();
    else if (millis() - lowSince > 2500) factoryResetNow();
  } else lowSince = 0;
}
static bool userPresetExists(int slot) {
  char key[8]; snprintf(key, sizeof(key), "u%d", slot);
  prefs.begin("mbahdac2", true);
  bool ok = prefs.isKey(key);
  prefs.end();
  return ok;
}

// ============================================================
//  WiFi (STA — always on)
// ============================================================
WebServer server(80);
static String wifiSSID, wifiPass;
static bool wifiOn = false, mdnsOn = false;

static void loadWifiCreds() {
  prefs.begin("mbahdac2", true);
  wifiSSID = prefs.getString("ssid", WIFI_SSID);
  wifiPass = prefs.getString("pass", WIFI_PASS);
  prefs.end();
}
static void wifiStart() {
  if (wifiOn) return;
  WiFi.mode(WIFI_STA);
  WiFi.setHostname(DAC_HOSTNAME);
  WiFi.setAutoReconnect(true);
  WiFi.begin(wifiSSID.c_str(), wifiPass.c_str());
  wifiOn = true;
}
static void wifiPoll() {
  static bool wasConnected = false;
  if (!wifiOn) return;
  if (WiFi.status() == WL_CONNECTED) {
    if (!wasConnected) {
      wasConnected = true;
      Serial0.printf("[WiFi] Tersambung: %s — panel di http://%s.local\n", WiFi.localIP().toString().c_str(), DAC_HOSTNAME);
      server.enableCORS(true);  // aplikasi web (Flutter Web) boleh membaca API dari origin lain
      server.begin();
      if (MDNS.begin(DAC_HOSTNAME)) mdnsOn = true;
    }
    server.handleClient();
  } else wasConnected = false;
}

// ============================================================
//  API web
// ============================================================
static String dlmsJson() {
  String j = ",\"dlms\":{\"on\":" + String(dlms.on ? 1 : 0);
  j += ",\"inOnA\":" + String(dlms.inOnA ? 1 : 0) + ",\"inOnB\":" + String(dlms.inOnB ? 1 : 0);
  j += ",\"inGainA\":" + String(dlms.inGainA, 1) + ",\"inGainB\":" + String(dlms.inGainB, 1);
  j += ",\"inDlyA\":" + String(dlms.inDlyA, 1) + ",\"inDlyB\":" + String(dlms.inDlyB, 1);
  j += ",\"ch\":[";
  for (int c = 0; c < 2; c++) {
    DlmsCh &h = dlms.ch[c];
    if (c) j += ",";
    j += "{\"on\":" + String(h.on ? 1 : 0) + ",\"src\":" + String(h.src) +
         ",\"hpfOn\":" + String(h.hpfOn ? 1 : 0) + ",\"hpfF\":" + String(h.hpfF, 0) +
         ",\"lpfOn\":" + String(h.lpfOn ? 1 : 0) + ",\"lpfF\":" + String(h.lpfF, 0) +
         ",\"tf\":" + String(h.tf) + ",\"peqOn\":" + String(h.peqOn ? 1 : 0) +
         ",\"peq\":[" + String(h.peq[0], 1) + "," + String(h.peq[1], 1) + "," + String(h.peq[2], 1) + "]" +
         ",\"gain\":" + String(h.gainDb, 1) + ",\"dly\":" + String(h.dlyMs, 1) +
         ",\"lim\":" + String(h.limDb, 1) + ",\"pol\":" + String(h.pol ? 1 : 0) + "}";
  }
  j += "]}";
  return j;
}

static String stateJson() {
  String j = "{";
  j += "\"fw\":\"" FW_VERSION "\",\"net\":\"" + String(wifiOn && WiFi.status() == WL_CONNECTED ? ("WiFi: " + WiFi.localIP().toString()) : "WiFi: Menghubungkan...") + "\"";
  j += ",\"ur\":" + String(statUnderrun) + ",\"ov\":" + String(statOverflow);
  j += ",\"preamp\":" + String(params.preampDb, 1);
  j += ",\"trimL\":" + String(params.trimLDb, 1) + ",\"trimR\":" + String(params.trimRDb, 1);
  j += ",\"polL\":" + String(params.polL ? 1 : 0) + ",\"polR\":" + String(params.polR ? 1 : 0);
  j += ",\"hpfOn\":" + String(params.hpfOn ? 1 : 0) + ",\"hpfFreq\":" + String(params.hpfFreq, 0) + ",\"hpfSlope\":" + String(params.hpfSlope);
  j += ",\"eqMode\":" + String(params.eqMode);
  j += ",\"bands\":[";
  for (int i = 0; i < NUM_BANDS; i++) {
    Band &b = params.band[i];
    if (i) j += ",";
    j += "{\"on\":" + String(b.on ? 1 : 0) + ",\"t\":" + String(b.type) + ",\"f\":" + String(b.freq, 0) +
         ",\"g\":" + String(b.gainDb, 1) + ",\"q\":" + String(b.q, 2) +
         ",\"dyn\":" + String(b.dynOn ? 1 : 0) + ",\"dthr\":" + String(b.dynThrDb, 0) + ",\"drng\":" + String(b.dynRangeDb, 1) + "}";
  }
  j += "],\"geq\":[";
  for (int i = 0; i < NUM_GEQ; i++) { if (i) j += ","; j += String(params.geq[i], 1); }
  j += "]";
  j += ",\"gateOn\":" + String(params.gateOn ? 1 : 0) + ",\"gateThr\":" + String(params.gateThrDb, 0) +
       ",\"gateAtk\":" + String(params.gateAttackMs, 0) + ",\"gateRel\":" + String(params.gateReleaseMs, 0);
  j += ",\"compOn\":" + String(params.compOn ? 1 : 0) + ",\"compThr\":" + String(params.compThrDb, 0) +
       ",\"compRatio\":" + String(params.compRatio, 1) + ",\"compAtk\":" + String(params.compAttackMs, 0) +
       ",\"compRel\":" + String(params.compReleaseMs, 0) + ",\"compMake\":" + String(params.compMakeupDb, 1);
  j += ",\"limiter\":" + String(params.limiterOn ? 1 : 0) + ",\"ceiling\":" + String(params.ceilingDb, 1);
  j += ",\"xfOn\":" + String(params.crossfeedOn ? 1 : 0) + ",\"xfAmt\":" + String(params.crossfeedAmt, 2);
  j += ",\"width\":" + String(params.width, 2) + ",\"mono\":" + String(params.monoCheck ? 1 : 0);
  j += ",\"balance\":" + String(params.balance, 2);
  j += ",\"dlyL\":" + String(params.delayLms, 2) + ",\"dlyR\":" + String(params.delayRms, 2);
  j += ",\"loud\":" + String(params.loudnessOn ? 1 : 0);
  j += ",\"vol\":" + String(params.volume, 3) + ",\"mute\":" + String(params.mute ? 1 : 0) +
       ",\"dim\":" + String(params.dim ? 1 : 0) + ",\"bypass\":" + String(params.bypass ? 1 : 0);
  j += ",\"volMax\":" + String(params.volMax, 2) + ",\"autoHr\":" + String(params.autoHeadroom ? 1 : 0);
  j += ",\"eqPeak\":" + String(estimateEqPeakDb(), 1);
  j += dlmsJson();
  j += ",\"dirty\":" + String((stateDirty || dlmsSaveDirty) ? 1 : 0) + ",\"autosave\":" + String(sysAutoSave ? 1 : 0);
  j += ",\"slots\":[";
  for (int i = 0; i < NUM_USER_PRESETS; i++) { if (i) j += ","; j += String(userPresetExists(i) ? 1 : 0); }
  j += "]}";
  return j;
}

// Halaman web disisipkan saat build (webpage.html -> PAGE)
static const char PAGE[] PROGMEM = R"HTML(<!DOCTYPE html>
<html lang="id"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>S3 Air — Halaman Darurat</title>
<style>
:root{color-scheme:dark}
body{margin:0;background:#17191d;color:#e8ecf0;font-family:system-ui,sans-serif;padding:18px}
.card{max-width:520px;margin:24px auto;background:#1d2026;border:1px solid #383e48;border-radius:10px;padding:20px}
h1{font-size:19px;margin:0 0 4px}
.dot{display:inline-block;width:9px;height:9px;border-radius:50%;background:#2ea05a;margin-right:7px}
.sub{color:#7ee2a8;font-size:13px;margin-bottom:14px}
p{color:#8a94a0;font-size:13px;line-height:1.5}
label{display:block;font-size:12px;color:#8a94a0;margin:12px 0 5px}
input{width:100%;box-sizing:border-box;background:#101216;border:1px solid #383e48;border-radius:6px;color:#e8ecf0;padding:10px;font-size:14px}
button{margin-top:16px;width:100%;background:#2ea05a;color:#08120c;border:0;border-radius:7px;padding:12px;font-size:14px;font-weight:700;cursor:pointer}
.kv{font-size:13px;margin:3px 0}.kv b{color:#6ed0e4}
#msg{font-size:13px;margin-top:12px;color:#f8cd37}
</style></head><body>
<div class="card">
<h1><span class="dot"></span>S3 Air</h1>
<div class="sub">Halaman darurat — setelan lengkap ada di aplikasi S3 Air</div>
<div id="info"><p>Memuat keadaan alat…</p></div>
<p>Halaman ini hanya untuk keadaan darurat (mis. aplikasi tidak tersedia) dan untuk memindahkan alat ke jaringan WiFi lain. Semua setelan suara diatur dari aplikasi S3 Air di Android.</p>
<form id="wf">
<label>Nama WiFi (SSID)</label>
<input id="ssid" autocomplete="off" required>
<label>Kata sandi WiFi</label>
<input id="pass" type="password">
<button type="submit">Simpan &amp; mulai ulang alat</button>
</form>
<div id="msg"></div>
</div>
<script>
fetch('/api/state').then(function(r){return r.json();}).then(function(s){
  document.getElementById('info').innerHTML =
   '<div class="kv">Firmware: <b>'+s.fw+'</b></div>'+
   '<div class="kv">Jaringan: <b>'+s.net+'</b></div>'+
   '<div class="kv">Host: <b>'+location.hostname+'</b></div>';
}).catch(function(){document.getElementById('info').innerHTML='<p>Tidak bisa membaca keadaan alat.</p>';});
document.getElementById('wf').addEventListener('submit',function(e){
  e.preventDefault();
  if(!confirm('Alat akan mulai ulang dan pindah ke jaringan baru. Lanjut?'))return;
  var b=new URLSearchParams();
  b.set('ssid',document.getElementById('ssid').value);
  b.set('pass',document.getElementById('pass').value);
  fetch('/api/wifi',{method:'POST',body:b}).then(function(){
    document.getElementById('msg').textContent='Tersimpan. Alat mulai ulang — sambungkan perangkat ini ke jaringan WiFi yang baru, lalu buka aplikasi S3 Air.';
  }).catch(function(){document.getElementById('msg').textContent='Gagal mengirim. Coba lagi.';});
});
</script>
</body></html>
)HTML";

// Terapkan daftar pasangan kunci=nilai ke parameter — dipakai bersama oleh
// HTTP /api/set dan perintah "set" Bluetooth (satu logika, dua pintu).
// preFactory dipanggil tepat sebelum reset pabrik berjalan agar pintu asal
// sempat mengirim jawaban; *didFactory = true bila reset pabrik dijalankan.
static void applySetList(const String names[], const String vals[], int n,
                         const std::function<void()> &preFactory, bool *didFactory) {
  auto has = [&](const char *k) { for (int i = 0; i < n; i++) if (names[i] == k) return true; return false; };
  auto get = [&](const char *k) -> String { for (int i = 0; i < n; i++) if (names[i] == k) return vals[i]; return String(""); };
  auto f = [&](const char *k) { return get(k).toFloat(); };
  auto on = [&](const char *k) { return get(k) == "1"; };
  bool coef = false, dyn = false;
  if (has("preamp")) params.preampDb = constrain(f("preamp"), -12, 6);
  if (has("trimL")) params.trimLDb = constrain(f("trimL"), -12, 12);
  if (has("trimR")) params.trimRDb = constrain(f("trimR"), -12, 12);
  if (has("polL")) params.polL = on("polL");
  if (has("polR")) params.polR = on("polR");
  if (has("hpfOn")) { params.hpfOn = on("hpfOn"); coef = true; }
  if (has("hpfFreq")) { params.hpfFreq = constrain(f("hpfFreq"), 20, 200); coef = true; }
  if (has("hpfSlope")) { params.hpfSlope = (uint8_t)constrain((int)f("hpfSlope"), 6, 24); coef = true; }
  if (has("eqMode")) { params.eqMode = (uint8_t)constrain((int)f("eqMode"), 0, 1); coef = true; armDip = true; }
  if (has("bypass")) { params.bypass = on("bypass"); coef = true; armDip = true; }
  if (has("loud")) { params.loudnessOn = on("loud"); coef = true; }
  for (int i = 0; i < NUM_BANDS; i++) {
    char k[16];
    snprintf(k, sizeof(k), "b%d_on", i); if (has(k)) { params.band[i].on = on(k); coef = true; armDip = true; }
    snprintf(k, sizeof(k), "b%d_t", i);  if (has(k)) { params.band[i].type = (uint8_t)constrain((int)f(k), 0, 4); coef = true; dyn = true; armDip = true; }
    snprintf(k, sizeof(k), "b%d_f", i);  if (has(k)) { params.band[i].freq = constrain(f(k), 20, 20000); coef = true; dyn = true; }
    snprintf(k, sizeof(k), "b%d_g", i);  if (has(k)) { params.band[i].gainDb = constrain(f(k), -15, 15); coef = true; }
    snprintf(k, sizeof(k), "b%d_q", i);  if (has(k)) { params.band[i].q = constrain(f(k), 0.4f, 16.0f); coef = true; dyn = true; }
    snprintf(k, sizeof(k), "b%d_dyn", i); if (has(k)) params.band[i].dynOn = on(k);
    snprintf(k, sizeof(k), "b%d_dthr", i); if (has(k)) params.band[i].dynThrDb = constrain(f(k), -60, 0);
    snprintf(k, sizeof(k), "b%d_drng", i); if (has(k)) params.band[i].dynRangeDb = constrain(f(k), -12, 12);
  }
  for (int i = 0; i < NUM_GEQ; i++) {
    char k[8]; snprintf(k, sizeof(k), "g%d", i);
    if (has(k)) { params.geq[i] = constrain(f(k), -12, 12); coef = true; }
  }
  if (has("gateOn")) params.gateOn = on("gateOn");
  if (has("gateThr")) params.gateThrDb = constrain(f("gateThr"), -80, -10);
  if (has("gateAtk")) params.gateAttackMs = constrain(f("gateAtk"), 0.1f, 200);
  if (has("gateRel")) params.gateReleaseMs = constrain(f("gateRel"), 10, 1000);
  if (has("compOn")) params.compOn = on("compOn");
  if (has("compThr")) params.compThrDb = constrain(f("compThr"), -48, 0);
  if (has("compRatio")) params.compRatio = constrain(f("compRatio"), 1, 20);
  if (has("compAtk")) params.compAttackMs = constrain(f("compAtk"), 0.1f, 300);
  if (has("compRel")) params.compReleaseMs = constrain(f("compRel"), 10, 1000);
  if (has("compMake")) params.compMakeupDb = constrain(f("compMake"), 0, 24);
  if (has("limiter")) params.limiterOn = on("limiter");
  if (has("ceiling")) params.ceilingDb = constrain(f("ceiling"), -6, 0);
  if (has("xfOn")) params.crossfeedOn = on("xfOn");
  if (has("xfAmt")) params.crossfeedAmt = constrain(f("xfAmt"), 0, 0.5f);
  if (has("width")) params.width = constrain(f("width"), 0, 2);
  if (has("mono")) params.monoCheck = on("mono");
  if (has("balance")) params.balance = constrain(f("balance"), -1, 1);
  if (has("dlyL")) params.delayLms = constrain(f("dlyL"), 0, 20);
  if (has("dlyR")) params.delayRms = constrain(f("dlyR"), 0, 20);
  if (has("vol")) { params.volume = constrain(f("vol"), 0, 1); } // loudness mengikuti volume terhalus (glide)
  if (has("mute")) params.mute = on("mute");
  if (has("dim")) params.dim = on("dim");
  if (has("volMax")) params.volMax = constrain(f("volMax"), 0.2f, 1.0f);
  if (has("autoHr")) params.autoHeadroom = on("autoHr");
  if (has("clipReset")) clipCount = 0;
  for (int a = 0; a < n; a++)
    dlmsSetKey(names[a], vals[a].toFloat(), vals[a] == "1");
  {
    bool touched = false;
    for (int a = 0; a < n; a++) {
      String an = names[a];
      if (an != "saveState" && an != "autosave" && an != "resetSoft" && an != "resetFactory" && an != "clipReset") touched = true;
    }
    if (has("autosave")) { sysAutoSave = on("autosave"); prefs.begin("mbahdac2", false); prefs.putBool("autosave", sysAutoSave); prefs.end(); }
    if (has("saveState")) saveStateNvs();
    if (has("resetSoft")) softResetNow();
    if (touched) { stateDirty = true; lastChangeMs = millis(); }
    if (has("resetFactory")) { if (preFactory) preFactory(); if (didFactory) *didFactory = true; factoryResetNow(); return; }
  }
  if (coef) coefDirty = true;
  if (dyn) rebuildDynCoefs();
}

static String setNamesHttp[40], setValsHttp[40];  // konteks loop (HTTP)
static String setNamesBle[40], setValsBle[40];    // konteks loop (perintah BLE dirakit di loop)

static void setupWeb() {
  server.on("/", []() { server.send_P(200, "text/html", PAGE); });
  server.on("/api/state", []() { server.send(200, "application/json", stateJson()); });
  server.on("/api/meter", []() {
    String j = "{\"l\":" + String(meterPeakL, 3) + ",\"r\":" + String(meterPeakR, 3) +
               ",\"lrms\":" + String(meterRmsL, 3) + ",\"rrms\":" + String(meterRmsR, 3) +
               ",\"gr\":" + String(grMeterDb, 1) + ",\"clip\":" + String(clipCount) +
               ",\"dirty\":" + String((stateDirty || dlmsSaveDirty) ? 1 : 0) +
               ",\"flt\":" + String(statFlush);
    // v5.2.4 (optimalisasi polling): RTA ikut dalam respons meter — aplikasi
    // cukup SATU panggilan 150 ms untuk meter + spektrum (dulu dua panggilan
    // 200+150 ms bersaing di server satu-utas). rtaLevels memang selalu
    // terpelihara oleh computeRTA(), jadi ini nyaris tanpa biaya.
    j += ",\"rta\":[";
    for (int i = 0; i < NUM_GEQ; i++) { if (i) j += ","; j += String(rtaLevels[i], 0); }
    j += "]}";
    meterPeakL = meterPeakR = 0;
    server.send(200, "application/json", j);
  });
  server.on("/api/rta", []() {
    String j = "{\"rta\":[";
    for (int i = 0; i < NUM_GEQ; i++) { if (i) j += ","; j += String(rtaLevels[i], 0); }
    j += "]}";
    server.send(200, "application/json", j);
  });
  server.on("/api/set", HTTP_POST, []() {
    int n = 0;
    for (int a = 0; a < server.args() && n < 40; a++) { setNamesHttp[n] = server.argName(a); setValsHttp[n] = server.arg(a); n++; }
    bool didFactory = false;
    applySetList(setNamesHttp, setValsHttp, n, []() { server.send(200, "application/json", "{\"ok\":1}"); }, &didFactory);
    if (!didFactory) server.send(200, "application/json", "{\"ok\":1}");
  });
  server.on("/api/preset", HTTP_POST, []() {
    String n = server.arg("name");
    bool ok = false;
    if (n.startsWith("user:")) ok = loadUserPreset(n.substring(5).toInt());
    else ok = setFactoryPreset(n.c_str(), params);
    if (ok) { coefDirty = true; armDip = true; rebuildDynCoefs(); stateDirty = true; lastChangeMs = millis(); }
    server.send(200, "application/json", ok ? "{\"ok\":1}" : "{\"ok\":0}");
  });
  server.on("/api/save", HTTP_POST, []() {
    int slot = server.hasArg("slot") ? server.arg("slot").toInt() : 0;
    server.send(200, "application/json", saveUserPreset(slot) ? "{\"ok\":1}" : "{\"ok\":0}");
  });
  server.on("/api/wifi", HTTP_POST, []() {
    prefs.begin("mbahdac2", false);
    prefs.putString("ssid", server.arg("ssid"));
    prefs.putString("pass", server.arg("pass"));
    prefs.end();
    server.send(200, "text/plain", "OK, restart...");
    delay(300);
    ESP.restart();
  });
}



// ============================================================
//  Kontrol serial (Serial0) - kanal kedua sejajar panel web.
//  Satu perintah per baris; daftar lengkap di PANDUAN.md.
// ============================================================
static bool serSetOne(const String &kv) {
  int eq = kv.indexOf('=');
  if (eq < 1) return false;
  String k = kv.substring(0, eq);
  float v = kv.substring(eq + 1).toFloat();
  bool b = (v != 0);
  if (k.length() >= 4 && k[0] == 'b' && k[1] >= '0' && k[1] <= '9' && k.indexOf('_') > 0) {
    int i = k.substring(1, k.indexOf('_')).toInt();
    if (i < 0 || i >= NUM_BANDS) return false;
    String f = k.substring(k.indexOf('_') + 1);
    Band &bd = params.band[i];
    if (f == "f") bd.freq = constrain(v, 20.0f, 20000.0f);
    else if (f == "g") bd.gainDb = constrain(v, -15.0f, 15.0f);
    else if (f == "q") bd.q = constrain(v, 0.4f, 16.0f);
    else if (f == "t") { bd.type = (uint8_t)constrain((int)v, 0, 4); rebuildDynCoefs(); }
    else if (f == "on") bd.on = b;
    else if (f == "dyn") { bd.dynOn = b; rebuildDynCoefs(); }
    else if (f == "dthr") bd.dynThrDb = constrain(v, -60.0f, 0.0f);
    else if (f == "drange") bd.dynRangeDb = constrain(v, -12.0f, 12.0f);
    else return false;
    coefDirty = true; return true;
  }
  if (k.length() >= 2 && k[0] == 'g' && k[1] >= '0' && k[1] <= '9') {
    int i = k.substring(1).toInt();
    if (i < 0 || i >= NUM_GEQ) return false;
    params.geq[i] = constrain(v, -12.0f, 12.0f); coefDirty = true; return true;
  }
  if (k == "saveState") { saveStateNvs(); serActed = true; return true; }
  if (k == "autosave") { sysAutoSave = b; prefs.begin("mbahdac2", false); prefs.putBool("autosave", sysAutoSave); prefs.end(); serActed = true; return true; }
  if (k == "resetSoft") { softResetNow(); serActed = true; return true; }
  if (k == "resetFactory") { serActed = true; factoryResetNow(); return true; }
  if (k == "dlmstest") { dlmsSelfTest(); return true; }
  if (dlmsSetKey(k, v, b)) return true;
#define SK(nm, field, lo, hi) if (k == nm) { field = constrain(v, lo, hi); coefDirty = true; return true; }
#define BK(nm, field) if (k == nm) { field = b; coefDirty = true; return true; }
  SK("pre", params.preampDb, -24.0f, 24.0f)
  SK("trimL", params.trimLDb, -12.0f, 12.0f)
  SK("trimR", params.trimRDb, -12.0f, 12.0f)
  BK("polL", params.polL) BK("polR", params.polR)
  BK("hpfOn", params.hpfOn)
  SK("hpfF", params.hpfFreq, 20.0f, 200.0f)
  if (k == "hpfSlope") { params.hpfSlope = (uint8_t)constrain((int)v, 6, 24); coefDirty = true; return true; }
  if (k == "eqMode") { params.eqMode = (uint8_t)constrain((int)v, 0, 1); coefDirty = true; armDip = true; return true; }
  if (k == "bypass") { params.bypass = b; coefDirty = true; armDip = true; return true; }
  BK("gateOn", params.gateOn)
  SK("gateThr", params.gateThrDb, -80.0f, 0.0f)
  SK("gateAtk", params.gateAttackMs, 0.1f, 100.0f)
  SK("gateRel", params.gateReleaseMs, 10.0f, 1000.0f)
  BK("compOn", params.compOn)
  SK("compThr", params.compThrDb, -60.0f, 0.0f)
  SK("compRatio", params.compRatio, 1.0f, 20.0f)
  SK("compAtk", params.compAttackMs, 0.1f, 300.0f)
  SK("compRel", params.compReleaseMs, 10.0f, 1000.0f)
  SK("compMakeup", params.compMakeupDb, 0.0f, 24.0f)
  BK("limOn", params.limiterOn)
  SK("ceil", params.ceilingDb, -12.0f, 0.0f)
  BK("xfOn", params.crossfeedOn)
  SK("xfAmt", params.crossfeedAmt, 0.0f, 0.5f)
  SK("width", params.width, 0.0f, 2.0f)
  BK("mono", params.monoCheck)
  SK("bal", params.balance, -1.0f, 1.0f)
  SK("dlyL", params.delayLms, 0.0f, 20.0f)
  SK("dlyR", params.delayRms, 0.0f, 20.0f)
  BK("loudOn", params.loudnessOn)
  SK("vol", params.volume, 0.0f, 1.0f)
  SK("volMax", params.volMax, 0.1f, 1.0f)
  BK("dim", params.dim) BK("mute", params.mute) BK("autoHr", params.autoHeadroom)
#undef SK
#undef BK
  return false;
}

static String serLine;
static void serialPoll() {
  while (Serial0.available()) {
    char c = (char)Serial0.read();
    if (c == '\r') continue;
    if (c != '\n') { if (serLine.length() < 300) serLine += c; continue; }
    String ln = serLine; serLine = "";
    ln.trim();
    if (!ln.length()) continue;
    if (ln == "state") { Serial0.println(stateJson()); continue; }
    if (ln.startsWith("preset ")) {
      bool ok = setFactoryPreset(ln.substring(7).c_str(), params);
      if (ok) { coefDirty = true; armDip = true; rebuildDynCoefs(); stateDirty = true; lastChangeMs = millis(); }
      Serial0.println(ok ? "OK" : "ERR preset"); continue;
    }
    if (ln.startsWith("upreset ")) {
      bool ok = loadUserPreset(ln.substring(8).toInt());
      if (ok) { coefDirty = true; armDip = true; rebuildDynCoefs(); stateDirty = true; lastChangeMs = millis(); }
      Serial0.println(ok ? "OK" : "ERR upreset"); continue;
    }
    if (ln.startsWith("save ")) { Serial0.println(saveUserPreset(ln.substring(5).toInt()) ? "OK" : "ERR save"); continue; }
    if (ln.startsWith("set ")) ln = ln.substring(4);
    int okc = 0;
    while (ln.length()) {
      int sp = ln.indexOf(' ');
      String tok = sp < 0 ? ln : ln.substring(0, sp);
      if (serSetOne(tok)) okc++;
      if (sp < 0) break;
      ln = ln.substring(sp + 1); ln.trim();
    }
    Serial0.println(okc ? "OK" : "ERR");
    if (okc > 0 && !serActed) { stateDirty = true; lastChangeMs = millis(); }
    serActed = false;
  }
}


// ============================================================
//  Bluetooth LE (NimBLE) — jalur lapangan aplikasi S3 KONSOL (v1.1.0).
//  Perintah teks sebaris diakhiri '\n' ditulis aplikasi ke kar. CMD;
//  potongan byte perintah dirakit dari ring buffer di loop() (konteks
//  yang sama dengan serial & HTTP — callback NimBLE hanya menyalin byte).
//  Jawaban mengalir sebagai notifikasi kar. RESP, diakhiri satu notifikasi
//  berisi tepat satu byte '\n' (JSON jawaban tidak pernah mengandung '\n').
//  Meter + spektrum: paket biner 55 byte / 50 ms di kar. METER.
//  Keamanan: bonding wajib (kar. CMD/RESP terenkripsi). Ikatan BARU hanya
//  diterima selama jendela pairing terbuka: 60 dtk setelah boot, atau
//  60 dtk setelah tombol BOOT ditekan (di luar masa reset pabrik 6 dtk).
// ============================================================
#define BLE_SVC_UUID   "9e7319c0-3a6d-4f7a-b001-5a33a1c00001"
#define BLE_CMD_UUID   "9e7319c0-3a6d-4f7a-b001-5a33a1c00002"
#define BLE_RESP_UUID  "9e7319c0-3a6d-4f7a-b001-5a33a1c00003"
#define BLE_METER_UUID "9e7319c0-3a6d-4f7a-b001-5a33a1c00004"

static NimBLECharacteristic *bleRespChr = nullptr, *bleMeterChr = nullptr;
static volatile bool bleLink = false;      // klien tersambung
static volatile bool bleSecure = false;    // tautan terenkripsi + terikat
static volatile bool bleMeterSub = false;  // klien berlangganan METER
static uint16_t bleHandle = BLE_HS_CONN_HANDLE_NONE;
static uint16_t bleMtuNow = 23;
static int bleBondsAtConnect = 0;
static uint32_t blePairUntil = 0;          // akhir jendela pairing (millis)
static char bleRxRing[1024];
static volatile uint16_t bleRxHead = 0, bleRxTail = 0;

static void bleRxPush(uint8_t c) {
  uint16_t nx = (bleRxHead + 1) % (uint16_t)sizeof(bleRxRing);
  if (nx != bleRxTail) { bleRxRing[bleRxHead] = (char)c; bleRxHead = nx; }
}

static void bleRespond(const String &payload) {
  if (!bleLink || !bleRespChr) return;
  size_t chunk = bleMtuNow > 3 ? bleMtuNow - 3 : 20;
  size_t len = payload.length(), off = 0;
  while (off < len) {
    size_t n = (len - off > chunk) ? chunk : len - off;
    bleRespChr->notify((const uint8_t *)payload.c_str() + off, n, bleHandle);
    off += n;
  }
  const uint8_t term = '\n';
  bleRespChr->notify(&term, 1, bleHandle);   // penanda akhir jawaban
}

static void bleHandleLine(String ln) {
  ln.trim();
  if (!ln.length()) return;
  if (!bleSecure) { Serial0.println("[ble] perintah ditolak: tautan belum aman"); return; }
  if (ln == "ping") { bleRespond("pong"); return; }
  if (ln == "state") { bleRespond(stateJson()); return; }
  if (ln.startsWith("set ")) {
    String body = ln.substring(4);
    int n = 0;
    while (body.length() && n < 40) {
      int amp = body.indexOf('&');
      String pair = (amp < 0) ? body : body.substring(0, amp);
      int eq = pair.indexOf('=');
      if (eq > 0) { setNamesBle[n] = pair.substring(0, eq); setValsBle[n] = pair.substring(eq + 1); n++; }
      if (amp < 0) break;
      body = body.substring(amp + 1);
    }
    bool didFactory = false;
    applySetList(setNamesBle, setValsBle, n, []() { bleRespond("{\"ok\":1}"); }, &didFactory);
    if (!didFactory) bleRespond("{\"ok\":1}");
    return;
  }
  if (ln.startsWith("preset ")) {
    String nm = ln.substring(7); nm.trim();
    bool ok = nm.startsWith("user:") ? loadUserPreset(nm.substring(5).toInt())
                                     : setFactoryPreset(nm.c_str(), params);
    if (ok) { coefDirty = true; armDip = true; rebuildDynCoefs(); stateDirty = true; lastChangeMs = millis(); }
    bleRespond(ok ? "{\"ok\":1}" : "{\"ok\":0}");
    return;
  }
  if (ln.startsWith("save ")) {
    bleRespond(saveUserPreset(ln.substring(5).toInt()) ? "{\"ok\":1}" : "{\"ok\":0}");
    return;
  }
  if (ln.startsWith("wifi ")) {   // format: wifi <ssid>\t<pass>
    String rest = ln.substring(5);
    int tab = rest.indexOf('\t');
    String ssid = (tab < 0) ? rest : rest.substring(0, tab);
    String pass = (tab < 0) ? String("") : rest.substring(tab + 1);
    prefs.begin("mbahdac2", false);
    prefs.putString("ssid", ssid);
    prefs.putString("pass", pass);
    prefs.end();
    bleRespond("{\"ok\":1}");
    delay(300);
    ESP.restart();
  }
}

class BleSrvCb : public NimBLEServerCallbacks {
  void onConnect(NimBLEServer *srv, NimBLEConnInfo &info) override {
    bleLink = true; bleSecure = false; bleMeterSub = false;
    bleHandle = info.getConnHandle(); bleMtuNow = info.getMTU();
    bleBondsAtConnect = NimBLEDevice::getNumBonds();
    srv->updateConnParams(info.getConnHandle(), 12, 24, 0, 180);
    Serial0.println("[ble] klien tersambung");
  }
  void onDisconnect(NimBLEServer *srv, NimBLEConnInfo &info, int reason) override {
    (void)srv; (void)info; (void)reason;
    bleLink = false; bleSecure = false; bleMeterSub = false;
    bleHandle = BLE_HS_CONN_HANDLE_NONE;
    NimBLEDevice::startAdvertising();
    Serial0.println("[ble] klien terputus");
  }
  void onMTUChange(uint16_t mtu, NimBLEConnInfo &info) override {
    (void)info; bleMtuNow = mtu;
  }
  void onAuthenticationComplete(NimBLEConnInfo &info) override {
    if (!info.isEncrypted()) {
      NimBLEDevice::getServer()->disconnect(info.getConnHandle());
      Serial0.println("[ble] enkripsi gagal — klien diputus");
      return;
    }
    bool windowOpen = (int32_t)(blePairUntil - millis()) > 0;
    if (NimBLEDevice::getNumBonds() > bleBondsAtConnect && !windowOpen) {
      NimBLEDevice::deleteBond(info.getAddress());
      NimBLEDevice::getServer()->disconnect(info.getConnHandle());
      Serial0.println("[ble] ikatan baru ditolak: jendela pairing tertutup");
      return;
    }
    bleSecure = true;
    Serial0.println("[ble] tautan aman (terikat)");
  }
};

class BleChrCb : public NimBLECharacteristicCallbacks {
  void onWrite(NimBLECharacteristic *chr, NimBLEConnInfo &info) override {
    (void)info;
    const uint8_t *d = chr->getValue().data();
    size_t n = chr->getValue().size();
    for (size_t i = 0; i < n; i++) bleRxPush(d[i]);
  }
  void onSubscribe(NimBLECharacteristic *chr, NimBLEConnInfo &info, uint16_t subValue) override {
    (void)info;
    if (chr == bleMeterChr) bleMeterSub = (subValue > 0);
  }
};

static BleSrvCb bleSrvCb;
static BleChrCb bleChrCb;

static void setupBle() {
  NimBLEDevice::init("S3 Air");
  NimBLEDevice::setSecurityAuth(true, false, true);   // bonding, just-works, koneksi aman (SC)
  NimBLEDevice::setMTU(512);
  NimBLEServer *srv = NimBLEDevice::createServer();
  srv->setCallbacks(&bleSrvCb);
  NimBLEService *svc = srv->createService(BLE_SVC_UUID);
  NimBLECharacteristic *cmd = svc->createCharacteristic(
      BLE_CMD_UUID, NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::WRITE_NR | NIMBLE_PROPERTY::WRITE_ENC);
  cmd->setCallbacks(&bleChrCb);
  bleRespChr = svc->createCharacteristic(
      BLE_RESP_UUID, NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY | NIMBLE_PROPERTY::READ_ENC);
  bleRespChr->setValue("S3 Air");
  bleRespChr->setCallbacks(&bleChrCb);
  bleMeterChr = svc->createCharacteristic(BLE_METER_UUID, NIMBLE_PROPERTY::NOTIFY);
  bleMeterChr->setCallbacks(&bleChrCb);
  svc->start();
  NimBLEAdvertising *adv = NimBLEDevice::getAdvertising();
  adv->addServiceUUID(BLE_SVC_UUID);
  adv->setName("S3 Air");
  NimBLEDevice::startAdvertising();
  blePairUntil = millis() + 60000;
  Serial0.println("[ble] S3 Air teriklankan — jendela pairing 60 detik");
}

static void blePoll() {
  // BOOT ditekan >= 250 ms (lewat masa reset pabrik) -> buka jendela pairing.
  static uint32_t lowSince = 0;
  static bool announced = false;
  if (millis() > 6000) {
    if (digitalRead(0) == LOW) {
      if (!lowSince) lowSince = millis();
      else if (!announced && millis() - lowSince > 250) {
        blePairUntil = millis() + 60000; announced = true;
        Serial0.println("[ble] jendela pairing dibuka 60 detik (BOOT)");
      }
    } else { lowSince = 0; announced = false; }
  }
  // Rakit perintah dari ring buffer, jalankan per baris.
  static String line;
  while (bleRxTail != bleRxHead) {
    char c = bleRxRing[bleRxTail];
    bleRxTail = (bleRxTail + 1) % (uint16_t)sizeof(bleRxRing);
    if (c == '\n') { bleHandleLine(line); line = ""; }
    else if (c != '\r' && line.length() < 700) line += c;
  }
  // Meter + spektrum biner tiap 50 ms untuk pelanggan yang tautannya aman.
  static uint32_t lastM = 0;
  if (bleLink && bleSecure && bleMeterSub && millis() - lastM >= 50) {
    lastM = millis();
    uint8_t pkt[55];
    auto putF = [&](int off, float v) { memcpy(pkt + off, &v, 4); };
    putF(0, meterPeakL); putF(4, meterPeakR);
    putF(8, meterRmsL);  putF(12, meterRmsR);
    putF(16, grMeterDb);
    pkt[20] = (uint8_t)(((stateDirty || dlmsSaveDirty) ? 1 : 0) | (dlms.on ? 2 : 0));
    uint32_t cc = (uint32_t)clipCount;
    pkt[21] = (uint8_t)(cc & 0xFF); pkt[22] = (uint8_t)((cc >> 8) & 0xFF);
    pkt[23] = (uint8_t)min<uint32_t>((uint32_t)statFlush, 255);
    for (int i = 0; i < 31; i++) {
      int b = (int)lrintf(rtaLevels[i]) + 60;
      pkt[24 + i] = (uint8_t)constrain(b, 0, 255);
    }
    bleMeterChr->notify(pkt, sizeof(pkt), bleHandle);
    meterPeakL = meterPeakR = 0;   // semantik sama /api/meter: puncak yang dibaca direset
  }
}

// ============================================================
void setup() {
  Serial0.begin(115200);
  delay(300);
  setFlat(params);
  loadStateNvs();
  setDlmsDefaults();
  loadDlmsNvs();
  loadSysPrefs();
  pinMode(0, INPUT_PULLUP);
  dlms.on = false; // pengaman boot v5.0.4: mode DLMS selalu mulai OFF (setelan lain utuh)
  snapDlmsSmooth();
  rebuildDlmsCoefs();
  snapSmoothToParams();
  rebuildCoefficients();
  rebuildDynCoefs();
  setupAudio();
  xTaskCreatePinnedToCore(dspTask, "dspTask", 8192, NULL, 8, &dspTaskHandle, 1);
  loadWifiCreds();
  setupWeb();
  wifiStart();
  setupBle();
  Serial0.println("[USB] " FW_VERSION " siap — tunggu Windows memilih 'MbahDAC USB'.");
}

void loop() {
  wifiPoll();
  serialPoll();
  blePoll();
  if ((stateDirty || dlmsSaveDirty) && sysAutoSave && millis() - lastChangeMs > 2000 && millis() - lastStateSave > 2000) {
    lastStateSave = millis(); saveStateNvs();
  }
  bootBtnPoll();
  { // pengawas DSP + pengumpan hening saat USB berhenti (gagal = diam)
    static uint32_t wdT = 0, wdBeat = 0; static uint8_t wdStall = 0;
    static bool spkWas = false, spkSilenced = true;
    uint32_t nowMs = millis();
    if (lastSpkMs && nowMs - lastSpkMs < 2000) { spkWas = true; spkSilenced = false; }
    else if (spkWas && !spkSilenced && nowMs - lastSpkMs > 300) {
      static const int32_t zeros[1024] = {0};
      for (int i = 0; i < 3; i++) i2s.write((uint8_t *)zeros, sizeof(zeros));
      spkSilenced = true;
    }
    if (nowMs - wdT > 1000) {
      wdT = nowMs;
      if (lastSpkMs && nowMs - lastSpkMs < 2000 && dspHeartbeat == wdBeat) {
        stateFlushNeeded = true;
        if (++wdStall >= 2) { Serial0.println("[wd] DSP macet - restart pengaman"); delay(50); ESP.restart(); }
      } else wdStall = 0;
      wdBeat = dspHeartbeat;
    }
  }
  static uint32_t lastRta = 0;
  if (millis() - lastRta > 100) { lastRta = millis(); computeRTA(); }
  delay(2);
}
