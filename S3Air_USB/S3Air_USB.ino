// ============================================================
//  S3 Air - USB DAC + DSP Console
//  ESP32-S3 (N16R8) + PCM5102 | USB Audio 16-bit / 48 kHz
//  (core ESP32-S3 hanya mendukung UAC 16-bit; jalur 24-bit khusus ESP32-P4)
//  DSP di task FreeRTOS (Core 1); kontrol panel web + serial.
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
#define FW_VERSION "v5.0.6 S3 Air"

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
static volatile uint16_t xfadeRemain = 0;
static const uint16_t XFADE_LEN = 24;

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
// Delay line 20 ms = 960 sampel @48k, sediakan 1024
static float dlyBufL[1024], dlyBufR[1024];
static uint16_t dlyWr = 0;
// DLMS: delay input maks 100 ms (4800 sampel) + penghalus khusus mode DLMS
static float inDlyBufA[4800], inDlyBufB[4800];
static uint16_t inDlyWr = 0;
static float smInGainA = 1.0f, smInGainB = 1.0f;
static float smChGain[2] = {1.0f, 1.0f}, smChCeil[2] = {1.0f, 1.0f};
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
      if (h.peqOn) coefBiquad(dlmsCoef[c][8 + j], BT_PEAKING, SAMPLE_RATE, DLMS_PEQ_F[j], smDlmsPeq[c][j], 1.0f);
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
    else if (f == "dly") h.dlyMs = constrain(v, 0.0f, 20.0f);
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
static int16_t ringIn[RING_FRAMES * 2], ringOut[RING_FRAMES * 2];
static volatile uint32_t ringInW = 0, ringInR = 0, ringOutW = 0, ringOutR = 0;
static volatile uint32_t statUnderrun = 0, statOverflow = 0;
static TaskHandle_t dspTaskHandle = NULL;
static int16_t dspBlock[512 * 2];

static void processBlock(int16_t *s, int frames) {
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
      smInGainA += ((dlms.inOnA ? db2lin(dlms.inGainA) : 0.0f) - smInGainA) * SMOOTH_COEF;
      smInGainB += ((dlms.inOnB ? db2lin(dlms.inGainB) : 0.0f) - smInGainB) * SMOOTH_COEF;
      smInDlyA += (dlms.inDlyA - smInDlyA) * SMOOTH_COEF;
      smInDlyB += (dlms.inDlyB - smInDlyB) * SMOOTH_COEF;
      for (int c = 0; c < 2; c++) {
        DlmsCh &hc = dlms.ch[c];
        smChGain[c] += ((hc.on ? db2lin(hc.gainDb) * (hc.pol ? -1.0f : 1.0f) : 0.0f) - smChGain[c]) * SMOOTH_COEF;
        smChCeil[c] += (db2lin(hc.limDb) - smChCeil[c]) * SMOOTH_COEF;
        smChDly[c] += (hc.dlyMs - smChDly[c]) * SMOOTH_COEF;
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
      if (filtersChanged()) {
        rebuildCoefficients();
        mb = mainBank[activeBank]; gb = geqBank[activeBank];
        if (armDip) { xfadeRemain = XFADE_LEN; armDip = false; }
      }
    }

    float L = s[2 * n] * (1.0f / 32768.0f), R = s[2 * n + 1] * (1.0f / 32768.0f);
    if (dlms.on) {
      // --- jalur DLMS: input -> routing -> filter per kanal -> limiter kanal -> master ---
      grMeterDb = 0;
      float a = L * smInGainA, b = R * smInGainB;
      inDlyBufA[inDlyWr] = a; inDlyBufB[inDlyWr] = b;
      uint16_t dA = (uint16_t)(smInDlyA * 48.0f), dB = (uint16_t)(smInDlyB * 48.0f);
      a = inDlyBufA[(inDlyWr + 4800 - dA) % 4800];
      b = inDlyBufB[(inDlyWr + 4800 - dB) % 4800];
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
      L = dlyBufL[(dlyWr - dd0) & 1023]; R = dlyBufR[(dlyWr - dd1) & 1023];
      dlyWr = (dlyWr + 1) & 1023;
      L = smChCeil[0] * tanhf(L / (smChCeil[0] + 1e-9f));
      R = smChCeil[1] * tanhf(R / (smChCeil[1] + 1e-9f));
      if (xfadeRemain > 0) {
        float fade = (float)xfadeRemain / (float)XFADE_LEN;
        L *= (1.0f - fade); R *= (1.0f - fade);
        xfadeRemain--;
      }
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

    if (xfadeRemain > 0) {
      float fade = (float)xfadeRemain / (float)XFADE_LEN;
      L *= (1.0f - fade);
      R *= (1.0f - fade);
      xfadeRemain--;
    }

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
    L = dlyBufL[(dlyWr - dlyL) & 1023]; R = dlyBufR[(dlyWr - dlyR) & 1023];
    dlyWr = (dlyWr + 1) & 1023;

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

    float oL = constrain(L, -1.0f, 1.0f), oR = constrain(R, -1.0f, 1.0f);
    s[2 * n] = (int16_t)(oL * 32767.0f);
    s[2 * n + 1] = (int16_t)(oR * 32767.0f);
  }
  if (frames > 0) {
    meterRmsL = sqrtf(sumL / frames); meterRmsR = sqrtf(sumR / frames);
  }
}

static void onSpkData(void *data, uint16_t len) {
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
  // tarik hasil olahan dari ringOut (hening + hitung underrun bila kurang)
  uint32_t r = ringOutR;
  uint32_t avail = ringOutW - r;
  for (int n = 0; n < frames; n++) {
    if (n < (int)avail) {
      uint32_t idx = (r + n) & (RING_FRAMES - 1);
      s[n * 2] = ringOut[idx * 2]; s[n * 2 + 1] = ringOut[idx * 2 + 1];
    } else { s[n * 2] = 0; s[n * 2 + 1] = 0; statUnderrun++; }
  }
  ringOutR = r + (frames < (int)avail ? frames : (int)avail);
  if (dspTaskHandle && (ringInW - ringInR) >= 64) xTaskNotifyGive(dspTaskHandle);
  i2s.write((uint8_t *)data, len);
}

// Uji mandiri diagnostik (serial: dlmstest=1) — cabang DLMS pada blok sintetik tanpa USB.
// Bila cabang ini penyebab crash, panic-nya tercetak lengkap di Serial0 (kabel COM).
static void dlmsSelfTest() {
  Serial0.println("[test] cabang DLMS: mulai (200 blok x 512 sampel, setelan tersimpan)");
  bool wasOn = dlms.on;
  dlms.on = true; dlmsTouched = true;
  static int16_t tb[512 * 2];
  for (int i = 0; i < 512; i++) {
    float v = 0.5f * sinf(2.0f * PI * 440.0f * i / 48000.0f);
    int16_t q = (int16_t)(v * 32767.0f);
    tb[2 * i] = q; tb[2 * i + 1] = (int16_t)(q / 2);
  }
  for (int r = 0; r < 200; r++) processBlock(tb, 512);
  dlms.on = wasOn;
  Serial0.println("[test] cabang DLMS: SELESAI tanpa crash");
}

static void dspTask(void *arg) {
  (void)arg;
  for (;;) {
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
      ringOut[idx * 2] = dspBlock[i * 2]; ringOut[idx * 2 + 1] = dspBlock[i * 2 + 1];
    }
    ringOutW = w + m;
  }
}


static void setupAudio() {
  i2s.setPins(PIN_BCK, PIN_LRCK, PIN_DIN, -1);
  i2s.begin(I2S_MODE_STD, 48000, I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_STEREO, I2S_STD_SLOT_BOTH);
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
  Serial0.println("[dbg] NVS simpan: mulai");
  prefs.begin("mbahdac2", false);
  Serial0.println("[dbg] NVS simpan: begin ok");
  prefs.putBytes("dlms2", &dlms, sizeof(dlms));
  Serial0.println("[dbg] NVS simpan: putBytes ok");
  prefs.end();
  Serial0.println("[dbg] NVS simpan: selesai");
}
static void loadDlmsNvs() {
  prefs.begin("mbahdac2", true);
  DlmsParams tmp;
  size_t r = prefs.getBytes("dlms2", &tmp, sizeof(tmp));
  prefs.end();
  if (r == sizeof(tmp)) dlms = tmp;
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
  j += ",\"slots\":[";
  for (int i = 0; i < NUM_USER_PRESETS; i++) { if (i) j += ","; j += String(userPresetExists(i) ? 1 : 0); }
  j += "]}";
  return j;
}

// Halaman web disisipkan saat build (webpage.html -> PAGE)
static const char PAGE[] PROGMEM = R"HTML(<!DOCTYPE html>
<html lang="id"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>S3 Air — Konsol DSP</title>
<style>
:root{--bg:#101010;--panel:#1c1c1e;--panel2:#242426;--line:#3a3a3c;--txt:#e8e8e8;--dim:#9a9aa0;
--teal:#12b5cb;--teal2:#0e7c86;--org:#e8930c;--blue:#2b5ea7;--yel:#ffd21f;--red:#e03131;--grn:#37b24d;
--mono:"Courier New",monospace}
*{box-sizing:border-box;margin:0;padding:0}
html,body{background:var(--bg);color:var(--txt);font-family:Arial,Helvetica,sans-serif;font-size:13px}
body{display:flex;flex-direction:column;min-height:100vh;overflow-x:hidden}
button{font-family:inherit;cursor:pointer}
input,select{font-family:inherit}
/* ===== top bar ===== */
#topbar{display:flex;align-items:stretch;gap:2px;background:linear-gradient(180deg,#26262a,#17171a);border-bottom:2px solid #000;padding:4px 6px;flex-wrap:wrap}
#logo{display:flex;flex-direction:column;justify-content:center;padding:2px 12px 2px 4px;border-right:1px solid var(--line);margin-right:4px}
#logo b{font-size:17px;letter-spacing:1px;color:#fff}
#logo b span{color:var(--teal)}
#logo small{color:var(--dim);font-size:9px;letter-spacing:2px}
.tabbtn{display:flex;flex-direction:column;align-items:center;gap:1px;background:#222225;color:#cfcfcf;border:1px solid #333;border-bottom:3px solid #333;border-radius:3px;padding:4px 13px;font-size:11px;min-width:58px}
.tabbtn svg{width:17px;height:17px;stroke:#cfcfcf;fill:none;stroke-width:1.7}
.tabbtn.act{background:#2c2f33;border-bottom:3px solid var(--teal);color:#fff}
.tabbtn.act svg{stroke:var(--teal)}
#topright{margin-left:auto;display:flex;align-items:center;gap:8px}
#netpill{background:#0d2b30;color:#7fe3f0;border:1px solid var(--teal2);border-radius:3px;padding:4px 8px;font-size:11px}
#cliplamp{background:#2a0f0f;color:#ff8787;border:1px solid #5c1a1a;border-radius:3px;padding:4px 8px;font-size:11px}
#cliplamp b{font-family:var(--mono)}
/* ===== work area ===== */
#work{flex:1;display:flex;gap:6px;padding:6px;min-height:0}
#leftcol{width:172px;flex:none;display:flex;flex-direction:column;gap:6px}
#stage{flex:1;display:flex;flex-direction:column;gap:6px;min-width:0}
#rightrail{width:158px;flex:none;display:flex;flex-direction:column;gap:6px}
.card{background:var(--panel);border:1px solid var(--line);border-radius:4px;padding:7px}
.card h4{font-size:10px;letter-spacing:1.5px;color:var(--dim);border-bottom:1px solid var(--line);padding-bottom:4px;margin-bottom:6px;font-weight:bold}
#scribble{background:var(--yel);color:#111;font-weight:bold;font-size:16px;text-align:center;border-radius:3px;padding:7px 2px;letter-spacing:.5px}
.kv{display:flex;justify-content:space-between;align-items:center;margin:4px 0;gap:4px}
.kv label{color:var(--dim);font-size:11px}
.kv output{font-family:var(--mono);font-size:11px;color:#fff}
input[type=number]{width:62px;background:#0c0c0d;border:1px solid var(--line);color:#fff;border-radius:3px;padding:3px 4px;font-family:var(--mono);font-size:12px;text-align:right}
select{background:#0c0c0d;border:1px solid var(--line);color:#fff;border-radius:3px;padding:3px;font-size:11px}
input[type=range]{accent-color:var(--teal)}
input[type=range].h{width:100%}
.tgl{background:#2b2b2e;border:1px solid var(--line);color:#bbb;border-radius:3px;padding:3px 9px;font-size:11px}
.tgl.on{background:var(--teal2);color:#fff;border-color:var(--teal)}
.tgl.red.on{background:#8c1d1d;border-color:var(--red)}
.tgl.yel.on{background:#8a6d00;border-color:var(--yel)}
/* display */
#dispwrap{display:flex;gap:6px;background:#000;border:1px solid var(--line);border-radius:4px;padding:6px}
#graph{flex:1;width:100%;height:300px;display:block;touch-action:none;cursor:crosshair;min-width:0}
#dispmeter{width:30px;flex:none;display:flex;gap:3px}
.mcol{flex:1;background:#0a0a0a;border:1px solid #26262a;position:relative;overflow:hidden}
.mfill{position:absolute;left:0;right:0;bottom:0;height:0%;background:linear-gradient(180deg,#ff5252 0%,#ffd43b 18%,#37b24d 45%)}
.mlab{font-size:8px;color:var(--dim);text-align:center;margin-top:2px}
/* band bar */
#bandbar{min-height:118px}
.chips{display:flex;gap:4px;flex-wrap:wrap;margin-bottom:7px}
.chip{border:1px solid var(--line);background:#26262a;color:#ddd;border-radius:3px;padding:4px 0;font-size:11px;flex:1;min-width:44px;text-align:center;border-top:3px solid #666}
.chip.sel{background:#33363b;outline:1px solid var(--teal);color:#fff}
.chip.off{opacity:.38}
.boxes{display:flex;gap:6px;flex-wrap:wrap}
.box{background:var(--panel2);border:1px solid var(--line);border-radius:3px;padding:5px 8px;min-width:86px}
.box h5{font-size:9px;letter-spacing:1px;color:var(--dim);margin-bottom:3px}
.box .row{display:flex;align-items:center;gap:4px}
.box output{font-family:var(--mono);font-size:13px}
.box small{color:var(--dim)}
#grwrap{background:#0c0c0d;border:1px solid var(--line);height:12px;border-radius:2px;overflow:hidden}
#grbar{height:100%;width:0%;background:linear-gradient(90deg,#37b24d,#ffd43b,#ff5252)}
/* right rail */
.util{display:grid;grid-template-columns:1fr 1fr;gap:4px}
.util button{background:#26262a;border:1px solid var(--line);color:#ddd;border-radius:3px;padding:6px 2px;font-size:11px}
.util button:hover{border-color:var(--teal)}
#mainbox{text-align:center}
#mainbox .scrib{background:var(--teal);color:#04222b;font-weight:bold;border-radius:3px;padding:4px;font-size:13px;margin-bottom:6px}
input.vfader{writing-mode:vertical-lr;direction:rtl;width:26px;height:150px;accent-color:var(--teal)}
#volDb{font-family:var(--mono);font-size:15px;margin:4px 0}
.mrow{display:flex;gap:4px;justify-content:center}
/* footer fader bank */
#faderbank{display:flex;gap:4px;background:linear-gradient(180deg,#1b1b1e,#121214);border-top:2px solid #000;padding:6px;overflow-x:auto;align-items:stretch}
.strip{flex:0 0 64px;background:var(--panel);border:1px solid #2c2c2e;border-radius:3px;display:flex;flex-direction:column;align-items:center;padding:4px 2px;gap:3px}
.strip .scrib{width:100%;text-align:center;font-size:10px;font-weight:bold;color:#111;border-radius:2px;padding:2px 0}
.strip .val{font-family:var(--mono);font-size:10px;color:#fff}
.strip input.vfader{height:110px;width:22px}
.strip .onled{width:100%;font-size:9px;padding:2px 0}
.strip.sel{outline:1px solid var(--teal)}
.strip.off{opacity:.45}
.strip.gq{flex:0 0 40px}
.strip.gq input.vfader{height:96px;width:18px}
#masterstrip{flex:0 0 84px;background:#26262a;border:1px solid var(--teal2)}
#masterstrip .scrib{background:var(--red);color:#fff}
.pg{display:none}.pg.act{display:block}
.wform{display:flex;gap:5px;flex-wrap:wrap;margin-top:6px}
.wform input[type=text],.wform input[type=password]{background:#0c0c0d;border:1px solid var(--line);color:#fff;border-radius:3px;padding:5px;flex:1;min-width:120px}
table.info{border-collapse:collapse;width:100%}
table.info td{border-bottom:1px solid #2c2c2e;padding:4px 6px;font-size:12px}
table.info td:first-child{color:var(--dim);width:38%}
.note{color:var(--dim);font-size:11px;line-height:1.5}
@media (max-width:900px){
 #work{flex-direction:column}
 #leftcol,#rightrail{width:100%;flex-direction:row;flex-wrap:wrap}
 #leftcol .card,#rightrail .card{flex:1;min-width:150px}
 #graph{height:230px}
}

<style>
/* ===== lapisan USER-FRIENDLY: target sentuh besar, lega, HP-first ===== */
html,body{font-size:15px}
.card{padding:12px;border-radius:8px}
.card h4{font-size:11px;margin-bottom:9px}
.tabbtn{min-width:76px;min-height:54px;font-size:13px;padding:7px 14px;border-radius:6px}
.tabbtn svg{width:20px;height:20px}
#logo b{font-size:19px}
.kv{margin:8px 0}
.kv label{font-size:13px}
.kv output{font-size:13px}
input[type=number]{width:76px;padding:8px 6px;font-size:14px;border-radius:6px}
select{padding:8px;font-size:13px;border-radius:6px}
.tgl{padding:8px 14px;font-size:13px;min-height:40px;border-radius:6px}
.chip{min-height:44px;font-size:13px;border-radius:6px}
.box{padding:9px 12px;border-radius:6px;min-width:104px}
.box h5{font-size:10px;margin-bottom:5px}
.box output{font-size:15px}
#graph{height:340px}
#scribble{font-size:19px;padding:10px 2px;border-radius:6px}
.strip{flex-basis:78px;padding:7px 3px;border-radius:6px}
.strip .val{font-size:12px}
.strip input.vfader{height:132px;width:28px}
.strip.gq{flex-basis:48px}
.strip.gq input.vfader{height:112px;width:24px}
#masterstrip{flex-basis:96px;margin-left:auto}
input.vfader{height:170px;width:34px}
#volDb{font-size:18px}
.util button{padding:10px 2px;font-size:13px;border-radius:6px}
.note{font-size:12.5px}
.wform input[type=text],.wform input[type=password]{padding:9px;font-size:14px}
@media (max-width:900px){
 #stage{order:1}#rightrail{order:2}#leftcol{order:3}
 #graph{height:280px}
 .tabbtn{min-width:64px;padding:6px 8px}
}

<style>
#work{display:block}
#graph{height:360px}
#masterstrip{flex-basis:112px}
#masterstrip input.vfader{height:120px}

<style>
html,body{height:100%}
body{height:100dvh}
#work{flex:1;min-height:0;display:flex}
#stage{flex:1;display:flex;flex-direction:column;min-width:0}
#dispwrap{flex:1;min-height:270px}
#graph{flex:1;height:100%;min-height:240px}
.strip input.vfader{height:185px}
#masterstrip input.vfader{height:185px}
@media (max-width:900px){.strip input.vfader{height:165px}#masterstrip input.vfader{height:165px}}
#dispwrap>div{display:flex;flex-direction:column;gap:4px;min-height:0}
#dispmeter{flex:1;min-height:0}
/* --- simetri & anti blok kosong (v3.6) --- */
#strips{flex:1;min-width:0}
/* --- v3.8: bank melayang aman + anti ketutup --- */
#faderbank{position:fixed;left:0;right:0;bottom:0;z-index:6;width:100%}
body{padding-bottom:268px}
.row{flex-wrap:wrap}
#pg-preset .boxes{align-items:start}
#pg-main input[type=number]{width:58px}
@media (max-width:900px){body{padding-bottom:232px}}
#pg-eq .boxes input[type=range]{flex:1;min-width:50px}
.strip{flex:1 1 0;min-width:0}
#masterstrip{flex:0 0 112px;margin-left:0}
#bandbar{min-height:0}
.boxes{display:grid;grid-template-columns:repeat(auto-fit,minmax(170px,1fr));gap:6px}
#factory{display:grid;grid-template-columns:repeat(auto-fit,minmax(100px,1fr));gap:5px}
#slots{display:grid;grid-template-columns:repeat(auto-fit,minmax(150px,1fr));gap:5px}
@media (max-width:900px){
  .strip{flex:0 0 68px}
  #masterstrip{flex:0 0 92px}
  .boxes{grid-template-columns:1fr 1fr}
  #pg-eq .boxes .box:nth-child(5){grid-column:span 2}
}

<style>
@media (max-width:900px){
  body{height:auto;min-height:100dvh}
  #topbar{flex-wrap:nowrap;overflow-x:auto;scrollbar-width:none}
  #topbar>*{flex:none}
  #work{display:block;min-height:0}
  #stage{display:block}
  #dispwrap{height:290px;flex:none}
  #graph{height:100%;min-height:0;flex:none}
  .chips{flex-wrap:nowrap;overflow-x:auto;scrollbar-width:none;padding-bottom:4px}
  .chip{flex:none}
  #bandbar .mrow{flex-wrap:wrap;justify-content:flex-start}
  #faderbank{position:sticky;bottom:0;z-index:6;box-shadow:0 -4px 14px rgba(0,0,0,.5)}
  #masterstrip{position:sticky;right:0;background:#26262a}
  #netpill{display:none}
  .strip input.vfader{height:150px}
  #masterstrip input.vfader{height:150px}
}

/* ===== v3.9 BROADCAST NEON ===== */
:root{--bg:#05070d;--panel:#0b1018;--panel2:#0e1622;--line:#22344d;--txt:#e8f3ff;--dim:#7d92ad;--teal:#22d3ee;--teal2:#0e7490;--org:#f471b5;--blue:#2563eb}
body{background:#05070d;background-image:radial-gradient(1000px 340px at 50% -120px,rgba(14,116,144,.38),rgba(5,7,13,0) 70%)}
#topbar{background:linear-gradient(180deg,#0d1626,#070b13);border-bottom:1px solid #22344d;box-shadow:0 2px 18px rgba(34,211,238,.10)}
#logo b span{color:#22d3ee;text-shadow:0 0 14px rgba(34,211,238,.75)}
.tabbtn{background:#0a111f;border-color:#1d2a3d;color:#a9c2da;border-radius:8px}
.tabbtn.act{background:#0d1f30;border-bottom:3px solid #22d3ee;color:#fff;box-shadow:0 0 16px rgba(34,211,238,.22)}
.tabbtn.act svg{stroke:#22d3ee}
#netpill{background:#07141d;border-color:#0e7490;color:#7fe3f0;border-radius:8px}
#cliplamp{border-radius:8px}
.card{background:linear-gradient(180deg,#0c1322,#080d16);border-color:#22344d;border-radius:12px}
#dispwrap{background:#04070d;border-color:#22344d;border-radius:12px;box-shadow:inset 0 0 46px rgba(34,211,238,.05)}
.box{background:#0c1322;border-color:#22344d;border-radius:10px}
.tgl{background:#0a111f;border-color:#22344d;color:#cfe6f5;border-radius:8px}
.tgl.on{background:#22d3ee;border-color:#22d3ee;color:#04121a;font-weight:bold;box-shadow:0 0 14px rgba(34,211,238,.45)}
.tgl.red.on{background:#f43f5e;border-color:#f43f5e;color:#fff;box-shadow:0 0 14px rgba(244,63,94,.45)}
.chip{background:#0c1322;border-color:#22344d;border-radius:8px}
.chip.sel{background:#10303f;outline:2px solid #22d3ee;color:#fff}
select{background:#0a111f;border-color:#22344d;border-radius:8px}
input[type=number],input[type=text]{background:#0a0f18;border:1px solid #22344d;color:var(--txt);border-radius:8px}
input[type=range]{accent-color:#22d3ee}
#faderbank{background:linear-gradient(180deg,#0c1424,#05080f);border-top:2px solid #22d3ee;box-shadow:0 -4px 26px rgba(34,211,238,.20)}
.strip{background:#0a111f;border-color:#22344d;border-radius:10px}
#masterstrip{background:#160a14;border-color:#f471b5;box-shadow:0 0 18px rgba(244,113,181,.22);border-radius:10px}
::-webkit-scrollbar{width:8px;height:8px}::-webkit-scrollbar-thumb{background:#22344d;border-radius:4px}::-webkit-scrollbar-track{background:transparent}
</style></head>
<body data-tab="eq">
<div id="topbar">
  <div id="logo"><b>S3 <span>AIR</span></b><small>USB DAC · DSP</small></div>
  <button class="tabbtn act" data-tab="eq"><svg viewBox="0 0 24 24"><path d="M2 12 Q6 4 9 12 T16 8 T22 10"/></svg>EQ</button>
  <button class="tabbtn" data-tab="gate"><svg viewBox="0 0 24 24"><path d="M3 20 V10 h6 v10 M9 10 h12"/></svg>Gate</button>
  <button class="tabbtn" data-tab="comp"><svg viewBox="0 0 24 24"><path d="M3 19 L12 19 L21 6"/></svg>Comp</button>
  <button class="tabbtn" data-tab="main"><svg viewBox="0 0 24 24"><path d="M4 6 h16 M4 12 h16 M4 18 h16"/><circle cx="9" cy="6" r="2"/><circle cx="15" cy="12" r="2"/><circle cx="7" cy="18" r="2"/></svg>Main</button>
  <button class="tabbtn" data-tab="preset"><svg viewBox="0 0 24 24"><path d="M4 4 h13 l3 3 v13 H4 z"/><path d="M7 4 v5 h9 V4 M7 20 v-6 h10 v6"/></svg>Preset</button>
  <button class="tabbtn" data-tab="sys"><svg viewBox="0 0 24 24"><circle cx="12" cy="12" r="3"/><path d="M12 2 v4 M12 18 v4 M2 12 h4 M18 12 h4"/></svg>Sistem</button>
  <button class="tabbtn" data-tab="dlms"><svg viewBox="0 0 24 24"><path d="M4 20 V10 M12 20 V4 M20 20 V13"/></svg>DLMS</button>
  <div id="topright"><span id="netpill">WiFi: ...</span><span id="cliplamp">CLIP <b id="clipct">0</b></span></div>
</div>

<div id="work">
  

  <main id="stage">
    <div id="dispwrap">
      <canvas id="graph" width="900" height="300"></canvas>
      <div>
        <div id="dispmeter"><div class="mcol"><div class="mfill" id="mLf"></div></div><div class="mcol"><div class="mfill" id="mRf"></div></div></div>
        <div class="mlab">OUT&nbsp;&nbsp;L&nbsp;&nbsp;R</div>
      </div>
    </div>

    <div id="lower"><div class="card" id="bandbar">
      <section class="pg act" id="pg-eq">
        <div class="mrow" style="justify-content:flex-start;gap:6px;margin-bottom:8px">
          <button class="tgl" id="modePeq">PEQ 10</button><button class="tgl" id="modeGeq">GEQ 31</button>
          <button class="tgl red" id="bypass">BYPASS</button><button class="tgl" id="loud">LOUDNESS</button><button class="tgl" id="autoHr">AUTO-HEADROOM</button>
        </div>
        <div class="chips" id="chips"></div>
        <div class="boxes">
          <div class="box"><h5 id="ed_name">BAND 1</h5><div class="row"><button class="tgl" id="ed_on">AKTIF</button><select id="ed_t"><option value="0">Peak</option><option value="1">Low Shelf</option><option value="2">High Shelf</option><option value="3">Notch</option><option value="4">Band Pass</option></select></div></div>
          <div class="box"><h5>GAIN</h5><div class="row"><input type="range" id="ed_g" min="-15" max="15" step="0.1" style="width:110px"><output id="ed_g_v">0.0</output><small>dB</small></div></div>
          <div class="box"><h5>FREKUENSI</h5><div class="row"><input type="range" id="ed_fr" min="0" max="1000" step="1" style="width:110px"><input type="number" id="ed_f" min="20" max="20000" step="1"><small>Hz</small></div></div>
          <div class="box"><h5>Q</h5><div class="row"><input type="range" id="ed_q" min="0.4" max="16" step="0.1" style="width:80px"><output id="ed_q_v">1.0</output></div></div>
          <div class="box"><h5>DYNAMIC EQ</h5><div class="row"><button class="tgl" id="ed_dyn">DYN</button><small>Thr</small><input type="number" id="ed_dthr" min="-60" max="0" step="1" style="width:52px"><small>Range</small><input type="number" id="ed_drng" min="-12" max="12" step="0.5" style="width:52px"></div></div>
        </div>
        <p class="note" style="margin-top:6px">Geser titik warna pada grafik untuk mengatur frekuensi &amp; gain. Putar roda mouse di atas titik untuk Q, klik ganda untuk on/off band.</p>
      </section>

      <section class="pg" id="pg-gate">
        <div class="boxes">
          <div class="box"><h5>NOISE GATE</h5><div class="row"><button class="tgl" id="gateOn">AKTIF</button></div></div>
          <div class="box"><h5>THRESHOLD</h5><div class="row"><input type="number" id="gateThr" min="-80" max="0" step="1"><small>dB</small></div></div>
          <div class="box"><h5>ATTACK</h5><div class="row"><input type="number" id="gateAtk" min="0.1" max="100" step="0.1" style="width:56px"><small>ms</small></div></div>
          <div class="box"><h5>RELEASE</h5><div class="row"><input type="number" id="gateRel" min="10" max="1000" step="10" style="width:56px"><small>ms</small></div></div>
        </div>
        <p class="note" style="margin-top:6px">Grafik menampilkan kurva transfer gate terhadap level input. Gate memutus sinyal di bawah threshold (desah antar-lagu hilang).</p>
      </section>

      <section class="pg" id="pg-comp">
        <div class="boxes">
          <div class="box"><h5>COMPRESSOR</h5><div class="row"><button class="tgl" id="compOn">AKTIF</button></div></div>
          <div class="box"><h5>THRESHOLD</h5><div class="row"><input type="number" id="compThr" min="-60" max="0" step="1"><small>dB</small></div></div>
          <div class="box"><h5>RATIO</h5><div class="row"><input type="number" id="compRatio" min="1" max="20" step="0.5" style="width:52px"><small>: 1</small></div></div>
          <div class="box"><h5>ATTACK</h5><div class="row"><input type="number" id="compAtk" min="0.1" max="300" step="1" style="width:56px"><small>ms</small></div></div>
          <div class="box"><h5>RELEASE</h5><div class="row"><input type="number" id="compRel" min="10" max="1000" step="10" style="width:56px"><small>ms</small></div></div>
          <div class="box"><h5>MAKEUP</h5><div class="row"><input type="number" id="compMake" min="0" max="24" step="0.5" style="width:52px"><small>dB</small></div></div>
          <div class="box"><h5>LIMITER</h5><div class="row"><button class="tgl" id="limiter">AKTIF</button><small>Ceiling</small><input type="number" id="ceiling" min="-12" max="0" step="0.5" style="width:52px"><small>dB</small></div></div>
        </div>
        <div style="margin-top:7px"><div class="kv"><label>Gain Reduction</label><output id="gr_v">0 dB</output></div><div id="grwrap"><div id="grbar"></div></div></div>
      </section>

      <section class="pg" id="pg-main">
        <div class="boxes">
          <div class="box"><h5>PREAMP</h5><div class="row"><input type="range" id="pre" min="-24" max="24" step="0.5" style="width:110px"><output id="pre_v">0.0 dB</output></div></div>
          <div class="box"><h5>TRIM L / R</h5><div class="row"><input type="number" id="trimL" min="-12" max="12" step="0.5" value="0"><input type="number" id="trimR" min="-12" max="12" step="0.5" value="0"></div></div>
          <div class="box"><h5>FASE (BALIK POLARITAS)</h5><div class="row"><button class="tgl" id="polL">INV L</button><button class="tgl" id="polR">INV R</button></div></div>
          <div class="box"><h5>LOW CUT (HPF)</h5><div class="row"><button class="tgl" id="hpfOn">AKTIF</button><input type="number" id="hpfFreq" min="20" max="200" step="1" value="30" style="width:56px"><small>Hz</small><select id="hpfSlope"><option value="6">6 dB</option><option value="12">12 dB</option><option value="18">18 dB</option><option value="24">24 dB</option></select></div></div>
          
          <div class="box"><h5>DELAY L</h5><div class="row"><input type="number" id="dlyL" min="0" max="20" step="0.1" style="width:56px"><small>ms</small></div></div>
          <div class="box"><h5>DELAY R</h5><div class="row"><input type="number" id="dlyR" min="0" max="20" step="0.1" style="width:56px"><small>ms</small></div></div>
          <div class="box"><h5>SELISIH JARAK</h5><div class="row"><output id="dlyCm">0 cm</output></div></div>
          <div class="box"><h5>STEREO WIDTH</h5><div class="row"><input type="range" id="width" min="0" max="2" step="0.05" style="width:100px"><output id="width_v">1.00</output></div></div>
          <div class="box"><h5>BALANCE</h5><div class="row"><input type="range" id="balance" min="-1" max="1" step="0.05" style="width:100px"><output id="balance_v">0</output></div></div>
          <div class="box"><h5>CROSSFEED</h5><div class="row"><button class="tgl" id="xfOn">AKTIF</button><input type="range" id="xfAmt" min="0" max="0.5" step="0.05" style="width:70px"><output id="xfAmt_v">0</output></div></div>
          <div class="box"><h5>LAINNYA</h5><div class="row"><button class="tgl" id="mono">MONO</button><button class="tgl yel" id="dim">DIM</button></div></div>
          <div class="box"><h5>BATAS VOLUME</h5><div class="row"><input type="range" id="volMax" min="0.1" max="1" step="0.05" style="width:90px"><output id="volMax_v">100%</output></div></div>
        </div>
      </section>

      <section class="pg" id="pg-preset">
        <div class="boxes">
          <div class="box"><h5>PRESET PABRIK</h5><div class="row" id="factory"></div></div>
          <div class="box"><h5>BANDING A/B</h5><div class="row"><button class="tgl" id="abBtn">Ke B</button><output id="abInfo">A aktif</output></div></div>
          <div class="box"><h5>EKSPOR / IMPOR</h5><div class="row"><button class="tgl" id="expBtn">Ekspor</button><button class="tgl" id="impBtn">Impor</button><input type="file" id="impFile" accept=".json" style="display:none"></div></div>
          <div class="box"><h5>CLIP</h5><div class="row"><button class="tgl" id="clipResetBtn">Reset Hitung</button></div></div>
        </div>
        <h4 style="font-size:10px;letter-spacing:1.5px;color:var(--dim);margin:8px 0 5px">PRESET SAYA (10 SLOT)</h4>
        <div class="chips" id="slots"></div>
      </section>

      <section class="pg" id="pg-sys">
        <table class="info">
          <tr><td>Jaringan</td><td id="netinfo">—</td></tr>
          <tr><td>Firmware</td><td id="fwinfo">S3 Air</td></tr>
          <tr><td>Ring Buffer DSP</td><td id="ringinfo">—</td></tr>
          <tr><td>Puncak EQ</td><td id="eqpeak">—</td></tr>
        </table>
        <div class="wform">
          <input type="text" id="ssid" placeholder="SSID WiFi baru">
          <input type="password" id="wpass" placeholder="Password WiFi baru">
          <button class="tgl" id="wifiBtn">Simpan &amp; Restart</button>
        </div>
        <p class="note" style="margin-top:6px">Panel ini juga bisa dikendalikan lewat port COM (Serial Monitor 115200): <i>set vol=0.8</i>, <i>set b0_g=3</i>, <i>preset Bass</i>, <i>state</i>.</p>
      </section>
<section class="pg" id="pg-dlms">
        <div class="mrow" style="justify-content:flex-start;gap:6px;margin-bottom:8px">
          <button class="tgl" id="dlmsOn">ON</button><span class="note">Mode DLMS — rantai reguler dilewati, master volume tetap jalan.</span>
        </div>
        <div class="dlmsbody">
        <div class="boxes">
          <div class="box"><h5>INPUT A (KIRI)</h5><div class="row"><button class="tgl" id="inOnA">ON</button><small>Gain</small><input type="range" id="inGainA" min="-60" max="12" step="0.5" style="width:90px"><output id="inGainAV">0.0 dB</output><small>Delay</small><input type="number" id="inDlyA" min="0" max="100" step="0.5" style="width:56px"><small>ms</small></div></div>
          <div class="box"><h5>INPUT B (KANAN)</h5><div class="row"><button class="tgl" id="inOnB">ON</button><small>Gain</small><input type="range" id="inGainB" min="-60" max="12" step="0.5" style="width:90px"><output id="inGainBV">0.0 dB</output><small>Delay</small><input type="number" id="inDlyB" min="0" max="100" step="0.5" style="width:56px"><small>ms</small></div></div>
        </div>
        <div class="dlmsgrid">
    <div class="box dch" id="dch0">
      <h5><span style="color:#3f8a5f;font-size:14px">L</span><button class="tgl" id="d0_on" style="float:right">ON</button></h5>
      <div class="drow"><label>SUMBER</label><button class="tgl srcbtn">IN A</button><button class="tgl srcbtn">IN B</button><button class="tgl srcbtn">A+B</button></div>
      <div class="drow"><label>HPF</label><button class="tgl" id="d0_hpfOn">ON</button><input type="range" id="d0_hpfF" min="0" max="1000" value="700"><span class="val" id="d0_hpfV">2.5 kHz</span></div>
      <div class="drow"><label>LPF</label><button class="tgl" id="d0_lpfOn">ON</button><input type="range" id="d0_lpfF" min="0" max="1000" value="700"><span class="val" id="d0_lpfV">2.5 kHz</span></div>
      <div class="drow"><label>TIPE</label><select class="dtf" id="d0_tf"><option value="0">LR12</option><option value="1" selected>LR24</option><option value="2">LR48</option><option value="3">BW12</option><option value="4">BW24</option><option value="5">BW48</option></select></div>
      <div class="drow"><label>PEQ</label><button class="tgl" id="d0_peqOn">ON</button><div class="peq3"><div class="pq"><input type="range" id="d0_p0" min="-12" max="12" step="0.5" value="0"><span id="d0_p0v">100 · 0.0</span></div><div class="pq"><input type="range" id="d0_p1" min="-12" max="12" step="0.5" value="0"><span id="d0_p1v">1000 · 0.0</span></div><div class="pq"><input type="range" id="d0_p2" min="-12" max="12" step="0.5" value="0"><span id="d0_p2v">8000 · 0.0</span></div></div></div>
      <div class="drow"><label>GAIN</label><input type="range" id="d0_gain" min="-60" max="12" step="0.5" value="0"><span class="val" id="d0_gainV">0.0 dB</span></div>
      <div class="drow"><label>DELAY</label><input type="range" id="d0_dly" min="0" max="20" step="0.1" value="0"><span class="val" id="d0_dlyV">0.0 ms</span></div>
      <div class="drow"><label>LIMITER</label><input type="range" id="d0_lim" min="-24" max="0" step="0.5" value="-3"><span class="val" id="d0_limV">-3.0 dB</span></div>
      <div class="drow"><label>FASE</label><button class="tgl" id="d0_pol">0°</button></div>
    </div>
    <div class="box dch" id="dch1">
      <h5><span style="color:#e58f8f;font-size:14px">R</span><button class="tgl" id="d1_on" style="float:right">ON</button></h5>
      <div class="drow"><label>SUMBER</label><button class="tgl srcbtn">IN A</button><button class="tgl srcbtn">IN B</button><button class="tgl srcbtn">A+B</button></div>
      <div class="drow"><label>HPF</label><button class="tgl" id="d1_hpfOn">ON</button><input type="range" id="d1_hpfF" min="0" max="1000" value="700"><span class="val" id="d1_hpfV">2.5 kHz</span></div>
      <div class="drow"><label>LPF</label><button class="tgl" id="d1_lpfOn">ON</button><input type="range" id="d1_lpfF" min="0" max="1000" value="700"><span class="val" id="d1_lpfV">2.5 kHz</span></div>
      <div class="drow"><label>TIPE</label><select class="dtf" id="d1_tf"><option value="0">LR12</option><option value="1" selected>LR24</option><option value="2">LR48</option><option value="3">BW12</option><option value="4">BW24</option><option value="5">BW48</option></select></div>
      <div class="drow"><label>PEQ</label><button class="tgl" id="d1_peqOn">ON</button><div class="peq3"><div class="pq"><input type="range" id="d1_p0" min="-12" max="12" step="0.5" value="0"><span id="d1_p0v">100 · 0.0</span></div><div class="pq"><input type="range" id="d1_p1" min="-12" max="12" step="0.5" value="0"><span id="d1_p1v">1000 · 0.0</span></div><div class="pq"><input type="range" id="d1_p2" min="-12" max="12" step="0.5" value="0"><span id="d1_p2v">8000 · 0.0</span></div></div></div>
      <div class="drow"><label>GAIN</label><input type="range" id="d1_gain" min="-60" max="12" step="0.5" value="0"><span class="val" id="d1_gainV">0.0 dB</span></div>
      <div class="drow"><label>DELAY</label><input type="range" id="d1_dly" min="0" max="20" step="0.1" value="0"><span class="val" id="d1_dlyV">0.0 ms</span></div>
      <div class="drow"><label>LIMITER</label><input type="range" id="d1_lim" min="-24" max="0" step="0.5" value="-1"><span class="val" id="d1_limV">-1.0 dB</span></div>
      <div class="drow"><label>FASE</label><button class="tgl" id="d1_pol">0°</button></div>
    </div>
        </div>
        </div>
      </section>
    </div><aside id="tower"><div class="towIn">
      <div class="tscrib">MAIN LR</div>
      <div class="towMid"><div class="towMeters"><div class="mcol towM"><div class="mfill" id="tLf"></div></div><div class="mcol towM"><div class="mfill" id="tRf"></div></div></div><input type="range" class="vfader" id="tvol" min="0" max="1" step="0.01" value="0.75"></div>
      <div id="tvolDb">-2.5 dB</div>
      <button class="tgl red onled" id="tmute">MUTE</button>
    </div></aside></div>
  </main>

  
</div>

<footer id="faderbank">
  <div id="strips" style="display:flex;gap:4px"></div>
  <div class="strip" id="masterstrip">
    <div class="scrib">MAIN</div>
    <input type="range" class="vfader" id="vol" min="0" max="1" step="0.01" value="0.75">
    <div class="val" id="volDb">-2.5 dB</div>
    <div class="mcol" style="width:22px;height:44px"><div class="mfill" id="mMf"></div></div>
    <button class="tgl red onled" id="mute">MUTE</button>
  </div>
</footer>

<style>
/* ===== S3 Air v4.0 - tata letak L3 Menara Master ===== */
#lower{display:flex;gap:6px;flex:1;min-height:0;margin-top:-6px}
#bandbar{flex:1;min-width:0;border-top-left-radius:0;border-top-right-radius:0}
#dispwrap{border-bottom-left-radius:0;border-bottom-right-radius:0;border-bottom:none}
#tower{display:none}
.towIn{position:absolute;top:0;left:0;right:0;bottom:-260px;z-index:7;background:#160a14;border:1px solid #f471b5;border-radius:12px;box-shadow:0 0 18px rgba(244,113,181,.22);display:flex;flex-direction:column;align-items:center;padding:10px;gap:9px}
.tscrib{width:100%;text-align:center;background:var(--red);color:#fff;font-weight:bold;font-size:13px;letter-spacing:1px;border-radius:6px;padding:5px 0}
.towMid{flex:1;display:flex;gap:10px;justify-content:space-between;width:100%;min-height:0;padding:0 12px}
.towMeters{display:flex;gap:6px;min-height:0}
.towM{flex:0 0 16px}
#tvol{width:30px;height:auto}
#tvolDb{font-family:var(--mono);font-size:19px;font-weight:bold}
#tmute{width:100%}
@media (min-width:901px){
 body[data-tab="eq"] #tower{display:block;width:196px;flex:none;position:relative}
 body[data-tab="eq"] #masterstrip{display:none}
 body[data-tab="eq"] #strips{margin-right:208px}
}
</style>
<style>
/* ===== S3 Air v4.1 - navbar pil + menara gaya rancangan ===== */
#topbar{align-items:center}
.tabbtn{flex-direction:row;min-width:0;min-height:38px;padding:7px 20px;border-radius:8px;font-size:13px;border:1px solid var(--line);background:var(--panel2)}
.tabbtn svg{display:none}
.tabbtn.act{background:#22d3ee;color:#04121a;border-color:#22d3ee;box-shadow:0 0 14px rgba(34,211,238,.45);font-weight:bold}
#tvol{accent-color:#e5484d}
.towM{flex:0 0 18px}
.towM::after{content:'';position:absolute;inset:0;background:repeating-linear-gradient(180deg,transparent 0 7px,rgba(0,0,0,.55) 7px 9px);pointer-events:none}
@media (max-width:900px){.tabbtn{padding:7px 13px;min-height:40px}}
</style>
<style>
/* ===== S3 Air v4.3.1 - warna SOFT + kap fader mixer + fader lurus tengah ===== */
:root{--teal:#63c7de;--teal2:#3d7e94;--org:#e79ec3}
body{background:radial-gradient(1100px 420px at 50% -140px,rgba(99,199,222,.07),transparent 62%),#05070d}
#logo b span{color:#63c7de;text-shadow:0 0 10px rgba(99,199,222,.35)}
#dispwrap{box-shadow:inset 0 0 46px rgba(99,199,222,.03)}
.tabbtn.act{background:#63c7de;border-color:#63c7de;box-shadow:0 0 10px rgba(99,199,222,.25)}
.tgl.on{background:#63c7de;border-color:#63c7de;box-shadow:0 0 10px rgba(99,199,222,.25)}
.chip.sel{outline-color:#63c7de}
input[type=range]{accent-color:#63c7de}
#faderbank{border-top-color:#63c7de;box-shadow:0 -4px 18px rgba(99,199,222,.12)}
#masterstrip{border-color:#e79ec3;box-shadow:0 0 12px rgba(231,158,195,.16)}
.towIn{border-color:#e79ec3;box-shadow:0 0 12px rgba(231,158,195,.16)}
/* --- kap fader mixer --- */
input[type=range]{-webkit-appearance:none;appearance:none;background:transparent;height:22px;--capline:#63c7de}
input[type=range]::-webkit-slider-runnable-track{height:6px;border-radius:3px;background:#101114;border:1px solid #000;box-shadow:inset 0 1px 3px rgba(0,0,0,.8)}
input[type=range]::-webkit-slider-thumb{-webkit-appearance:none;width:15px;height:24px;margin-top:-10px;border-radius:3px;border:1px solid #000;background:linear-gradient(90deg,transparent 40%,var(--capline) 40%,var(--capline) 60%,transparent 60%),linear-gradient(180deg,#43464e,#1a1b1f 55%,#2e3138);box-shadow:0 2px 6px rgba(0,0,0,.6),inset 0 1px 0 rgba(255,255,255,.20)}
input[type=range]::-moz-range-track{height:6px;border-radius:3px;background:#101114;border:1px solid #000}
input[type=range]::-moz-range-thumb{width:15px;height:24px;border-radius:3px;border:1px solid #000;background:linear-gradient(90deg,transparent 40%,var(--capline) 40%,var(--capline) 60%,transparent 60%),linear-gradient(180deg,#43464e,#1a1b1f 55%,#2e3138)}
input.vfader{height:auto;width:9px;transform:translateX(4px)}
input.vfader::-webkit-slider-runnable-track{width:9px;height:auto;background:linear-gradient(180deg,transparent 49.2%,rgba(230,240,250,.38) 49.2%,rgba(230,240,250,.38) 50.8%,transparent 50.8%),linear-gradient(90deg,#08090b,#191a1e 45%,#26282e 55%,#08090b);border:1px solid #000;border-radius:5px;box-shadow:inset 0 0 0 1px rgba(255,255,255,.05),inset 0 0 12px rgba(0,0,0,.8)}
input.vfader::-webkit-slider-thumb{width:40px;height:21px;margin-top:0;margin-left:-20px;border-radius:4px;background:linear-gradient(180deg,transparent 42%,var(--capline) 42%,var(--capline) 58%,transparent 58%),repeating-linear-gradient(90deg,transparent 0 7px,rgba(0,0,0,.30) 7px 8px),linear-gradient(180deg,#43464e,#1a1b1f 55%,#2e3138);box-shadow:0 3px 8px rgba(0,0,0,.65),inset 0 1px 0 rgba(255,255,255,.22)}
input.vfader::-moz-range-track{width:9px;background:linear-gradient(180deg,transparent 49.2%,rgba(230,240,250,.38) 49.2%,rgba(230,240,250,.38) 50.8%,transparent 50.8%),#141519;border:1px solid #000;border-radius:5px}
input.vfader::-moz-range-thumb{width:40px;height:21px;margin-left:-20px;border-radius:4px;background:linear-gradient(180deg,transparent 42%,var(--capline) 42%,var(--capline) 58%,transparent 58%),linear-gradient(180deg,#43464e,#1a1b1f 55%,#2e3138)}
.strip.gq input.vfader{width:30px}
.strip.gq input.vfader::-webkit-slider-thumb{width:30px}
.strip.gq input.vfader::-moz-range-thumb{width:30px}
#tvol{--capline:#f08585;width:48px}
#tvol::-webkit-slider-thumb{width:48px;height:24px;transform:none}
#tvol::-moz-range-thumb{width:48px;height:24px;transform:none}
.strip.gq input.vfader::-webkit-slider-thumb{margin-left:-15px}
.strip.gq input.vfader::-moz-range-thumb{margin-left:-15px}
#tvol::-moz-range-thumb{width:48px;height:24px}
#vol{--capline:#f08585}
</style>

<style>
/* ===== S3 Air v5.0 - tema SOFT HIJAU ala VoiceMeeter + tab DLMS ===== */
:root{--teal:#3f8a5f;--teal2:#2c5c40;--org:#e58f8f}
body{background:radial-gradient(1100px 420px at 50% -140px,rgba(63,138,95,.05),transparent 62%),#05070d}
#logo b span{color:#3f8a5f;text-shadow:0 0 10px rgba(63,138,95,.22)}
#dispwrap{box-shadow:inset 0 0 46px rgba(63,138,95,.03)}
.tabbtn.act{background:#3f8a5f;border-color:#3f8a5f;box-shadow:0 0 10px rgba(63,138,95,.15)}
.tgl.on{background:#3f8a5f;border-color:#3f8a5f;box-shadow:0 0 10px rgba(63,138,95,.15)}
.chip.sel{outline-color:#3f8a5f}
input[type=range]{accent-color:#3f8a5f;--capline:#3f8a5f}
#faderbank{border-top-color:#3f8a5f;box-shadow:0 -4px 18px rgba(63,138,95,.08)}
#masterstrip{border-color:#e58f8f;box-shadow:0 0 12px rgba(229,143,143,.16)}
.towIn{border-color:#e58f8f;box-shadow:0 0 12px rgba(229,143,143,.16)}
#bandbar{overflow-y:auto;min-height:0;scrollbar-width:thin}
#strips .strip.dls{flex:0 0 84px}
/* --- halaman DLMS --- */
#pg-dlms.dlmsoff .dlmsbody{opacity:.38;pointer-events:none}
.dlmsgrid{display:grid;grid-template-columns:1fr 1fr;gap:6px;margin-top:6px}
@media (max-width:900px){.dlmsgrid{grid-template-columns:1fr}}
.dch{transition:opacity .2s}
.dch.off{opacity:.5}
.drow{display:flex;align-items:center;gap:8px;margin:8px 0}
.drow label{width:62px;flex:none;color:var(--dim);font-size:11px;letter-spacing:1.2px}
.drow .val{width:78px;flex:none;text-align:right;font-variant-numeric:tabular-nums;font-size:13px}
.drow input[type=range]{flex:1;min-width:0}
.drow .tgl{flex:none}
select.dtf{background:#0a0f18;color:var(--txt);border:1px solid var(--line);border-radius:6px;padding:6px 8px;font-size:13px;flex:1;min-width:0}
.peq3{display:flex;gap:10px;flex:1;min-width:0}
.peq3 .pq{flex:1;text-align:center;min-width:0}
.peq3 .pq input{width:100%}
.peq3 .pq span{display:block;font-size:11px;color:var(--dim);font-variant-numeric:tabular-nums}
</style>
<script>
const $=id=>document.getElementById(id);
let S=null, tab='eq', sel=0, RT=new Array(31).fill(-60), RTs=new Array(31).fill(-60), snapA=null, snapB=null, abIsB=false;
const BCOL=['#5ec2d6','#7db8f0','#8ad99a','#f2d06b','#f5b97f','#ef8fb5','#c3aef2','#f49a9a','#7fe0c3','#d3a6ec'];
const GEQF=[20,25,31.5,40,50,63,80,100,125,160,200,250,315,400,500,630,800,1000,1250,1600,2000,2500,3150,4000,5000,6300,8000,10000,12500,16000,20000];
const FS=48000;
function send(o){return fetch('/api/set',{method:'POST',headers:{'Content-Type':'application/x-www-form-urlencoded'},body:new URLSearchParams(o)}).catch(()=>{});}
function tgl(id,on){$(id).classList.toggle('on',!!on);}
function fmtF(f){return f>=1000?((f/1000).toFixed(f>=10000?0:1).replace('.0','')+'k'):(''+Math.round(f));}
function volDb(v){return v<=0.001?'-inf':(20*Math.log10(v)).toFixed(1);}
/* ---------- hitung respon biquad (cermin firmware) ---------- */
function coef(type,f0,g,q){
  if((type==0||type==1||type==2)&&Math.abs(g)<0.01)return null;
  f0=Math.min(Math.max(f0,10),FS*0.49); q=Math.min(Math.max(q,0.1),20);
  const w=2*Math.PI*f0/FS,cw=Math.cos(w),sw=Math.sin(w);
  let b0,b1,b2,a0,a1,a2;
  if(type==1||type==2){
    const A=Math.pow(10,g/40),al=sw*0.5*Math.sqrt((A+1/A)*(1/0.9-1)+2),sA=Math.sqrt(A),t=2*sA*al;
    if(type==1){b0=A*((A+1)-(A-1)*cw+t);b1=2*A*((A-1)-(A+1)*cw);b2=A*((A+1)-(A-1)*cw-t);a0=(A+1)+(A-1)*cw+t;a1=-2*((A-1)+(A+1)*cw);a2=(A+1)+(A-1)*cw-t;}
    else{b0=A*((A+1)+(A-1)*cw+t);b1=-2*A*((A-1)+(A+1)*cw);b2=A*((A+1)+(A-1)*cw-t);a0=(A+1)-(A-1)*cw+t;a1=2*((A-1)-(A+1)*cw);a2=(A+1)-(A-1)*cw-t;}
  }else{
    const A=Math.pow(10,g/40),al=sw/(2*q);
    if(type==0){b0=1+al*A;b1=-2*cw;b2=1-al*A;a0=1+al/A;a1=-2*cw;a2=1-al/A;}
    else if(type==3){b0=1;b1=-2*cw;b2=1;a0=1+al;a1=-2*cw;a2=1-al;}
    else{b0=al;b1=0;b2=-al;a0=1+al;a1=-2*cw;a2=1-al;}
  }
  return{b0:b0/a0,b1:b1/a0,b2:b2/a0,a1:a1/a0,a2:a2/a0};
}
function respDb(c,f){
  if(!c)return 0;
  const w=2*Math.PI*f/FS,c1=Math.cos(w),s1=Math.sin(w),c2=Math.cos(2*w),s2=Math.sin(2*w);
  const nr=c.b0+c.b1*c1+c.b2*c2, ni=-(c.b1*s1+c.b2*s2);
  const dr=1+c.a1*c1+c.a2*c2, di=-(c.a1*s1+c.a2*s2);
  return 10*Math.log10((nr*nr+ni*ni)/Math.max(1e-12,dr*dr+di*di));
}
function hpfDb(f){
  if(!S.hpfOn||S.bypass)return 0;
  const n=S.hpfSlope/6, fc=S.hpfFreq;
  return -10*Math.log10(1+Math.pow(fc/Math.max(f,1),2*n));
}
function loudDb(f){
  if(!S.loud||S.bypass)return 0;
  const d=Math.min(Math.max(0.85-S.vol,0),0.85);
  return respDb(coef(1,100,d*9,0.9),f)+respDb(coef(2,10000,d*6,0.9),f);
}
function totalDb(f){
  if(S.bypass)return 0;
  let d=hpfDb(f)+loudDb(f);
  if(S.eqMode==0){for(let i=0;i<10;i++){const b=S.bands[i];if(b.on)d+=respDb(coef(b.t,b.f,b.g,b.q),f);}}
  else{for(let i=0;i<31;i++){if(S.geq[i])d+=respDb(coef(0,GEQF[i],S.geq[i],4.318),f);}}
  return d;
}
/* ---------- grafik ---------- */
const cv=$('graph'),ctx=cv.getContext('2d');
function fitCv(){const r=cv.getBoundingClientRect(),d=window.devicePixelRatio||1;cv.width=Math.max(300,r.width)*d;cv.height=Math.max(240,r.height)*d;}
function xFor(f,W){return (Math.log(f/20)/Math.log(1000))*W;}
function fForX(x,W){return 20*Math.pow(1000,x/W);}
function yFor(db,H){return (1-(db+15)/30)*H;}
function dbForY(y,H){return (1-y/H)*30-15;}
function draw(){
  if(!S)return;
  const W=cv.width,H=cv.height;ctx.clearRect(0,0,W,H);
  ctx.fillStyle='#04070d';ctx.fillRect(0,0,W,H);
  if(tab=='gate'||tab=='comp')return drawDyn(W,H);
  if(tab=='dlms')return drawDlms(W,H);
  // RTA
  const bw=W/31;
  ctx.fillStyle='rgba(63,138,95,0.38)';
  for(let i=0;i<31;i++){const v=Math.min(Math.max((RTs[i]+60)/60,0),1);ctx.fillRect(i*bw+1,H-v*H,bw-2,v*H);}
  // grid
  ctx.strokeStyle='#182635';ctx.fillStyle='#5f7d99';ctx.lineWidth=1;ctx.font=`${Math.round(H*0.038)}px Arial`;
  [20,50,100,200,500,1000,2000,5000,10000,20000].forEach(f=>{const x=xFor(f,W);ctx.beginPath();ctx.moveTo(x,0);ctx.lineTo(x,H);ctx.stroke();ctx.fillText(fmtF(f),x+2,H-4);});
  [12,6,0,-6,-12].forEach(db=>{const y=yFor(db,H);ctx.strokeStyle=db==0?'#27435f':'#182635';ctx.beginPath();ctx.moveTo(0,y);ctx.lineTo(W,y);ctx.stroke();ctx.fillStyle='#5f7d99';ctx.fillText((db>0?'+':'')+db,3,y-3);});
  // kurva total
  const N=140;ctx.beginPath();
  for(let i=0;i<=N;i++){const f=20*Math.pow(1000,i/N),x=xFor(f,W),y=yFor(Math.max(-15,Math.min(15,totalDb(f))),H);i?ctx.lineTo(x,y):ctx.moveTo(x,y);}
  ctx.strokeStyle='#3f8a5f';ctx.lineWidth=2.2;ctx.stroke();
  ctx.lineTo(W,H);ctx.lineTo(0,H);ctx.closePath();ctx.fillStyle='rgba(63,138,95,0.10)';ctx.fill();
  // kurva band terpilih
  if(S.eqMode==0){const b=S.bands[sel];if(b.on){ctx.beginPath();for(let i=0;i<=N;i++){const f=20*Math.pow(1000,i/N),x=xFor(f,W),y=yFor(Math.max(-15,Math.min(15,respDb(coef(b.t,b.f,b.g,b.q),f))),H);i?ctx.lineTo(x,y):ctx.moveTo(x,y);}ctx.strokeStyle=BCOL[sel];ctx.lineWidth=1.2;ctx.stroke();}}
  // titik node
  if(S.eqMode==0){
    for(let i=0;i<10;i++){const b=S.bands[i],x=xFor(b.f,W),y=yFor(b.g,H);
      ctx.beginPath();ctx.arc(x,y,i==sel?16:13,0,7);ctx.fillStyle=b.on?BCOL[i]:'#555';ctx.fill();
      ctx.lineWidth=i==sel?3:1.5;ctx.strokeStyle=i==sel?'#fff':'#111';ctx.stroke();
      ctx.fillStyle='#0a0a0a';ctx.font='bold 11px Arial';ctx.textAlign='center';ctx.fillText(i+1,x,y+4);ctx.textAlign='left';}
  }else{
    for(let i=0;i<31;i++){const x=xFor(GEQF[i],W),y=yFor(S.geq[i],H);
      ctx.beginPath();ctx.arc(x,y,7,0,7);ctx.fillStyle='#3f8a5f';ctx.fill();ctx.strokeStyle='#111';ctx.stroke();}
  }
}
function drawDyn(W,H){
  const x0=40,x1=W-8,y0=8,y1=H-22;
  const X=v=>x0+(v+60)/60*(x1-x0), Y=v=>y0+(1-(v+60)/60)*(y1-y0);
  // RTA di belakang kurva transfer — spektrum tetap aktif di tab Gate/Comp
  const bw=(x1-x0)/31;ctx.fillStyle='rgba(63,138,95,0.38)';
  for(let i=0;i<31;i++){const v=Math.min(Math.max((RTs[i]+60)/60,0),1);ctx.fillRect(x0+i*bw+1,y1-v*(y1-y0),bw-2,v*(y1-y0));}
  ctx.strokeStyle='#182635';ctx.fillStyle='#5f7d99';ctx.font='10px Arial';
  for(let v=-50;v<0;v+=10){ctx.beginPath();ctx.moveTo(X(v),y0);ctx.lineTo(X(v),y1);ctx.stroke();ctx.beginPath();ctx.moveTo(x0,Y(v));ctx.lineTo(x1,Y(v));ctx.stroke();ctx.fillText(v,X(v)-4,y1+12);ctx.fillText(v,6,Y(v)+3);}
  ctx.strokeStyle='#27435f';ctx.beginPath();ctx.moveTo(X(-60),Y(-60));ctx.lineTo(X(0),Y(0));ctx.stroke();
  ctx.beginPath();
  for(let i=0;i<=120;i++){const inn=-60+60*i/120;let out;
    if(tab=='gate'){out=!S.gateOn?inn:(inn>S.gateThr?inn:-60);}
    else{out=inn;if(S.compOn&&inn>S.compThr)out=S.compThr+(inn-S.compThr)/Math.max(S.compRatio,1);out+=S.compOn?S.compMake:0;if(S.limiter)out=Math.min(out,S.ceiling);}
    out=Math.max(-60,Math.min(3,out));const px=X(inn),py=Y(out);i?ctx.lineTo(px,py):ctx.moveTo(px,py);}
  ctx.strokeStyle='#3f8a5f';ctx.lineWidth=2.2;ctx.stroke();
  if(tab=='comp'&&S.limiter){ctx.strokeStyle='#e03131';ctx.setLineDash([5,4]);ctx.beginPath();ctx.moveTo(x0,Y(S.ceiling));ctx.lineTo(x1,Y(S.ceiling));ctx.stroke();ctx.setLineDash([]);}
}

/* ---------- DLMS ---------- */
const DPEQF=[100,1000,8000];
const DCOL=['#3f8a5f','#e58f8f'];
function f2sl(f){return Math.round(Math.log10(Math.min(Math.max(f,20),20000)/20)/3*1000);}
function sl2f(v){return 20*Math.pow(1000,v/1000);}
function fmtHz(f){f=Math.round(f);return f>=1000?(f/1000).toFixed(2).replace(/\.?0+$/,'')+' kHz':f+' Hz';}
function bq2(kind,f0,q){
  f0=Math.min(Math.max(f0,10),FS*0.49);
  const w=2*Math.PI*f0/FS,cw=Math.cos(w),sw=Math.sin(w),al=sw/(2*q),a0=1+al;
  if(kind=='hp')return{b0:(1+cw)/2/a0,b1:-(1+cw)/a0,b2:(1+cw)/2/a0,a1:-2*cw/a0,a2:(1-al)/a0};
  return{b0:(1-cw)/2/a0,b1:(1-cw)/a0,b2:(1-cw)/2/a0,a1:-2*cw/a0,a2:(1-al)/a0};
}
function bq1(kind,f0){
  const rc=1/(2*Math.PI*f0),dt=1/FS;
  if(kind=='hp'){const a=rc/(rc+dt);return{b0:a,b1:-a,b2:0,a1:-a,a2:0};}
  const a=dt/(rc+dt);return{b0:a,b1:0,b2:0,a1:-(1-a),a2:0};
}
function filtDb(kind,f0,tf,f){
  let d=0;const S2=q=>{d+=respDb(bq2(kind,f0,q),f);},S1=()=>{d+=respDb(bq1(kind,f0),f);};
  if(tf==0){S1();S1();}else if(tf==1){S2(0.7071);S2(0.7071);}
  else if(tf==2){S2(0.7071);S2(0.7071);S2(0.7071);S2(0.7071);}
  else if(tf==3){S2(0.7071);}else if(tf==4){S2(0.5412);S2(1.3065);}
  else{S2(0.3896);S2(0.6266);S2(1.1759);S2(2.5629);}
  return d;
}
function dlmsDb(ch,f){
  const D=S.dlms;if(!D||!D.on)return 0;
  let d=ch.gain;
  if(ch.hpfOn)d+=filtDb('hp',ch.hpfF,ch.tf,f);
  if(ch.lpfOn)d+=filtDb('lp',ch.lpfF,ch.tf,f);
  if(ch.peqOn)for(let j=0;j<3;j++)d+=respDb(coef(0,DPEQF[j],ch.peq[j],1.0),f);
  return d;
}
function drawDlms(W,H){
  const bw=W/31;ctx.fillStyle='rgba(63,138,95,0.38)';
  for(let i=0;i<31;i++){const v=Math.min(Math.max((RTs[i]+60)/60,0),1);ctx.fillRect(i*bw+1,H-v*H,bw-2,v*H);}
  ctx.strokeStyle='#182635';ctx.fillStyle='#5f7d99';ctx.lineWidth=1;ctx.font=`${Math.round(H*0.038)}px Arial`;
  [20,50,100,200,500,1000,2000,5000,10000,20000].forEach(f=>{const x=xFor(f,W);ctx.beginPath();ctx.moveTo(x,0);ctx.lineTo(x,H);ctx.stroke();ctx.fillText(fmtF(f),x+2,H-4);});
  [12,6,0,-6,-12].forEach(db=>{const y=yFor(db,H);ctx.strokeStyle=db==0?'#27435f':'#182635';ctx.beginPath();ctx.moveTo(0,y);ctx.lineTo(W,y);ctx.stroke();ctx.fillStyle='#5f7d99';ctx.fillText((db>0?'+':'')+db,3,y-3);});
  if(!S.dlms)return;
  [1,0].forEach(c=>{
    const ch=S.dlms.ch[c];if(!ch)return;
    const N=140;ctx.beginPath();
    for(let i=0;i<=N;i++){const f=20*Math.pow(1000,i/N),x=xFor(f,W),y=yFor(Math.max(-15,Math.min(15,dlmsDb(ch,f))),H);i?ctx.lineTo(x,y):ctx.moveTo(x,y);}
    if(S.dlms.on&&ch.on){ctx.strokeStyle=DCOL[c];ctx.lineWidth=2.2;ctx.stroke();
      ctx.lineTo(W,H);ctx.lineTo(0,H);ctx.closePath();ctx.fillStyle=DCOL[c]+'22';ctx.fill();}
    else{ctx.strokeStyle='rgba(132,148,173,.55)';ctx.lineWidth=1.6;ctx.setLineDash([5,4]);ctx.stroke();ctx.setLineDash([]);}
  });
  ctx.font='bold 12px Arial';ctx.fillStyle=DCOL[0];ctx.fillText('L',6,16);
  ctx.fillStyle=DCOL[1];ctx.textAlign='right';ctx.fillText('R',W-6,16);ctx.textAlign='left';
}
function tgTxt(id,on){tgl(id,on);$(id).textContent=on?'ON':'OFF';}
function fillDlms(){
  const D=S.dlms;if(!D)return;
  tgTxt('dlmsOn',D.on);$('pg-dlms').classList.toggle('dlmsoff',!D.on);
  tgTxt('inOnA',D.inOnA);tgTxt('inOnB',D.inOnB);
  $('inGainA').value=D.inGainA;$('inGainAV').textContent=(+D.inGainA).toFixed(1)+' dB';
  $('inGainB').value=D.inGainB;$('inGainBV').textContent=(+D.inGainB).toFixed(1)+' dB';
  $('inDlyA').value=D.inDlyA;$('inDlyB').value=D.inDlyB;
  for(let c=0;c<2;c++){
    const ch=D.ch[c];if(!ch)continue;
    $('dch'+c).classList.toggle('off',!ch.on);
    tgTxt('d'+c+'_on',ch.on);
    $('dch'+c).querySelectorAll('.srcbtn').forEach((b,i)=>b.classList.toggle('on',ch.src==i));
    tgTxt('d'+c+'_hpfOn',ch.hpfOn);tgTxt('d'+c+'_lpfOn',ch.lpfOn);tgTxt('d'+c+'_peqOn',ch.peqOn);
    const hs=$('d'+c+'_hpfF'),ls=$('d'+c+'_lpfF');
    hs.value=f2sl(ch.hpfF);ls.value=f2sl(ch.lpfF);hs.disabled=!ch.hpfOn;ls.disabled=!ch.lpfOn;
    $('d'+c+'_hpfV').textContent=fmtHz(ch.hpfF);$('d'+c+'_lpfV').textContent=fmtHz(ch.lpfF);
    $('d'+c+'_tf').value=ch.tf;
    for(let j=0;j<3;j++){const s=$('d'+c+'_p'+j);s.value=ch.peq[j];s.disabled=!ch.peqOn;$('d'+c+'_p'+j+'v').textContent=DPEQF[j]+' · '+(+ch.peq[j]).toFixed(1);}
    $('d'+c+'_gain').value=ch.gain;$('d'+c+'_gainV').textContent=(+ch.gain).toFixed(1)+' dB';
    $('d'+c+'_dly').value=ch.dly;$('d'+c+'_dlyV').textContent=(+ch.dly).toFixed(1)+' ms';
    $('d'+c+'_lim').value=ch.lim;$('d'+c+'_limV').textContent=(+ch.lim).toFixed(1)+' dB';
    const pb=$('d'+c+'_pol');pb.textContent=ch.pol?'180°':'0°';pb.classList.toggle('on',!!ch.pol);
  }
}
function bindDlms(){
  const D=()=>S&&S.dlms;
  $('dlmsOn').onclick=()=>{const d=D();if(!d)return;d.on=d.on?0:1;tgTxt('dlmsOn',d.on);$('pg-dlms').classList.toggle('dlmsoff',!d.on);send({dlmsOn:d.on});draw();};
  const inTgl=(id,key)=>{$(id).onclick=()=>{const d=D();if(!d)return;d[key]=d[key]?0:1;tgTxt(id,d[key]);send({[key]:d[key]});};};
  inTgl('inOnA','inOnA');inTgl('inOnB','inOnB');
  const inSl=(id,vid,key)=>{$(id).oninput=()=>{const d=D();if(!d)return;d[key]=+$(id).value;$(vid).textContent=d[key].toFixed(1)+' dB';sendSoon({[key]:d[key]});};};
  inSl('inGainA','inGainAV','inGainA');inSl('inGainB','inGainBV','inGainB');
  const inNum=(id,key)=>{$(id).onchange=()=>{const d=D();if(!d)return;d[key]=+$(id).value;send({[key]:d[key]});};};
  inNum('inDlyA','inDlyA');inNum('inDlyB','inDlyB');
  for(let c=0;c<2;c++){
    const K=k=>'c'+c+'_'+k, ID=k=>'d'+c+'_'+k;
    const ch=()=>{const d=D();return d?d.ch[c]:null;};
    $(ID('on')).onclick=()=>{const h=ch();if(!h)return;h.on=h.on?0:1;tgTxt(ID('on'),h.on);$('dch'+c).classList.toggle('off',!h.on);send({[K('on')]:h.on});draw();};
    $('dch'+c).querySelectorAll('.srcbtn').forEach((b,i)=>b.onclick=()=>{const h=ch();if(!h)return;h.src=i;$('dch'+c).querySelectorAll('.srcbtn').forEach(x=>x.classList.toggle('on',x==b));send({[K('src')]:i});});
    const fTgl=(fid,key,slid)=>{$(fid).onclick=()=>{const h=ch();if(!h)return;h[key]=h[key]?0:1;tgTxt(fid,h[key]);if(slid)$(slid).disabled=!h[key];send({[K(key)]:h[key]});draw();};};
    fTgl(ID('hpfOn'),'hpfOn',ID('hpfF'));fTgl(ID('lpfOn'),'lpfOn',ID('lpfF'));
    $(ID('peqOn')).onclick=()=>{const h=ch();if(!h)return;h.peqOn=h.peqOn?0:1;tgTxt(ID('peqOn'),h.peqOn);for(let j=0;j<3;j++)$(ID('p'+j)).disabled=!h.peqOn;send({[K('peqOn')]:h.peqOn});draw();};
    const fSl=(fid,vid,key)=>{$(fid).oninput=()=>{const h=ch();if(!h)return;h[key]=Math.round(sl2f(+$(fid).value));$(vid).textContent=fmtHz(h[key]);sendSoon({[K(key)]:h[key]});draw();};};
    fSl(ID('hpfF'),ID('hpfV'),'hpfF');fSl(ID('lpfF'),ID('lpfV'),'lpfF');
    $(ID('tf')).onchange=()=>{const h=ch();if(!h)return;h.tf=+$(ID('tf')).value;send({[K('tf')]:h.tf});draw();};
    for(let j=0;j<3;j++){const pid=ID('p'+j);$(pid).oninput=()=>{const h=ch();if(!h)return;h.peq[j]=+$(pid).value;$(pid+'v').textContent=DPEQF[j]+' · '+h.peq[j].toFixed(1);sendSoon({[K('p'+j)]:h.peq[j]});draw();};}
    const nSl=(fid,vid,key,unit,dr)=>{$(fid).oninput=()=>{const h=ch();if(!h)return;h[key]=+$(fid).value;$(vid).textContent=h[key].toFixed(1)+unit;sendSoon({[K(key)]:h[key]});if(dr)draw();};};
    $(ID('gain')).oninput=()=>{const h=ch();if(!h)return;h.gain=+$(ID('gain')).value;$(ID('gainV')).textContent=h.gain.toFixed(1)+' dB';const sf=$('sf'+c);if(sf){sf.value=h.gain;sf.closest('.strip').querySelector('.val').textContent=h.gain.toFixed(1);}sendSoon({[K('gain')]:h.gain});draw();};
    nSl(ID('dly'),ID('dlyV'),'dly',' ms',false);nSl(ID('lim'),ID('limV'),'lim',' dB',false);
    $(ID('pol')).onclick=()=>{const h=ch();if(!h)return;h.pol=h.pol?0:1;const pb=$(ID('pol'));pb.textContent=h.pol?'180°':'0°';pb.classList.toggle('on',!!h.pol);send({[K('pol')]:h.pol});};
  }
}

/* ---------- interaksi grafik ---------- */
let drag=null,lastSend=0,pendSend=null;
function cvPos(e){const r=cv.getBoundingClientRect();if(!r.width||!r.height)return{x:0,y:0};return{x:(e.clientX-r.left)*(cv.width/r.width),y:(e.clientY-r.top)*(cv.height/r.height)};}
function nodeAt(p){
  if(!S||tab!='eq')return -1;
  if(S.eqMode==0){for(let i=0;i<10;i++){const b=S.bands[i],x=xFor(b.f,cv.width),y=yFor(b.g,cv.height);if(Math.hypot(p.x-x,p.y-y)<26)return i;}}
  else{for(let i=0;i<31;i++){const x=xFor(GEQF[i],cv.width),y=yFor(S.geq[i],cv.height);if(Math.hypot(p.x-x,p.y-y)<20)return 100+i;}}
  return -1;
}
function sendSoon(o){
  const now=performance.now();
  if(now-lastSend>50){lastSend=now;send(o);}
  else{pendSend=o;clearTimeout(sendSoon.t);sendSoon.t=setTimeout(()=>{if(pendSend){send(pendSend);pendSend=null;lastSend=performance.now();}},60);}
}
cv.addEventListener('pointerdown',e=>{
  const id=nodeAt(cvPos(e));if(id<0)return;
  drag=id;cv.setPointerCapture(e.pointerId);
  if(id<100){sel=id;fillChips();fillEditor();draw();}
});
cv.addEventListener('pointermove',e=>{
  if(drag===null||drag===undefined||!S)return;
  const p=cvPos(e);
  if(drag<100){const b=S.bands[drag];
    b.f=Math.round(Math.min(20000,Math.max(20,fForX(p.x,cv.width))));
    b.g=Math.round(Math.min(15,Math.max(-15,dbForY(p.y,cv.height)))*10)/10;
    sendSoon({['b'+drag+'_f']:b.f,['b'+drag+'_g']:b.g});
    fillEditor();fillStrips();draw();
  }else{const i=drag-100;
    S.geq[i]=Math.round(Math.min(12,Math.max(-12,dbForY(p.y,cv.height)*0.8))*10)/10;
    sendSoon({['g'+i]:S.geq[i]});fillStrips();draw();
  }
});
cv.addEventListener('pointerup',()=>{drag=null;});
cv.addEventListener('pointercancel',()=>{drag=null;});
cv.addEventListener('dblclick',e=>{
  const id=nodeAt(cvPos(e));
  if(id>=0&&id<100){const b=S.bands[id];b.on=!b.on;send({['b'+id+'_on']:b.on?1:0});fillEditor();fillChips();fillStrips();draw();}
});
cv.addEventListener('wheel',e=>{
  const id=nodeAt(cvPos(e));
  if(id>=0&&id<100){e.preventDefault();const b=S.bands[id];b.q=Math.round(Math.min(16,Math.max(0.4,b.q*(e.deltaY<0?1.15:0.87)))*10)/10;send({['b'+id+'_q']:b.q});if(id==sel)fillEditor();draw();}
},{passive:false});
/* ---------- builders ---------- */
function fillChips(){
  const w=$('chips');w.innerHTML='';
  S.bands.forEach((b,i)=>{
    const c=document.createElement('button');c.className='chip'+(i==sel?' sel':'')+(b.on?'':' off');
    c.style.borderTopColor=BCOL[i];c.textContent='B'+(i+1)+' · '+fmtF(b.f);
    c.onclick=()=>{sel=i;fillChips();fillEditor();draw();};
    w.appendChild(c);
  });
}
function fillStrips(){
  const w=$('strips');w.innerHTML='';
  if(tab=='dlms'&&S.dlms&&S.dlms.ch){
    S.dlms.ch.forEach((h,c)=>{
      const d=document.createElement('div');d.className='strip dls'+(h.on?'':' off');
      d.innerHTML=`<div class="scrib" style="background:${DCOL[c]}">${c==0?'L':'R'}</div><div class="val">${(+h.gain).toFixed(1)}</div>`;
      const f=document.createElement('input');f.type='range';f.className='vfader';f.id='sf'+c;f.min=-60;f.max=12;f.step=0.5;f.value=h.gain;
      f.oninput=()=>{h.gain=+f.value;d.querySelector('.val').textContent=h.gain.toFixed(1);const cs=$('d'+c+'_gain');if(cs){cs.value=h.gain;$('d'+c+'_gainV').textContent=h.gain.toFixed(1)+' dB';}sendSoon({['c'+c+'_gain']:h.gain});draw();};
      const on=document.createElement('button');on.className='tgl onled'+(h.on?' on':'');on.textContent='ON';
      on.onclick=()=>{h.on=h.on?0:1;send({['c'+c+'_on']:h.on});fillStrips();fillDlms();draw();};
      d.appendChild(f);d.appendChild(on);w.appendChild(d);
    });
    return;
  }
  if(S.eqMode==0){
    S.bands.forEach((b,i)=>{
      const d=document.createElement('div');d.className='strip'+(i==sel?' sel':'')+(b.on?'':' off');
      d.innerHTML=`<div class="scrib" style="background:${BCOL[i]}">B${i+1}</div><div class="val">${b.g.toFixed(1)}</div>`;
      const f=document.createElement('input');f.type='range';f.className='vfader';f.min=-15;f.max=15;f.step=0.1;f.value=b.g;
      f.oninput=()=>{b.g=+f.value;d.querySelector('.val').textContent=b.g.toFixed(1);sendSoon({['b'+i+'_g']:b.g});if(i==sel)fillEditor();draw();};
      f.onclick=()=>{sel=i;fillChips();fillEditor();};
      const on=document.createElement('button');on.className='tgl onled'+(b.on?' on':'');on.textContent='ON';
      on.onclick=()=>{b.on=!b.on;send({['b'+i+'_on']:b.on?1:0});fillStrips();fillChips();fillEditor();draw();};
      d.appendChild(f);d.appendChild(on);w.appendChild(d);
    });
  }else{
    GEQF.forEach((fr,i)=>{
      const d=document.createElement('div');d.className='strip gq';
      d.innerHTML=`<div class="scrib" style="background:#8a6d00">${fmtF(fr)}</div><div class="val">${S.geq[i].toFixed(1)}</div>`;
      const f=document.createElement('input');f.type='range';f.className='vfader';f.min=-12;f.max=12;f.step=0.5;f.value=S.geq[i];
      f.oninput=()=>{S.geq[i]=+f.value;d.querySelector('.val').textContent=S.geq[i].toFixed(1);sendSoon({['g'+i]:S.geq[i]});draw();};
      d.appendChild(f);w.appendChild(d);
    });
  }
}
function fillEditor(){
  const b=S.bands[sel];if(!b)return;
  $('ed_name').textContent='BAND '+(sel+1)+' — '+fmtF(b.f)+' Hz';
  tgl('ed_on',b.on);$('ed_t').value=b.t;
  $('ed_g').value=b.g;$('ed_g_v').textContent=b.g.toFixed(1);
  $('ed_f').value=Math.round(b.f);
  $('ed_fr').value=Math.round(Math.log(b.f/20)/Math.log(1000)*1000);
  $('ed_q').value=b.q;$('ed_q_v').textContent=b.q.toFixed(1);
  tgl('ed_dyn',b.dyn);$('ed_dthr').value=b.dthr;$('ed_drng').value=b.drng;
}
function fillSlots(){
  const w=$('slots');w.innerHTML='';
  for(let i=0;i<10;i++){
    const d=document.createElement('div');d.className='chip'+(S.slots[i]?'':' off');d.style.flex='0 0 104px';d.style.padding='4px';
    d.innerHTML=`<div style="font-size:11px;margin-bottom:3px">Slot ${i+1}${S.slots[i]?'':' (kosong)'}</div>`;
    const l=document.createElement('button');l.className='tgl';l.textContent='Muat';l.style.marginRight='3px';
    l.onclick=async()=>{await fetch('/api/preset',{method:'POST',headers:{'Content-Type':'application/x-www-form-urlencoded'},body:new URLSearchParams({name:'user:'+i})});load();};
    const sv=document.createElement('button');sv.className='tgl';sv.textContent='Simpan';
    sv.onclick=async()=>{if(confirm('Simpan setting sekarang ke Slot '+(i+1)+'?')){await fetch('/api/save',{method:'POST',headers:{'Content-Type':'application/x-www-form-urlencoded'},body:new URLSearchParams({slot:i})});load();}};
    d.appendChild(l);d.appendChild(sv);w.appendChild(d);
  }
}
/* ---------- settings <-> objek datar ---------- */
function capture(){
  const o={preamp:S.preamp,trimL:S.trimL,trimR:S.trimR,polL:S.polL,polR:S.polR,hpfOn:S.hpfOn,hpfFreq:S.hpfFreq,hpfSlope:S.hpfSlope,eqMode:S.eqMode,bypass:S.bypass,loud:S.loud,gateOn:S.gateOn,gateThr:S.gateThr,gateAtk:S.gateAtk,gateRel:S.gateRel,compOn:S.compOn,compThr:S.compThr,compRatio:S.compRatio,compAtk:S.compAtk,compRel:S.compRel,compMake:S.compMake,limiter:S.limiter,ceiling:S.ceiling,xfOn:S.xfOn,xfAmt:S.xfAmt,width:S.width,mono:S.mono,balance:S.balance,dlyL:S.dlyL,dlyR:S.dlyR,vol:S.vol,mute:S.mute,dim:S.dim,volMax:S.volMax,autoHr:S.autoHr};
  S.bands.forEach((b,i)=>{o['b'+i+'_on']=b.on?1:0;o['b'+i+'_t']=b.t;o['b'+i+'_f']=b.f;o['b'+i+'_g']=b.g;o['b'+i+'_q']=b.q;o['b'+i+'_dyn']=b.dyn?1:0;o['b'+i+'_dthr']=b.dthr;o['b'+i+'_drng']=b.drng;});
  S.geq.forEach((g,i)=>{o['g'+i]=g;});
  return o;
}
/* ---------- fill semua kontrol ---------- */
function fill(){
  $('netpill').textContent=S.net;$('netinfo').textContent=S.net+' · http://mbahdac.local';
  $('fwinfo').textContent='S3 Air '+S.fw;
  $('ringinfo').textContent='underrun '+S.ur+' · overflow '+S.ov;
  $('eqpeak').textContent=(S.eqPeak>=0?'+':'')+S.eqPeak+' dB';
  $('pre').value=S.preamp;$('pre_v').textContent=S.preamp.toFixed(1)+' dB';
  $('trimL').value=S.trimL;$('trimR').value=S.trimR; tgl('polL',S.polL);tgl('polR',S.polR);
  tgl('hpfOn',S.hpfOn);$('hpfFreq').value=S.hpfFreq;$('hpfSlope').value=S.hpfSlope;
  tgl('modePeq',S.eqMode==0);tgl('modeGeq',S.eqMode==1);tgl('bypass',S.bypass);
  $('chips').style.display=S.eqMode==0?'':'none';
  document.querySelector('#pg-eq .boxes').style.display=S.eqMode==0?'':'none';
  tgl('loud',S.loud);tgl('autoHr',S.autoHr);
  tgl('gateOn',S.gateOn);$('gateThr').value=S.gateThr;$('gateAtk').value=S.gateAtk;$('gateRel').value=S.gateRel;
  tgl('compOn',S.compOn);$('compThr').value=S.compThr;$('compRatio').value=S.compRatio;$('compAtk').value=S.compAtk;$('compRel').value=S.compRel;$('compMake').value=S.compMake;
  tgl('limiter',S.limiter);$('ceiling').value=S.ceiling;
  $('dlyL').value=S.dlyL;$('dlyR').value=S.dlyR;
  $('dlyCm').textContent=((S.dlyR-S.dlyL)*34.3).toFixed(0)+' cm';
  $('width').value=S.width;$('width_v').textContent=S.width.toFixed(2);
  $('balance').value=S.balance;$('balance_v').textContent=S.balance.toFixed(2);
  tgl('xfOn',S.xfOn);$('xfAmt').value=S.xfAmt;$('xfAmt_v').textContent=S.xfAmt.toFixed(2);
  tgl('mono',S.mono);tgl('dim',S.dim);
  $('volMax').value=S.volMax;$('volMax_v').textContent=Math.round(S.volMax*100)+'%';
  $('vol').value=S.vol;$('volDb').textContent=volDb(S.vol)+' dB';$('tvol').value=S.vol;$('tvolDb').textContent=volDb(S.vol)+' dB';tgl('tmute',S.mute);
  tgl('mute',S.mute);
  fillChips();fillEditor();fillStrips();fillSlots();fillDlms();draw();
}
/* ---------- binding ---------- */
function bindTgl(id,key,after){$(id).onclick=()=>{S[key]=S[key]?0:1;tgl(id,S[key]);send({[key]:S[key]});if(after)after();draw();};}
function bindNum(id,key,after){$(id).onchange=()=>{S[key]=+$(id).value;send({[key]:S[key]});if(after)after();draw();};}
function bind(){
  document.querySelectorAll('.tabbtn').forEach(b=>b.onclick=()=>{
    tab=b.dataset.tab;document.body.dataset.tab=tab;
    document.querySelectorAll('.tabbtn').forEach(x=>x.classList.toggle('act',x==b));
    document.querySelectorAll('.pg').forEach(p=>p.classList.toggle('act',p.id=='pg-'+tab||(tab=='sys'&&p.id=='pg-sys')));
    if(S)fillStrips();
    draw();
  });
  $('pre').oninput=()=>{S.preamp=+$('pre').value;$('pre_v').textContent=S.preamp.toFixed(1)+' dB';sendSoon({preamp:S.preamp});};
  bindNum('trimL','trimL');bindNum('trimR','trimR');
  bindTgl('polL','polL');bindTgl('polR','polR');
  bindTgl('hpfOn','hpfOn');bindNum('hpfFreq','hpfFreq');bindNum('hpfSlope','hpfSlope');
  $('modePeq').onclick=()=>{S.eqMode=0;send({eqMode:0});fill();};
  $('modeGeq').onclick=()=>{S.eqMode=1;send({eqMode:1});fill();};
  bindTgl('bypass','bypass');
  bindTgl('loud','loud');bindTgl('autoHr','autoHr');
  bindTgl('gateOn','gateOn');bindNum('gateThr','gateThr');bindNum('gateAtk','gateAtk');bindNum('gateRel','gateRel');
  bindTgl('compOn','compOn');bindNum('compThr','compThr');bindNum('compRatio','compRatio');bindNum('compAtk','compAtk');bindNum('compRel','compRel');bindNum('compMake','compMake');
  bindTgl('limiter','limiter');bindNum('ceiling','ceiling');
  bindNum('dlyL','dlyL',()=>$('dlyCm').textContent=((S.dlyR-S.dlyL)*34.3).toFixed(0)+' cm');
  bindNum('dlyR','dlyR',()=>$('dlyCm').textContent=((S.dlyR-S.dlyL)*34.3).toFixed(0)+' cm');
  $('width').oninput=()=>{S.width=+$('width').value;$('width_v').textContent=S.width.toFixed(2);sendSoon({width:S.width});};
  $('balance').oninput=()=>{S.balance=+$('balance').value;$('balance_v').textContent=S.balance.toFixed(2);sendSoon({balance:S.balance});};
  bindTgl('xfOn','xfOn');
  $('xfAmt').oninput=()=>{S.xfAmt=+$('xfAmt').value;$('xfAmt_v').textContent=S.xfAmt.toFixed(2);sendSoon({xfAmt:S.xfAmt});};
  bindTgl('mono','mono');bindTgl('dim','dim');
  $('volMax').oninput=()=>{S.volMax=+$('volMax').value;$('volMax_v').textContent=Math.round(S.volMax*100)+'%';sendSoon({volMax:S.volMax});};
  $('vol').oninput=()=>{S.vol=+$('vol').value;$('volDb').textContent=volDb(S.vol)+' dB';$('tvol').value=S.vol;$('tvolDb').textContent=volDb(S.vol)+' dB';sendSoon({vol:S.vol});draw();};
  $('tvol').oninput=()=>{S.vol=+$('tvol').value;$('tvolDb').textContent=volDb(S.vol)+' dB';$('vol').value=S.vol;$('volDb').textContent=volDb(S.vol)+' dB';sendSoon({vol:S.vol});draw();};
  $('tmute').onclick=()=>$('mute').click();
  bindTgl('mute','mute',()=>tgl('tmute',S.mute));
  // editor band
  $('ed_on').onclick=()=>{const b=S.bands[sel];b.on=!b.on;tgl('ed_on',b.on);send({['b'+sel+'_on']:b.on?1:0});fillChips();fillStrips();draw();};
  $('ed_t').onchange=()=>{const b=S.bands[sel];b.t=+$('ed_t').value;send({['b'+sel+'_t']:b.t});draw();};
  $('ed_g').oninput=()=>{const b=S.bands[sel];b.g=+$('ed_g').value;$('ed_g_v').textContent=b.g.toFixed(1);sendSoon({['b'+sel+'_g']:b.g});fillStrips();draw();};
  $('ed_f').onchange=()=>{const b=S.bands[sel];b.f=Math.min(20000,Math.max(20,+$('ed_f').value));send({['b'+sel+'_f']:b.f});fillEditor();fillChips();draw();};
  $('ed_fr').oninput=()=>{const b=S.bands[sel];b.f=Math.round(20*Math.pow(1000,+$('ed_fr').value/1000));$('ed_f').value=b.f;sendSoon({['b'+sel+'_f']:b.f});fillChips();draw();};
  $('ed_q').oninput=()=>{const b=S.bands[sel];b.q=+$('ed_q').value;$('ed_q_v').textContent=b.q.toFixed(1);sendSoon({['b'+sel+'_q']:b.q});draw();};
  $('ed_dyn').onclick=()=>{const b=S.bands[sel];b.dyn=b.dyn?0:1;tgl('ed_dyn',b.dyn);send({['b'+sel+'_dyn']:b.dyn});};
  $('ed_dthr').onchange=()=>{const b=S.bands[sel];b.dthr=+$('ed_dthr').value;send({['b'+sel+'_dthr']:b.dthr});};
  $('ed_drng').onchange=()=>{const b=S.bands[sel];b.drng=+$('ed_drng').value;send({['b'+sel+'_drng']:b.drng});};
  // preset
  const fw=$('factory');
  [['flat','Flat'],['speaker','Speaker DIY'],['vokal','Vokal'],['bass','Bass'],['malam','Malam']].forEach(([n,l])=>{
    const b=document.createElement('button');b.className='tgl';b.textContent=l;
    b.onclick=async()=>{await fetch('/api/preset',{method:'POST',headers:{'Content-Type':'application/x-www-form-urlencoded'},body:new URLSearchParams({name:n})});load();};
    fw.appendChild(b);
  });
  $('abBtn').onclick=async()=>{
    if(!abIsB){snapA=capture();if(!snapB)snapB=JSON.parse(JSON.stringify(snapA));await send(snapB);abIsB=true;$('abBtn').textContent='Ke A';$('abInfo').textContent='B aktif';}
    else{snapB=capture();await send(snapA);abIsB=false;$('abBtn').textContent='Ke B';$('abInfo').textContent='A aktif';}
    setTimeout(load,200);
  };
  const doExport=()=>{
    const blob=new Blob([JSON.stringify(capture(),null,1)],{type:'application/json'});
    const a=document.createElement('a');a.href=URL.createObjectURL(blob);a.download='mbahdac-setting.json';a.click();URL.revokeObjectURL(a.href);
  };
  $('expBtn').onclick=doExport;
  $('impBtn').onclick=()=>$('impFile').click();
  $('impFile').onchange=()=>{const f=$('impFile').files[0];if(!f)return;const r=new FileReader();r.onload=async()=>{try{await send(JSON.parse(r.result));load();}catch(e){alert('File tidak valid');}};r.readAsText(f);};
  const doClip=()=>send({clipReset:1});
  $('clipResetBtn').onclick=doClip;
  $('wifiBtn').onclick=()=>{if(confirm('Alat restart dan gabung WiFi rumah dengan data baru. Lanjut?'))fetch('/api/wifi',{method:'POST',headers:{'Content-Type':'application/x-www-form-urlencoded'},body:new URLSearchParams({ssid:$('ssid').value,pass:$('wpass').value})});};
  bindDlms();
}
/* ---------- data ---------- */
async function load(){try{S=await (await fetch('/api/state')).json();fill();}catch(e){}}
setInterval(async()=>{try{const m=await (await fetch('/api/meter')).json();
  $('mLf').style.height=Math.min(100,m.l*100)+'%';$('mRf').style.height=Math.min(100,m.r*100)+'%';$('mMf').style.height=Math.min(100,Math.max(m.l,m.r)*100)+'%';$('tLf').style.height=Math.min(100,m.l*100)+'%';$('tRf').style.height=Math.min(100,m.r*100)+'%';
  $('grbar').style.width=Math.min(100,Math.max(0,-m.gr/24*100))+'%';$('gr_v').textContent=m.gr+' dB';$('clipct').textContent=m.clip;}catch(e){}},200);
let rtBusy=false;
setInterval(async()=>{if(rtBusy)return;rtBusy=true;try{const r=await (await fetch('/api/rta')).json();RT=r.rta;}catch(e){}rtBusy=false;},150);
setInterval(()=>{let moved=false;for(let i=0;i<31;i++){const d=RT[i]-RTs[i];if(Math.abs(d)>0.05){RTs[i]+=d*(d>0?0.55:0.16);moved=true;}}if(moved)draw();},50);
window.addEventListener('resize',()=>{fitCv();draw();});
let sysBusy=false;
setInterval(async()=>{if(tab!='sys'||sysBusy)return;sysBusy=true;try{const st=await (await fetch('/api/state')).json();
  $('netinfo').textContent=st.net+' · http://mbahdac.local';$('fwinfo').textContent='S3 Air '+st.fw;
  $('ringinfo').textContent='underrun '+st.ur+' · overflow '+st.ov;$('eqpeak').textContent=(st.eqPeak>=0?'+':'')+st.eqPeak+' dB';}catch(e){}sysBusy=false;},2000);
setInterval(()=>{if(!S)load();},3000);
fitCv();bind();load();
</script></body></html>
)HTML";

static void setupWeb() {
  server.on("/", []() { server.send_P(200, "text/html", PAGE); });
  server.on("/api/state", []() { server.send(200, "application/json", stateJson()); });
  server.on("/api/meter", []() {
    String j = "{\"l\":" + String(meterPeakL, 3) + ",\"r\":" + String(meterPeakR, 3) +
               ",\"lrms\":" + String(meterRmsL, 3) + ",\"rrms\":" + String(meterRmsR, 3) +
               ",\"gr\":" + String(grMeterDb, 1) + ",\"clip\":" + String(clipCount) + "}";
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
    auto has = [&](const char *k) { return server.hasArg(k); };
    auto f = [&](const char *k) { return server.arg(k).toFloat(); };
    auto on = [&](const char *k) { return server.arg(k) == "1"; };
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
    int dlmsHit = 0;
    for (int a = 0; a < server.args(); a++)
      if (dlmsSetKey(server.argName(a), server.arg(a).toFloat(), server.arg(a) == "1")) dlmsHit++;
    if (dlmsHit) Serial0.printf("[dbg] set DLMS x%d diproses, on=%d\n", dlmsHit, dlms.on ? 1 : 0);
    if (coef) coefDirty = true;
    if (dyn) rebuildDynCoefs();
    server.send(200, "application/json", "{\"ok\":1}");
  });
  server.on("/api/preset", HTTP_POST, []() {
    String n = server.arg("name");
    bool ok = false;
    if (n.startsWith("user:")) ok = loadUserPreset(n.substring(5).toInt());
    else ok = setFactoryPreset(n.c_str(), params);
    if (ok) { coefDirty = true; armDip = true; rebuildDynCoefs(); }
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
      if (ok) { coefDirty = true; armDip = true; rebuildDynCoefs(); }
      Serial0.println(ok ? "OK" : "ERR preset"); continue;
    }
    if (ln.startsWith("upreset ")) {
      bool ok = loadUserPreset(ln.substring(8).toInt());
      if (ok) { coefDirty = true; armDip = true; rebuildDynCoefs(); }
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
  }
}

// ============================================================
void setup() {
  Serial0.begin(115200);
  delay(300);
  setFlat(params);
  setDlmsDefaults();
  loadDlmsNvs();
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
  Serial0.println("[USB] " FW_VERSION " siap — tunggu Windows memilih 'MbahDAC USB'.");
}

void loop() {
  wifiPoll();
  serialPoll();
  static uint32_t lastDlmsSave = 0;
  if (dlmsSaveDirty && millis() - lastDlmsSave > 2000) {
    lastDlmsSave = millis(); dlmsSaveDirty = false; saveDlmsNvs();
  }
  static uint32_t lastRta = 0;
  if (millis() - lastRta > 100) { lastRta = millis(); computeRTA(); }
  delay(2);
}
