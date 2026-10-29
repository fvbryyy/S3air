// ============================================================
//  MbahDAC FINAL v2.0 — USB DAC + DSP Console
//  ESP32-S3 + PCM5102 | USB Audio Class 1 (48 kHz / 16-bit)
//  Kontrol: web konsol gaya mixer digital via WiFi rumah (STA)
//
//  PENTING — setting Arduino IDE (terbukti jalan di Windows):
//    Board            : ESP32S3 Dev Module
//    USB Mode         : USB-OTG (TinyUSB)
//    USB CDC On Boot  : DISABLED   <-- WAJIB. CDC+Audio komposit
//                       membuat driver usbaudio.sys Windows gagal
//                       start (Code 10). Debug serial via Serial0
//                       (port COM/UART satunya), seperti DrumCore.
//    PSRAM            : OPI PSRAM
//    Flash Size       : 16MB
//    Core             : esp32 3.3.12
//
//  Pin I2S (rakitan user): BCK = GPIO42, LRCK = GPIO21, DIN = GPIO2
//  PCM5102 mode PLL (SCK ke GND), tanpa MCLK.
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
#define FW_VERSION "v3.0 Gabungan"

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
USBAudioCard uac(48000, UAC_BPS_24, UAC_SPK_STEREO, UAC_MIC_NONE); // 24-bit dalam slot 32-bit (ala DQ-DSP)
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
//  RTOS: ring SPSC — callback USB ringan, DSP di task khusus Core 1
// ============================================================
#define RING_FRAMES 4096
static int32_t ringIn[RING_FRAMES * 2], ringOut[RING_FRAMES * 2];
static volatile uint32_t ringInW = 0, ringInR = 0, ringOutW = 0, ringOutR = 0;
static volatile uint32_t statUnderrun = 0, statOverflow = 0;
static TaskHandle_t dspTaskHandle = NULL;
static int32_t dspBlock[512 * 2];

static void processBlock(int32_t *s, int frames) {
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
    // laju kontrol (tiap 48 sampel): luncurkan frekuensi/Q/HPF secara rasio, bangun ulang bila berubah
    if (++ctrlCount >= 48) {
      ctrlCount = 0;
      for (int i = 0; i < NUM_BANDS; i++) {
        smBandFreq[i] *= 1.0f + 0.09f * (params.band[i].freq / smBandFreq[i] - 1.0f);
        smBandQ[i] += (params.band[i].q - smBandQ[i]) * 0.09f;
      }
      smHpfFreq *= 1.0f + 0.09f * (params.hpfFreq / smHpfFreq - 1.0f);
      if (filtersChanged()) {
        rebuildCoefficients();
        mb = mainBank[activeBank]; gb = geqBank[activeBank];
        if (armDip) { xfadeRemain = XFADE_LEN; armDip = false; }
      }
    }

    float L = s[2 * n] * (1.0f / 2147483648.0f), R = s[2 * n + 1] * (1.0f / 2147483648.0f);
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

    float aL = fabsf(L), aR = fabsf(R);
    if (aL > meterPeakL) meterPeakL = aL;
    if (aR > meterPeakR) meterPeakR = aR;
    sumL += L * L; sumR += R * R;
    if (aL >= 0.999f || aR >= 0.999f) clipCount++;
    rtaBuf[rtaWr] = (L + R) * 0.5f; rtaWr = (rtaWr + 1) & 1023;

    float oL = constrain(L, -1.0f, 1.0f), oR = constrain(R, -1.0f, 1.0f);
    s[2 * n] = (int32_t)(oL * 2147483647.0f);
    s[2 * n + 1] = (int32_t)(oR * 2147483647.0f);
  }
  if (frames > 0) {
    meterRmsL = sqrtf(sumL / frames); meterRmsR = sqrtf(sumR / frames);
  }
}

static void onSpkData(void *data, uint16_t len) {
  uac.applyVolume(data, len);
  int32_t *s = (int32_t *)data;
  int frames = len / 8;
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
    float acc = 0; int cnt = 0;
    for (int k = kLo; k < kHi; k++) { acc += sqrtf(re[k] * re[k] + im[k] * im[k]); cnt++; }
    float mag = cnt ? acc / cnt / 256.0f : 0;
    float db = 20.0f * log10f(mag + 1e-6f);
    float newLevel = constrain(db, -60.0f, 0.0f);
    rtaSmooth[b] += (newLevel - rtaSmooth[b]) * 0.15f;
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
  prefs.begin("mbahdac2", false);
  size_t w = prefs.putBytes(key, &params, sizeof(params));
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
  return true;
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
  j += ",\"slots\":[";
  for (int i = 0; i < NUM_USER_PRESETS; i++) { if (i) j += ","; j += String(userPresetExists(i) ? 1 : 0); }
  j += "]}";
  return j;
}

// Halaman web disisipkan saat build (webpage.html -> PAGE)
static const char PAGE[] PROGMEM = R"HTML(<!DOCTYPE html>
<html lang="id"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1,maximum-scale=1,user-scalable=no">
<title>MbahDAC — Digital Console</title>
<style>
*{box-sizing:border-box;margin:0;padding:0}
:root{--bg:#0a0c0f;--panel:#13161b;--panel2:#0e1014;--line:#232830;--txt:#e0e7ef;--dim:#7f8c9a;--cy:#2ea3e0;--am:#f0a51e;--gn:#3ecf6e;--rd:#e5484d;--radius:10px}
html{scroll-behavior:smooth}
body{font-family:'Inter',system-ui,-apple-system,sans-serif;background:var(--bg);color:var(--txt);min-height:100dvh;-webkit-font-smoothing:antialiased}
::-webkit-scrollbar{width:6px;height:6px}
::-webkit-scrollbar-track{background:transparent}
::-webkit-scrollbar-thumb{background:#333;border-radius:3px}
header{display:flex;flex-wrap:wrap;gap:10px;align-items:center;justify-content:space-between;padding:12px 18px;background:linear-gradient(180deg,#181b21 0%,#0f1115 100%);border-bottom:1px solid var(--line);position:sticky;top:0;z-index:100;backdrop-filter:blur(10px)}
.logo{font-size:22px;font-weight:900;letter-spacing:.5px}
.logo span{color:var(--cy)}
.logo small{display:block;font-size:8.5px;font-weight:700;letter-spacing:3px;color:var(--dim);margin-top:2px;text-transform:uppercase}
.pills{display:flex;gap:6px;flex-wrap:wrap}
.pill{font-size:10.5px;background:rgba(255,255,255,.04);border:1px solid var(--line);border-radius:6px;padding:5px 10px;color:var(--dim);white-space:nowrap;backdrop-filter:blur(4px)}
.pill b{color:var(--gn);font-weight:700}
nav{display:flex;gap:3px;padding:8px 18px 0;flex-wrap:wrap;border-bottom:1px solid var(--line)}
nav button{background:transparent;color:var(--dim);border:none;border-bottom:2px solid transparent;padding:10px 16px;font-size:11.5px;font-weight:700;letter-spacing:1.2px;cursor:pointer;transition:all .2s;text-transform:uppercase}
nav button:hover{color:var(--txt)}
nav button.act{color:var(--cy);border-bottom-color:var(--cy)}
.wrap{display:flex;gap:14px;padding:14px 18px;align-items:flex-start;max-width:1400px;margin:0 auto}
#pages{flex:1;min-width:0}
.page{display:none}.page.act{display:block}
.card{background:var(--panel);border:1px solid var(--line);border-radius:var(--radius);padding:14px 16px;margin-bottom:14px;transition:border-color .2s}
.card:hover{border-color:#333a44}
.card h2{font-size:10.5px;margin:0 0 12px;color:var(--cy);letter-spacing:2.5px;font-weight:800;text-transform:uppercase;display:flex;align-items:center;gap:8px}
.card h2::before{content:'';width:3px;height:14px;background:var(--cy);border-radius:2px;display:inline-block}
canvas{width:100%;height:200px;display:block;background:var(--panel2);border-radius:8px;border:1px solid var(--line)}
.grid2{display:grid;grid-template-columns:1fr 1fr;gap:6px 28px}
.grid3{display:grid;grid-template-columns:1fr 1fr 1fr;gap:6px 20px}
@media(max-width:900px){.grid2,.grid3{grid-template-columns:1fr}}
.row{display:flex;align-items:center;gap:10px;margin:6px 0;min-height:36px}
.row label{flex:0 0 130px;font-size:12px;color:var(--dim);font-weight:500}
.row input[type=range]{flex:1;accent-color:var(--cy);height:22px;min-width:50px;cursor:pointer}
.row .val{flex:0 0 68px;text-align:right;font-variant-numeric:tabular-nums;font-size:12px;color:var(--am);font-weight:600}
select,input[type=number],input[type=text],input[type=password]{background:var(--panel2);color:var(--txt);border:1px solid var(--line);border-radius:6px;padding:7px 8px;font-size:12px;transition:border-color .2s;outline:none}
select:focus,input:focus{border-color:var(--cy)}
input[type=checkbox]{accent-color:var(--cy);width:16px;height:16px;cursor:pointer}
button.btn{background:linear-gradient(135deg,var(--cy),#1b8ec7);color:#fff;border:0;border-radius:8px;padding:10px 16px;font-weight:700;margin:3px 2px;cursor:pointer;font-size:12px;transition:transform .1s,box-shadow .2s;box-shadow:0 2px 8px rgba(46,163,224,.25)}
button.btn:hover{transform:translateY(-1px);box-shadow:0 4px 12px rgba(46,163,224,.35)}
button.btn:active{transform:scale(.97)}
button.ghost{background:rgba(255,255,255,.05);color:var(--txt);border:1px solid var(--line);border-radius:8px;padding:9px 14px;margin:3px 2px;cursor:pointer;font-size:12px;font-weight:500;transition:all .2s}
button.ghost:hover{background:rgba(255,255,255,.08);border-color:#444}
button.ghost.on{background:var(--am);color:#171106;font-weight:700;border-color:var(--am);box-shadow:0 2px 8px rgba(240,165,30,.3)}
button.ghost.red.on{background:var(--rd);border-color:var(--rd);color:#fff;box-shadow:0 2px 8px rgba(229,72,77,.3)}
#strips,#gstrips{display:flex;gap:8px;overflow-x:auto;padding:6px 2px 12px;scroll-snap-type:x mandatory}
.strip{flex:0 0 100px;background:var(--panel2);border:1px solid var(--line);border-radius:var(--radius);padding:0 8px 10px;text-align:center;overflow:hidden;scroll-snap-align:start;transition:opacity .2s,border-color .2s}
.strip:hover{border-color:#444}
.strip .scr{margin:0 -8px 8px;padding:5px 2px;font-size:10px;font-weight:800;letter-spacing:.5px;color:#0a0c0e}
.strip select{width:100%;padding:4px;font-size:10.5px;margin-bottom:5px}
.strip input[type=range].vert{writing-mode:vertical-lr;direction:rtl;width:28px;height:120px;accent-color:var(--ac,#f0a51e)}
.strip .gv{font-size:11.5px;font-variant-numeric:tabular-nums;margin:5px 0 3px;color:var(--am);font-weight:700}
.strip input[type=number]{width:68px;padding:3px 4px;font-size:11px;margin:2px 0}
.strip .lb{font-size:9px;color:var(--dim);margin:1px 0}
.strip .dynbox{border-top:1px dashed #2a2f38;margin-top:8px;padding-top:6px;display:none}
.strip .dynbox.open{display:block}
.strip.off{opacity:.3}
.gstrip{flex:0 0 40px;background:var(--panel2);border:1px solid var(--line);border-radius:8px;padding:5px 2px;text-align:center;scroll-snap-align:start;transition:border-color .2s}
.gstrip:hover{border-color:#444}
.gstrip .gf{font-size:8.5px;color:var(--dim);font-weight:600}
.gstrip input[type=range].vert{writing-mode:vertical-lr;direction:rtl;width:20px;height:100px;accent-color:var(--cy)}
.gstrip .gv{font-size:9.5px;color:var(--am);font-variant-numeric:tabular-nums;font-weight:600}
#rail{flex:0 0 124px;background:linear-gradient(180deg,#181b21 0%,#0f1115 100%);border:1px solid var(--line);border-radius:var(--radius);padding:14px 10px;text-align:center;position:sticky;top:70px}
#rail h2{font-size:10.5px;color:var(--cy);margin:0 0 10px;letter-spacing:2.5px;font-weight:800}
#rail input[type=range].vert{writing-mode:vertical-lr;direction:rtl;width:38px;height:180px;accent-color:var(--am)}
.meters{display:flex;gap:8px;justify-content:center;margin:10px 0}
.mcol{width:16px}
.mbar{height:120px;background:var(--panel2);border:1px solid var(--line);border-radius:4px;display:flex;flex-direction:column-reverse;overflow:hidden;position:relative}
.mbar>i{display:block;width:100%;background:linear-gradient(0deg,#2fbf5a 55%,#f0a51e 78%,#e5484d 95%);height:0%;transition:height .15s linear}
.mcol span{font-size:9px;color:var(--dim);font-weight:600}
#rail .rbtn{width:100%;margin:4px 0;padding:9px 0;font-size:11px;font-weight:600;border-radius:8px}
.grwrap{background:var(--panel2);border:1px solid var(--line);border-radius:6px;height:18px;overflow:hidden}
.grbar{height:100%;width:0%;background:linear-gradient(90deg,#f0a51e,#e5484d);float:right;transition:width .2s}
table.info{border-collapse:collapse;font-size:12px;width:100%}
table.info td{padding:6px 16px 6px 0;color:#c6d2de;border-bottom:1px solid var(--line)}
table.info td:first-child{color:var(--dim);font-weight:500;white-space:nowrap}
table.info tr:last-child td{border-bottom:none}
small{color:var(--dim);line-height:1.6;font-size:11px}
.slot{display:flex;align-items:center;gap:8px;background:var(--panel2);border:1px solid var(--line);border-radius:8px;padding:8px 10px;margin:5px 0;transition:border-color .2s}
.slot:hover{border-color:#444}
.slot b{flex:0 0 70px;font-size:12px;font-weight:700}
.slot .st{flex:1;font-size:11px;color:var(--dim)}
.slot.full .st{color:var(--gn);font-weight:500}
@media(max-width:760px){
  .wrap{flex-direction:column;padding:10px 12px}
  #rail{flex:none;width:100%;position:static;display:flex;flex-wrap:wrap;gap:10px;align-items:center;justify-content:center;order:-1}
  #rail input[type=range].vert{height:80px}
  .mbar{height:60px}
  header{padding:10px 12px}
  nav{padding:6px 12px 0}
  nav button{padding:8px 12px;font-size:10.5px}
  .strip{flex:0 0 88px}
  .gstrip{flex:0 0 36px}
}
@media(max-width:480px){
  .grid2,.grid3{grid-template-columns:1fr}
  .row label{flex:0 0 100px;font-size:11px}
  .row .val{flex:0 0 55px;font-size:11px}
  .strip{flex:0 0 82px}
  .strip input[type=range].vert{height:90px}
  canvas{height:160px}
}
</style></head><body>
<header>
  <div class="logo">Mbah<span>DAC</span><small>Digital Console &bull; USB Audio DSP</small></div>
  <div class="pills">
    <span class="pill"><b>USB</b> 48 kHz / 16-bit</span>
    <span class="pill" id="netpill">WiFi: ...</span>
    <span class="pill">PCM5102 &bull; 112 dB SNR</span>
    <span class="pill" id="peakpill">Puncak EQ: - dB</span>
  </div>
</header>
<nav>
  <button data-tab="eq" class="act">EQUALIZER</button>
  <button data-tab="dyn">DYNAMICS</button>
  <button data-tab="out">OUTPUT</button>
  <button data-tab="preset">PRESET</button>
  <button data-tab="sys">SYSTEM</button>
</nav>
<div class="wrap">
<div id="pages">

<section class="page act" id="pg-eq">
  <div class="card"><h2>EQ CURVE + RTA</h2>
    <canvas id="cv" width="940" height="200"></canvas>
  </div>
  <div class="card"><h2>INPUT</h2>
    <div class="grid3">
      <div class="row"><label>Preamp</label><input type="range" id="preamp" min="-12" max="6" step="0.5"><span class="val" id="preamp_v"></span></div>
      <div class="row"><label>Trim Kiri</label><input type="range" id="trimL" min="-12" max="12" step="0.5"><span class="val" id="trimL_v"></span></div>
      <div class="row"><label>Trim Kanan</label><input type="range" id="trimR" min="-12" max="12" step="0.5"><span class="val" id="trimR_v"></span></div>
      <div class="row"><label>Balik Fasa</label><span><input type="checkbox" id="polL"> L &nbsp;<input type="checkbox" id="polR"> R</span><span class="val"></span></div>
      <div class="row"><label>HPF Proteksi</label><input type="checkbox" id="hpfOn"><input type="range" id="hpfFreq" min="20" max="200"><span class="val" id="hpfFreq_v"></span></div>
      <div class="row"><label>Slope HPF</label><select id="hpfSlope"><option value="6">6 dB/okt</option><option value="12">12 dB/okt</option><option value="18">18 dB/okt</option><option value="24">24 dB/okt</option></select><span class="val"></span></div>
    </div>
  </div>
  <div class="card"><h2 id="eqtitle">PARAMETRIC EQ &mdash; 10 BAND</h2>
    <div style="margin-bottom:10px;display:flex;gap:6px;flex-wrap:wrap">
      <button class="ghost" id="modePeq">Parametrik</button>
      <button class="ghost" id="modeGeq">Grafik 31 Band</button>
      <button class="ghost" id="flatBtn">Ratakan EQ</button>
    </div>
    <div id="strips"></div>
    <div id="gstrips" style="display:none"></div>
  </div>
</section>

<section class="page" id="pg-dyn">
  <div class="card"><h2>NOISE GATE</h2>
    <div class="row"><label>Gate Aktif</label><input type="checkbox" id="gateOn"><span class="val"></span></div>
    <div class="grid3">
      <div class="row"><label>Threshold</label><input type="range" id="gateThr" min="-80" max="-10"><span class="val" id="gateThr_v"></span></div>
      <div class="row"><label>Attack</label><input type="range" id="gateAtk" min="0.1" max="200" step="0.1"><span class="val" id="gateAtk_v"></span></div>
      <div class="row"><label>Release</label><input type="range" id="gateRel" min="10" max="1000" step="10"><span class="val" id="gateRel_v"></span></div>
    </div>
  </div>
  <div class="card"><h2>COMPRESSOR</h2>
    <div class="row"><label>Compressor Aktif</label><input type="checkbox" id="compOn"><span class="val"></span></div>
    <div class="grid3">
      <div class="row"><label>Threshold</label><input type="range" id="compThr" min="-48" max="0"><span class="val" id="compThr_v"></span></div>
      <div class="row"><label>Ratio</label><input type="range" id="compRatio" min="1" max="20" step="0.5"><span class="val" id="compRatio_v"></span></div>
      <div class="row"><label>Makeup Gain</label><input type="range" id="compMake" min="0" max="24" step="0.5"><span class="val" id="compMake_v"></span></div>
      <div class="row"><label>Attack</label><input type="range" id="compAtk" min="0.1" max="300" step="0.1"><span class="val" id="compAtk_v"></span></div>
      <div class="row"><label>Release</label><input type="range" id="compRel" min="10" max="1000" step="10"><span class="val" id="compRel_v"></span></div>
    </div>
    <div class="row"><label>Gain Reduction</label><div class="grwrap" style="flex:1"><div class="grbar" id="grbar"></div></div><span class="val" id="gr_v">0 dB</span></div>
  </div>
  <div class="card"><h2>LIMITER</h2>
    <div class="grid2">
      <div class="row"><label>Limiter Aktif</label><input type="checkbox" id="limiter"><span class="val"></span></div>
      <div class="row"><label>Ceiling</label><input type="range" id="ceiling" min="-6" max="0" step="0.5"><span class="val" id="ceiling_v"></span></div>
    </div>
    <div class="row"><label>Clip Terdeteksi</label><b id="clipct" style="color:var(--rd)">0</b><button class="ghost" id="clipReset" style="padding:6px 10px;font-size:11px">Reset</button></div>
  </div>
</section>

<section class="page" id="pg-out">
  <div class="card"><h2>MASTER OUTPUT</h2>
    <div class="grid2">
      <div class="row"><label>Volume Master</label><input type="range" id="vol2" min="0" max="100"><span class="val" id="vol2_v"></span></div>
      <div class="row"><label>Batas Maks</label><input type="range" id="volMax" min="20" max="100"><span class="val" id="volMax_v"></span></div>
      <div class="row"><label>Auto Headroom</label><input type="checkbox" id="autoHr"><small style="flex:1">preamp otomatis menahan puncak EQ</small></div>
      <div class="row"><label>Loudness</label><input type="checkbox" id="loud"><small style="flex:1">kontur Fletcher-Munson</small></div>
    </div>
  </div>
  <div class="card"><h2>STEREO &amp; ALIGNMENT</h2>
    <div class="grid2">
      <div class="row"><label>Balance</label><input type="range" id="balance" min="-10" max="10"><span class="val" id="balance_v"></span></div>
      <div class="row"><label>Stereo Width</label><input type="range" id="width" min="0" max="200"><span class="val" id="width_v"></span></div>
      <div class="row"><label>Cek Mono</label><input type="checkbox" id="mono"><small style="flex:1">gabung L+R jadi mono</small></div>
      <div class="row"><label>Crossfeed</label><input type="checkbox" id="xfOn"><input type="range" id="xfAmt" min="0" max="50"><span class="val" id="xfAmt_v"></span></div>
      <div class="row"><label>Delay Kiri</label><input type="range" id="dlyL" min="0" max="20" step="0.05"><span class="val" id="dlyL_v"></span></div>
      <div class="row"><label>Delay Kanan</label><input type="range" id="dlyR" min="0" max="20" step="0.05"><span class="val" id="dlyR_v"></span></div>
    </div>
  </div>
</section>

<section class="page" id="pg-preset">
  <div class="card"><h2>PRESET PABRIK</h2>
    <div style="display:flex;gap:6px;flex-wrap:wrap">
      <button class="btn" data-preset="flat">Flat</button>
      <button class="btn" data-preset="speaker">Speaker DIY</button>
      <button class="btn" data-preset="vokal">Vokal</button>
      <button class="btn" data-preset="bass">Bass</button>
      <button class="btn" data-preset="malam">Malam</button>
    </div>
  </div>
  <div class="card"><h2>SLOT PENGGUNA</h2><div id="slots"></div>
  <small>10 slot tersimpan permanen di NVS.</small></div>
  <div class="card"><h2>BANDING A / B</h2>
    <button class="ghost" id="abBtn">Sedang: A &mdash; ketuk untuk ke B</button>
  </div>
  <div class="card"><h2>EKSPOR / IMPOR</h2>
    <div style="display:flex;gap:6px;flex-wrap:wrap">
      <button class="ghost" id="expBtn">Ekspor Setting</button>
      <label class="ghost" style="display:inline-flex;cursor:pointer;align-items:center">Impor Setting<input type="file" id="impFile" accept=".json" style="display:none"></label>
    </div>
  </div>
</section>

<section class="page" id="pg-sys">
  <div class="card"><h2>PERANGKAT</h2>
    <table class="info">
      <tr><td>DAC</td><td>PCM5102 &mdash; SNR 112 dB, THD+N -93 dB</td></tr>
      <tr><td>Prosesor</td><td>ESP32-S3 &mdash; USB Audio Class 1.0, 48 kHz/16-bit</td></tr>
      <tr><td>Pin I2S</td><td>BCK GPIO42 &middot; LRCK GPIO21 &middot; DIN GPIO2</td></tr>
      <tr><td>DSP Chain</td><td>Trim &rarr; HPF &rarr; EQ &rarr; Gate &rarr; Comp &rarr; XFeed &rarr; Width &rarr; Delay &rarr; Loud &rarr; Vol &rarr; Limiter</td></tr>
      <tr><td>Jaringan</td><td id="netinfo">&mdash;</td></tr>
      <tr><td>Firmware</td><td id="fwinfo">MbahDAC</td></tr>
      <tr><td>Ring Buffer DSP</td><td id="ringinfo">&mdash;</td></tr>
    </table>
  </div>
  <div class="card"><h2>WIFI</h2>
    <div class="row"><label>Nama WiFi</label><input type="text" id="ssid" style="flex:1"></div>
    <div class="row"><label>Password</label><input type="password" id="wpass" style="flex:1"></div>
    <button class="ghost" id="wifiBtn">Simpan &amp; Restart</button>
    <div style="margin-top:8px"><small>WiFi always on. Upload firmware lewat port COM (UART).</small></div>
  </div>
</section>

</div>
<aside id="rail">
  <h2>MASTER</h2>
  <input type="range" class="vert" id="vol" min="0" max="100">
  <div id="vol_v" style="font-size:14px;color:var(--am);font-weight:700;margin:4px 0">&ndash;</div>
  <div class="meters">
    <div class="mcol"><div class="mbar"><i id="mL"></i></div><span>L</span></div>
    <div class="mcol"><div class="mbar"><i id="mR"></i></div><span>R</span></div>
  </div>
  <button class="ghost rbtn red" id="muteBtn">MUTE</button>
  <button class="ghost rbtn" id="dimBtn">DIM -20</button>
  <button class="ghost rbtn" id="bypassBtn">BYPASS</button>
  <button class="ghost rbtn" id="limBtn">LIMITER</button>
</aside>
</div>

<script>
let S=null, AB='A', slotB=null, RT=new Array(31).fill(-60);
const $=id=>document.getElementById(id);
const GEQF=[20,25,31.5,40,50,63,80,100,125,160,200,250,315,400,500,630,800,1000,1250,1600,2000,2500,3150,4000,5000,6300,8000,10000,12500,16000,20000];
const COL=['#e5484d','#e5484d','#f07f13','#f07f13','#f0a51e','#f0a51e','#3ecf6e','#3ecf6e','#2ea3e0','#b07fe8'];
const TN=['Peak','Low Shelf','High Shelf','Notch','Band-Pass'];
function send(o){return fetch('/api/set',{method:'POST',headers:{'Content-Type':'application/x-www-form-urlencoded'},body:new URLSearchParams(o)});}
// ---- koefisien biquad sisi browser (meniru firmware) ----
function db2lin(db){return Math.pow(10,db*0.05);}
function cBQ(t,f0,g,q){const w=2*Math.PI*f0/48000,cw=Math.cos(w),sw=Math.sin(w);let A,al,a0,b0,b1,b2,a1,a2;
 if(t==1||t==2){A=db2lin(g*0.5);al=sw*0.5*Math.sqrt((A+1/A)*(1/0.9-1)+2);const sA=Math.sqrt(A),tt=2*sA*al;
  if(t==1){b0=A*((A+1)-(A-1)*cw+tt);b1=2*A*((A-1)-(A+1)*cw);b2=A*((A+1)-(A-1)*cw-tt);a0=(A+1)+(A-1)*cw+tt;a1=-2*((A-1)+(A+1)*cw);a2=(A+1)+(A-1)*cw-tt;}
  else{b0=A*((A+1)+(A-1)*cw+tt);b1=-2*A*((A-1)+(A+1)*cw);b2=A*((A+1)+(A-1)*cw-tt);a0=(A+1)-(A-1)*cw+tt;a1=2*((A-1)-(A+1)*cw);a2=(A+1)-(A-1)*cw-tt;}}
 else if(t==3){al=sw/(2*q);b0=1;b1=-2*cw;b2=1;a0=1+al;a1=-2*cw;a2=1-al;}
 else if(t==4){al=sw/(2*q);b0=al;b1=0;b2=-al;a0=1+al;a1=-2*cw;a2=1-al;}
 else{if(Math.abs(g)<0.01)return null;A=db2lin(g*0.5);al=sw/(2*q);b0=1+al*A;b1=-2*cw;b2=1-al*A;a0=1+al/A;a1=-2*cw;a2=1-al/A;}
 return[b0/a0,b1/a0,b2/a0,a1/a0,a2/a0];}
function cHP2(f0){const w=2*Math.PI*f0/48000,cw=Math.cos(w),sw=Math.sin(w),al=sw/(2*0.7071),a0=1+al;return[(1+cw)*0.5/a0,-(1+cw)/a0,(1+cw)*0.5/a0,-2*cw/a0,(1-al)/a0];}
function cHP1(f0){const rc=1/(2*Math.PI*f0),a=rc/(rc+1/48000);return[a,-a,0,-a,0];}
function mag(c,f){if(!c)return 0;const w=2*Math.PI*f/48000,c1=Math.cos(w),c2=Math.cos(2*w);
 const n=c[0]*c[0]+c[1]*c[1]+c[2]*c[2]+2*(c[0]*c[1]+c[1]*c[2])*c1+2*c[0]*c[2]*c2;
 const d=1+c[3]*c[3]+c[4]*c[4]+2*(c[3]+c[3]*c[4])*c1+2*c[4]*c2;return 10*Math.log10(Math.max(n/d,1e-9));}
function drawCurve(){
 const cv=$('cv'),ctx=cv.getContext('2d'),W=cv.width,H=cv.height;ctx.clearRect(0,0,W,H);
 const fmin=20,fmax=20000,X=f=>(Math.log10(f/fmin)/Math.log10(fmax/fmin))*W;
 // RTA bars
 ctx.fillStyle='rgba(46,163,224,0.20)';
 GEQF.forEach((f,i)=>{const h=Math.max(0,(RT[i]+60)/60)*(H-14);ctx.fillRect(X(f)-5,H-h,10,h);});
 ctx.strokeStyle='#232830';ctx.fillStyle='#5f7484';ctx.font='10px system-ui';
 [20,50,100,200,500,1000,2000,5000,10000,20000].forEach(f=>{ctx.beginPath();ctx.moveTo(X(f),0);ctx.lineTo(X(f),H);ctx.stroke();ctx.fillText(f>=1000?(f/1000)+'k':f,X(f)+3,H-4);});
 const y0=H/2,sc=(H/2-8)/16;
 [-12,-6,0,6,12].forEach(db=>{const y=y0-db*sc;ctx.beginPath();ctx.moveTo(0,y);ctx.lineTo(W,y);ctx.strokeStyle=db==0?'#39424e':'#171c21';ctx.stroke();ctx.fillStyle='#5f7484';ctx.fillText((db>0?'+':'')+db,4,y-3);});
 let fil=[];
 if(!S.bypass){
  if(S.hpfOn){if(S.hpfSlope<=6)fil.push(cHP1(S.hpfFreq));else if(S.hpfSlope<=12)fil.push(cHP2(S.hpfFreq));else if(S.hpfSlope<=18){fil.push(cHP2(S.hpfFreq));fil.push(cHP1(S.hpfFreq));}else{fil.push(cHP2(S.hpfFreq));fil.push(cHP2(S.hpfFreq));}}
  if(S.eqMode==0)S.bands.forEach(b=>{if(b.on&&(b.t==0||b.t==1||b.t==2)&&Math.abs(b.g)<0.01)return;if(b.on)fil.push(cBQ(b.t,b.f,b.g,b.q));});
  else S.geq.forEach((g,i)=>{if(Math.abs(g)>0.01)fil.push(cBQ(0,GEQF[i],g,4.318));});
  if(S.loud){const d=Math.min(0.85,Math.max(0,0.85-S.vol));fil.push(cBQ(1,120,d*9,0.9));fil.push(cBQ(2,9000,d*6,0.9));}
 }
 let peak=-60;ctx.beginPath();
 for(let px=0;px<=W;px+=3){const f=fmin*Math.pow(fmax/fmin,px/W);let db=0;fil.forEach(c=>db+=mag(c,f));if(db>peak)peak=db;const y=y0-db*sc;px==0?ctx.moveTo(px,y):ctx.lineTo(px,y);}
 ctx.strokeStyle=S.bypass?'#5f7484':'#f0a51e';ctx.lineWidth=2;ctx.stroke();
 $('peakpill').textContent='Puncak EQ: '+(S.bypass?'bypass':(peak<=-59?'+0.0':(peak>0?'+':'')+peak.toFixed(1))+' dB');
}
// ---- strip ----
function buildStrips(){
 const w=$('strips');w.innerHTML='';
 S.bands.forEach((b,i)=>{
  const d=document.createElement('div');d.className='strip'+(b.on?'':' off');d.style.setProperty('--ac',COL[i]);
  d.innerHTML=`<div class="scr" style="background:${COL[i]}">B${i+1} · ${b.f>=1000?(b.f/1000)+'k':b.f} Hz</div>
  <div><input type="checkbox" id="bon${i}" ${b.on?'checked':''} title="Band aktif"></div>
  <select id="bt${i}">${TN.map((t,ti)=>`<option value="${ti}" ${b.t==ti?'selected':''}>${t}</option>`).join('')}</select>
  <input type="range" class="vert" id="bg${i}" min="-15" max="15" step="0.5" value="${b.g}">
  <div class="gv" id="bg${i}_v">${b.g} dB</div>
  <input type="number" id="bf${i}" min="20" max="20000" value="${b.f}"><div class="lb">Hz</div>
  <input type="number" id="bq${i}" min="0.4" max="16" step="0.1" value="${b.q}"><div class="lb">Q</div>
  <button class="ghost" id="bdyn${i}" style="padding:3px 8px;font-size:10.5px;${b.dyn?'border-color:var(--am);color:var(--am)':''}">DYN</button>
  <div class="dynbox" id="dynbox${i}">
    <div class="lb">Dynamic EQ</div>
    <div><input type="checkbox" id="bdon${i}" ${b.dyn?'checked':''}> <span class="lb">aktif</span></div>
    <input type="number" id="bdthr${i}" min="-60" max="0" value="${b.dthr}"><div class="lb">Threshold dB</div>
    <input type="number" id="bdrng${i}" min="-12" max="12" step="0.5" value="${b.drng}"><div class="lb">Rentang dB</div>
  </div>`;
  w.appendChild(d);
  $('bg'+i).oninput=e=>{b.g=+e.target.value;$('bg'+i+'_v').textContent=b.g+' dB';send({['b'+i+'_g']:b.g});drawCurve();};
  $('bf'+i).onchange=e=>{b.f=+e.target.value;send({['b'+i+'_f']:b.f});buildStrips();drawCurve();};
  $('bq'+i).onchange=e=>{b.q=+e.target.value;send({['b'+i+'_q']:b.q});drawCurve();};
  $('bt'+i).onchange=e=>{b.t=+e.target.value;send({['b'+i+'_t']:b.t});drawCurve();};
  $('bon'+i).onchange=e=>{b.on=e.target.checked;send({['b'+i+'_on']:b.on?1:0});d.classList.toggle('off',!b.on);drawCurve();};
  $('bdyn'+i).onclick=()=>$('dynbox'+i).classList.toggle('open');
  $('bdon'+i).onchange=e=>{b.dyn=e.target.checked;send({['b'+i+'_dyn']:b.dyn?1:0});};
  $('bdthr'+i).onchange=e=>{b.dthr=+e.target.value;send({['b'+i+'_dthr']:b.dthr});};
  $('bdrng'+i).onchange=e=>{b.drng=+e.target.value;send({['b'+i+'_drng']:b.drng});};
 });
 const gw=$('gstrips');gw.innerHTML='';
 S.geq.forEach((g,i)=>{
  const d=document.createElement('div');d.className='gstrip';
  d.innerHTML=`<div class="gf">${GEQF[i]>=1000?(GEQF[i]/1000)+'k':GEQF[i]}</div>
  <input type="range" class="vert" id="gg${i}" min="-12" max="12" step="0.5" value="${g}">
  <div class="gv" id="gg${i}_v">${g}</div>`;
  gw.appendChild(d);
  $('gg'+i).oninput=e=>{S.geq[i]=+e.target.value;$('gg'+i+'_v').textContent=e.target.value;send({['g'+i]:e.target.value});drawCurve();};
 });
}
function buildSlots(){
 const w=$('slots');w.innerHTML='';
 S.slots.forEach((full,i)=>{
  const d=document.createElement('div');d.className='slot'+(full?' full':'');
  d.innerHTML=`<b>Slot ${i+1}</b><span class="st">${full?'Terisi — setting tersimpan':'Kosong'}</span>
  <button class="ghost" data-load="${i}" ${full?'':'disabled'}>Muat</button>
  <button class="ghost" data-save="${i}">Simpan</button>`;
  w.appendChild(d);
 });
 w.querySelectorAll('[data-load]').forEach(b=>b.onclick=async()=>{await fetch('/api/preset',{method:'POST',headers:{'Content-Type':'application/x-www-form-urlencoded'},body:new URLSearchParams({name:'user:'+b.dataset.load})});load();});
 w.querySelectorAll('[data-save]').forEach(b=>b.onclick=async()=>{if(confirm('Simpan setting sekarang ke Slot '+(+b.dataset.save+1)+'?')){await fetch('/api/save',{method:'POST',headers:{'Content-Type':'application/x-www-form-urlencoded'},body:new URLSearchParams({slot:b.dataset.save})});load();}});
}
function fill(){
 $('vol').value=S.vol*100;$('vol_v').textContent=Math.round(S.vol*100)+'%';
 $('vol2').value=S.vol*100;$('vol2_v').textContent=Math.round(S.vol*100)+'%';
 $('volMax').value=S.volMax*100;$('volMax_v').textContent=Math.round(S.volMax*100)+'%';
 $('preamp').value=S.preamp;$('preamp_v').textContent=S.preamp+' dB';
 $('trimL').value=S.trimL;$('trimL_v').textContent=S.trimL+' dB';
 $('trimR').value=S.trimR;$('trimR_v').textContent=S.trimR+' dB';
 $('polL').checked=S.polL;$('polR').checked=S.polR;
 $('muteBtn').classList.toggle('on',S.mute);$('muteBtn').textContent=S.mute?'MUTED':'MUTE';
 $('dimBtn').classList.toggle('on',S.dim);
 $('bypassBtn').classList.toggle('on',S.bypass);
 $('limiter').checked=S.limiter;$('limBtn').classList.toggle('on',S.limiter);
 $('ceiling').value=S.ceiling;$('ceiling_v').textContent=S.ceiling+' dB';
 $('hpfOn').checked=S.hpfOn;$('hpfFreq').value=S.hpfFreq;$('hpfFreq_v').textContent=S.hpfFreq+' Hz';$('hpfSlope').value=S.hpfSlope;
 $('gateOn').checked=S.gateOn;$('gateThr').value=S.gateThr;$('gateThr_v').textContent=S.gateThr+' dB';
 $('gateAtk').value=S.gateAtk;$('gateAtk_v').textContent=S.gateAtk+' ms';$('gateRel').value=S.gateRel;$('gateRel_v').textContent=S.gateRel+' ms';
 $('compOn').checked=S.compOn;$('compThr').value=S.compThr;$('compThr_v').textContent=S.compThr+' dB';
 $('compRatio').value=S.compRatio;$('compRatio_v').textContent=S.compRatio+' : 1';
 $('compMake').value=S.compMake;$('compMake_v').textContent=S.compMake+' dB';
 $('compAtk').value=S.compAtk;$('compAtk_v').textContent=S.compAtk+' ms';$('compRel').value=S.compRel;$('compRel_v').textContent=S.compRel+' ms';
 $('xfOn').checked=S.xfOn;$('xfAmt').value=S.xfAmt*100;$('xfAmt_v').textContent=Math.round(S.xfAmt*100)+'%';
 $('width').value=S.width*100;$('width_v').textContent=Math.round(S.width*100)+'%';$('mono').checked=S.mono;
 $('balance').value=S.balance*10;$('balance_v').textContent=S.balance==0?'Tengah':S.balance;
 $('dlyL').value=S.dlyL;$('dlyL_v').textContent=S.dlyL+' ms · '+(S.dlyL*34.3).toFixed(0)+' cm';
 $('dlyR').value=S.dlyR;$('dlyR_v').textContent=S.dlyR+' ms · '+(S.dlyR*34.3).toFixed(0)+' cm';
 $('loud').checked=S.loud;$('autoHr').checked=S.autoHr;
 $('modePeq').classList.toggle('on',S.eqMode==0);$('modeGeq').classList.toggle('on',S.eqMode==1);
 $('strips').style.display=S.eqMode==0?'flex':'none';$('gstrips').style.display=S.eqMode==1?'flex':'none';
 $('eqtitle').textContent=S.eqMode==0?'PARAMETRIC EQ — 10 BAND':'GRAPHIC EQ — 31 BAND 1/3 OKTAF';
 $('netpill').innerHTML=S.net;$('netinfo').textContent=S.net+' · http://mbahdac.local';$('fwinfo').textContent='MbahDAC '+S.fw;$('ringinfo').textContent='underrun '+S.ur+' - overflow '+S.ov;
 buildStrips();buildSlots();drawCurve();
}
async function load(){S=await (await fetch('/api/state')).json();fill();}
function bulk(s){const o={preamp:s.preamp,trimL:s.trimL,trimR:s.trimR,polL:s.polL?1:0,polR:s.polR?1:0,hpfOn:s.hpfOn?1:0,hpfFreq:s.hpfFreq,hpfSlope:s.hpfSlope,eqMode:s.eqMode,bypass:s.bypass?1:0,loud:s.loud?1:0,gateOn:s.gateOn?1:0,gateThr:s.gateThr,gateAtk:s.gateAtk,gateRel:s.gateRel,compOn:s.compOn?1:0,compThr:s.compThr,compRatio:s.compRatio,compAtk:s.compAtk,compRel:s.compRel,compMake:s.compMake,limiter:s.limiter?1:0,ceiling:s.ceiling,xfOn:s.xfOn?1:0,xfAmt:s.xfAmt,width:s.width,mono:s.mono?1:0,balance:s.balance,dlyL:s.dlyL,dlyR:s.dlyR,vol:s.vol,mute:s.mute?1:0,dim:s.dim?1:0,volMax:s.volMax,autoHr:s.autoHr?1:0};
 s.bands.forEach((b,i)=>{o['b'+i+'_on']=b.on?1:0;o['b'+i+'_t']=b.t;o['b'+i+'_f']=b.f;o['b'+i+'_g']=b.g;o['b'+i+'_q']=b.q;o['b'+i+'_dyn']=b.dyn?1:0;o['b'+i+'_dthr']=b.dthr;o['b'+i+'_drng']=b.drng;});
 s.geq.forEach((g,i)=>{o['g'+i]=g;});return send(o);}
// ikatan kontrol sederhana
function bindRange(id,key,fmt,scale){$(id).oninput=e=>{const v=+e.target.value*(scale||1);S[key]=v;$(id+'_v').textContent=fmt(v);send({[key]:v});};}
function bindChk(id,key,after){$(id).onchange=e=>{S[key]=e.target.checked;send({[key]:S[key]?1:0});if(after)after();};}
bindRange('preamp','preamp',v=>v+' dB');bindRange('trimL','trimL',v=>v+' dB');bindRange('trimR','trimR',v=>v+' dB');
bindRange('hpfFreq','hpfFreq',v=>v+' Hz');bindRange('ceiling','ceiling',v=>v+' dB');
bindRange('gateThr','gateThr',v=>v+' dB');bindRange('gateAtk','gateAtk',v=>v+' ms');bindRange('gateRel','gateRel',v=>v+' ms');
bindRange('compThr','compThr',v=>v+' dB');bindRange('compRatio','compRatio',v=>v+' : 1');bindRange('compMake','compMake',v=>v+' dB');
bindRange('compAtk','compAtk',v=>v+' ms');bindRange('compRel','compRel',v=>v+' ms');
bindChk('polL','polL');bindChk('polR','polR');bindChk('hpfOn','hpfOn',drawCurve);bindChk('gateOn','gateOn');bindChk('compOn','compOn');
bindChk('xfOn','xfOn');bindChk('mono','mono');bindChk('loud','loud',drawCurve);bindChk('autoHr','autoHr');
$('hpfSlope').onchange=e=>{S.hpfSlope=+e.target.value;send({hpfSlope:S.hpfSlope});drawCurve();};
$('xfAmt').oninput=e=>{S.xfAmt=e.target.value/100;$('xfAmt_v').textContent=e.target.value+'%';send({xfAmt:S.xfAmt});};
$('width').oninput=e=>{S.width=e.target.value/100;$('width_v').textContent=e.target.value+'%';send({width:S.width});};
$('balance').oninput=e=>{S.balance=e.target.value/10;$('balance_v').textContent=S.balance==0?'Tengah':S.balance;send({balance:S.balance});};
$('dlyL').oninput=e=>{S.dlyL=+e.target.value;$('dlyL_v').textContent=S.dlyL+' ms · '+(S.dlyL*34.3).toFixed(0)+' cm';send({dlyL:S.dlyL});};
$('dlyR').oninput=e=>{S.dlyR=+e.target.value;$('dlyR_v').textContent=S.dlyR+' ms · '+(S.dlyR*34.3).toFixed(0)+' cm';send({dlyR:S.dlyR});};
function setVol(v){S.vol=v/100;$('vol').value=v;$('vol2').value=v;$('vol_v').textContent=v+'%';$('vol2_v').textContent=v+'%';send({vol:S.vol});drawCurve();}
$('vol').oninput=e=>setVol(+e.target.value);$('vol2').oninput=e=>setVol(+e.target.value);
$('volMax').oninput=e=>{S.volMax=e.target.value/100;$('volMax_v').textContent=e.target.value+'%';send({volMax:S.volMax});};
$('limiter').onchange=e=>{S.limiter=e.target.checked;send({limiter:S.limiter?1:0});$('limBtn').classList.toggle('on',S.limiter);};
$('limBtn').onclick=()=>{S.limiter=!S.limiter;send({limiter:S.limiter?1:0});fill();};
$('muteBtn').onclick=()=>{S.mute=!S.mute;send({mute:S.mute?1:0});fill();};
$('dimBtn').onclick=()=>{S.dim=!S.dim;send({dim:S.dim?1:0});fill();};
$('bypassBtn').onclick=()=>{S.bypass=!S.bypass;send({bypass:S.bypass?1:0});fill();};
$('modePeq').onclick=()=>{S.eqMode=0;send({eqMode:0});fill();};
$('modeGeq').onclick=()=>{S.eqMode=1;send({eqMode:1});fill();};
$('flatBtn').onclick=async()=>{const o={};if(S.eqMode==0)S.bands.forEach((b,i)=>o['b'+i+'_g']=0);else S.geq.forEach((g,i)=>o['g'+i]=0);await send(o);load();};
$('clipReset').onclick=()=>{send({clipReset:1});$('clipct').textContent='0';};
// tab
document.querySelectorAll('nav button').forEach(b=>b.onclick=()=>{
 document.querySelectorAll('nav button').forEach(x=>x.classList.remove('act'));b.classList.add('act');
 document.querySelectorAll('.page').forEach(p=>p.classList.remove('act'));$('pg-'+b.dataset.tab).classList.add('act');
 if(b.dataset.tab=='eq')drawCurve();});
// preset pabrik
document.querySelectorAll('[data-preset]').forEach(b=>b.onclick=async()=>{
 await fetch('/api/preset',{method:'POST',headers:{'Content-Type':'application/x-www-form-urlencoded'},body:new URLSearchParams({name:b.dataset.preset})});load();});
$('abBtn').onclick=async()=>{
 if(AB=='A'){slotB=JSON.parse(JSON.stringify(S));AB='B';}
 else{const cur=JSON.parse(JSON.stringify(S));await bulk(slotB);slotB=cur;AB='A';load();}
 $('abBtn').textContent='Sedang: '+AB+' — ketuk untuk ke '+(AB=='A'?'B':'A');};
$('expBtn').onclick=()=>{const blob=new Blob([JSON.stringify(S,null,1)],{type:'application/json'});const a=document.createElement('a');a.href=URL.createObjectURL(blob);a.download='mbahdac-preset.json';a.click();};
$('impFile').onchange=async e=>{try{const s=JSON.parse(await e.target.files[0].text());await bulk(s);load();}catch(err){alert('File tidak valid');}};
$('wifiBtn').onclick=()=>{if(confirm('Alat restart dan gabung WiFi rumah dengan data baru. Lanjut?'))fetch('/api/wifi',{method:'POST',headers:{'Content-Type':'application/x-www-form-urlencoded'},body:new URLSearchParams({ssid:$('ssid').value,pass:$('wpass').value})});};
setInterval(async()=>{try{const m=await (await fetch('/api/meter')).json();
 $('mL').style.height=Math.min(100,m.l*100)+'%';$('mR').style.height=Math.min(100,m.r*100)+'%';
 $('grbar').style.width=Math.min(100,-m.gr/24*100)+'%';$('gr_v').textContent=m.gr+' dB';$('clipct').textContent=m.clip;}catch(e){}},300);
setInterval(async()=>{try{const r=await (await fetch('/api/rta')).json();RT=r.rta;if($('pg-eq').classList.contains('act'))drawCurve();}catch(e){}},600);
load();
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
//  Kontrol serial (port COM / Serial0) — kanal kedua ala DQ-DSP
//  Satu perintah per baris. Contoh:
//    set vol=0.8 mute=0        set b0_g=3 b0_f=100 b0_on=1
//    set g12=4 (GEQ)           preset Bass | upreset 2 | save 2 | state
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
  snapSmoothToParams();
  rebuildCoefficients();
  rebuildDynCoefs();
  setupAudio();
  xTaskCreatePinnedToCore(dspTask, "dspTask", 8192, NULL, 8, &dspTaskHandle, 1);
  loadWifiCreds();
  setupWeb();
  wifiStart();
  Serial0.println("[USB] MbahDAC " FW_VERSION " siap — tunggu Windows memilih 'MbahDAC USB'.");
}

void loop() {
  wifiPoll();
  serialPoll();
  static uint32_t lastRta = 0;
  if (millis() - lastRta > 140) { lastRta = millis(); computeRTA(); }
  delay(2);
}
