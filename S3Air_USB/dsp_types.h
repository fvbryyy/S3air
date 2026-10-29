// ============================================================
//  MbahDAC FINAL — tipe data DSP (dibagi .ino utama)
//  Rantai: Trim/Polarity -> HPF -> EQ (PEQ 10 / GEQ 31, +Dynamic EQ)
//          -> Gate -> Compressor -> Crossfeed -> Width/Balance
//          -> Delay -> Loudness -> Volume/Dim -> Limiter -> Out
// ============================================================
#pragma once
#include <Arduino.h>

#define NUM_BANDS 10
#define NUM_GEQ   31
#define NUM_USER_PRESETS 10

// Tipe band PEQ
enum BandType : uint8_t {
  BT_PEAKING = 0,
  BT_LOWSHELF = 1,
  BT_HIGHSHELF = 2,
  BT_NOTCH = 3,
  BT_BANDPASS = 4
};

struct Band {
  bool     on;
  uint8_t  type;      // BandType
  float    freq;      // Hz
  float    gainDb;    // -15..+15 (shelf/peaking)
  float    q;         // 0.4..16
  bool     dynOn;     // dynamic EQ (semua band bisa, detektor bandpass sendiri)
  float    dynThrDb;  // threshold detektor -60..0
  float    dynRangeDb;// rentang gerak gain dinamis -12..+12
};

struct DspParams {
  // --- input ---
  float preampDb;          // -12..+6
  float trimLDb, trimRDb;  // -12..+12
  bool  polL, polR;        // invert polarity per channel
  // --- HPF proteksi ---
  bool  hpfOn;
  float hpfFreq;           // 20..200
  uint8_t hpfSlope;        // 6 / 12 / 18 / 24 dB per oktaf
  // --- EQ ---
  uint8_t eqMode;          // 0 = PEQ, 1 = GEQ
  Band  band[NUM_BANDS];
  float geq[NUM_GEQ];      // gain dB per band 1/3 oktaf, -12..+12
  // --- gate ---
  bool  gateOn;
  float gateThrDb;         // -80..-10
  float gateAttackMs, gateReleaseMs;
  // --- compressor ---
  bool  compOn;
  float compThrDb;         // -48..0
  float compRatio;         // 1..20
  float compAttackMs, compReleaseMs, compMakeupDb;
  // --- limiter ---
  bool  limiterOn;
  float ceilingDb;         // -6..0
  // --- output / alignment ---
  bool  crossfeedOn;
  float crossfeedAmt;      // 0..0.5
  float width;             // 0..2 (1 = normal)
  bool  monoCheck;
  float balance;           // -1..+1
  float delayLms, delayRms;// 0..20 ms
  bool  loudnessOn;
  // --- master ---
  float volume;            // 0..1
  bool  mute;
  bool  dim;               // -20 dB sesaat
  bool  bypass;            // lewati HPF+EQ (dinamika & output tetap jalan)
  float volMax;            // kunci volume maksimum 0.2..1
  bool  autoHeadroom;      // preamp otomatis menahan puncak kurva EQ
};

// Koefisien biquad ternormalisasi (a0 = 1)
struct BiquadCoef {
  float b0, b1, b2, a1, a2;
  bool  bypass;
};

// Keadaan filter stereo (direct form I transposed ringkas)
struct BiquadState {
  float x1L, x2L, y1L, y2L;
  float x1R, x2R, y1R, y2R;
};
