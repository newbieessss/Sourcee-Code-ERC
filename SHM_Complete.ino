/*
 * ============================================================
 *   STRUCTURAL HEALTH MONITORING (SHM) — FULL INTEGRATED
 *   Platform  : ESP32
 *   Sensors   : MPU6050 (vibrasi) + 2x HX711 (load cell)
 *   Interface : WiFi HTTP Server → SHM Dashboard HTML
 * ============================================================
 *
 *  LIBRARY YANG PERLU DIINSTALL (Arduino Library Manager):
 *    1. Adafruit MPU6050        (by Adafruit)
 *    2. Adafruit Unified Sensor (by Adafruit)
 *    3. arduinoFFT              (by kosme)
 *    4. HX711                   (by Bogdan Necula / olkal)
 *    5. ArduinoJson             (by Benoit Blanchon) ← versi 6.x
 *
 *  Board: ESP32 Dev Module (Tools → Board → ESP32 Arduino)
 *
 *  ENDPOINT API:
 *    GET  /          → info singkat
 *    GET  /status    → mode + baseline valid
 *    GET  /data      → JSON data monitoring terbaru
 *    GET  /baseline  → JSON statistik baseline
 *    POST /cmd       → body: cmd=b|m|t|r|s
 *
 *  COMMANDS (via POST /cmd atau Serial Monitor):
 *    b → mulai baseline baru
 *    m → mulai monitoring
 *    t → tare load cell
 *    r → reset baseline NVS
 *    s → cek status
 * ============================================================
 */

// ============================================================
//  LIBRARY
// ============================================================
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include "arduinoFFT.h"
#include "HX711.h"
#include <Preferences.h>
#include <math.h>
#include <WiFi.h>
#include <WebServer.h>
#include <ArduinoJson.h>

// ============================================================
//  *** GANTI INI SESUAI WIFI KAMU ***
// ============================================================
const char* WIFI_SSID     = "Abiumi";
const char* WIFI_PASSWORD = "bismillah";

// ============================================================
//  KONFIGURASI SAMPLING
// ============================================================
const uint16_t SAMPLES           = 256;
const double   SAMPLING_FREQ     = 100.0;          // Hz
const double   SAMPLING_PERIOD_MS = 1000.0 / SAMPLING_FREQ; // 10 ms

// ============================================================
//  KONFIGURASI BASELINE
// ============================================================
const int N_BASELINE  = 10;
const int TREND_WINDOW = 10;

// ============================================================
//  PIN KONFIGURASI
// ============================================================
#define LOADCELL1_DOUT  26
#define LOADCELL1_SCK   25
#define LOADCELL2_DOUT  5
#define LOADCELL2_SCK   18

// ============================================================
//  MODE OPERASI
// ============================================================
#define MODE_IDLE     0
#define MODE_BASELINE 1
#define MODE_MONITOR  2

// ============================================================
//  OBJEK GLOBAL
// ============================================================
TwoWire          I2CMPU = TwoWire(0);
Adafruit_MPU6050 mpu;
HX711            scale1, scale2;
Preferences      prefs;
WebServer        server(80);

double vReal[SAMPLES];
double vImag[SAMPLES];
ArduinoFFT<double> FFT = ArduinoFFT<double>(vReal, vImag, SAMPLES, SAMPLING_FREQ);

// ============================================================
//  STRUCT FITUR
// ============================================================
struct SHMFeatures {
  double rms;
  double peak;
  double crestFactor;
  double kurtosis;
  double freqPeak1;
  double freqPeak2;
  double freqPeak3;
  double freqPeakMag1;
  double freqBandEnergy[4];
  float  totalLoad;
  float  ratio;
};

// ============================================================
//  STRUCT BASELINE
// ============================================================
struct BaselineStat {
  double mean;
  double stddev;
};

struct SHMBaseline {
  BaselineStat rms;
  BaselineStat crestFactor;
  BaselineStat kurtosis;
  BaselineStat freqPeak1;
  BaselineStat totalLoad;
  BaselineStat ratio;
  bool         valid;
};

SHMBaseline baseline;

// Buffer baseline
double buf_rms[N_BASELINE];
double buf_crest[N_BASELINE];
double buf_kurtosis[N_BASELINE];
double buf_freq[N_BASELINE];
double buf_load[N_BASELINE];
double buf_ratio[N_BASELINE];
int    baseline_count = 0;

// ============================================================
//  TREND TRACKING
// ============================================================
double trend_rms[TREND_WINDOW];
double trend_crest[TREND_WINDOW];
int    trend_idx   = 0;
int    trend_count = 0;

// ============================================================
//  STATE
// ============================================================
int   mode        = MODE_IDLE;
float calibFactor1 = 1062.27;
float calibFactor2 = 1052.77;

// ============================================================
//  DATA TERBARU UNTUK API (diupdate tiap monitoring cycle)
// ============================================================
struct LatestPayload {
  // Fitur
  double rms         = 0;
  double peak        = 0;
  double crestFactor = 0;
  double kurtosis    = 0;
  double freqPeak1   = 0;
  double freqPeak2   = 0;
  double freqPeak3   = 0;
  double bandEnergy[4] = {0,0,0,0};
  float  totalLoad   = 0;
  float  ratio       = 0.5;
  // Anomali
  double z_rms       = 0;
  double z_cf        = 0;
  double z_kurt      = 0;
  double z_freq      = 0;
  double z_load      = 0;
  double z_ratio     = 0;
  double score       = 0;
  bool   trendRising = false;
  int    anomalyLevel = 0;  // 0=NORMAL 1=WARNING 2=ALERT
  // Meta
  String statusStr   = "IDLE";
  unsigned long ts   = 0;
} latest;

// ============================================================
//  HELPER: STATISTIK
// ============================================================
double arrayMean(double* arr, int n) {
  double s = 0;
  for (int i = 0; i < n; i++) s += arr[i];
  return s / n;
}

double arrayStddev(double* arr, int n, double mean) {
  double s = 0;
  for (int i = 0; i < n; i++) {
    double d = arr[i] - mean;
    s += d * d;
  }
  return sqrt(s / n);
}

// ============================================================
//  HELPER: CARI N-TH LARGEST INDEX FFT
// ============================================================
int nthLargestIndex(double* arr, int n, int excludeA = -1, int excludeB = -1) {
  double maxVal = -1;
  int    maxIdx = 2;
  for (int i = 2; i < n / 2; i++) {
    bool nearA = (excludeA >= 0 && abs(i - excludeA) <= 2);
    bool nearB = (excludeB >= 0 && abs(i - excludeB) <= 2);
    if (nearA || nearB) continue;
    if (arr[i] > maxVal) { maxVal = arr[i]; maxIdx = i; }
  }
  return maxIdx;
}

// ============================================================
//  AKUISISI & EKSTRAKSI FITUR
// ============================================================
bool acquireFeatures(SHMFeatures &f) {

  // 1. SAMPLING
  for (int i = 0; i < SAMPLES; i++) {
    unsigned long t = millis();
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    vReal[i] = sqrt(a.acceleration.x * a.acceleration.x +
                    a.acceleration.y * a.acceleration.y +
                    a.acceleration.z * a.acceleration.z);
    vImag[i] = 0;

    // Handle WiFi client selama sampling agar tidak timeout
    server.handleClient();

    while (millis() - t < (unsigned long)SAMPLING_PERIOD_MS) { /* spin */ }
  }

  // 2. SUBTRACT DC OFFSET (gravitasi)
  double dc = arrayMean(vReal, SAMPLES);
  for (int i = 0; i < SAMPLES; i++) vReal[i] -= dc;

  // 3. FITUR TIME DOMAIN
  double sumSq   = 0;
  double maxAmp  = 0;
  double minV = vReal[0], maxV = vReal[0];

  for (int i = 0; i < SAMPLES; i++) {
    double v = vReal[i];
    sumSq += v * v;
    if (fabs(v) > maxAmp) maxAmp = fabs(v);
    if (v > maxV) maxV = v;
    if (v < minV) minV = v;
  }

  f.rms        = sqrt(sumSq / SAMPLES);
  f.peak       = maxV - minV;
  f.crestFactor = (f.rms > 1e-9) ? (maxAmp / f.rms) : 0;

  // Kurtosis = (mean of x^4) / rms^4
  double sumFourth = 0;
  for (int i = 0; i < SAMPLES; i++) {
    double v2 = vReal[i] * vReal[i];
    sumFourth += v2 * v2;
  }
  double rms4 = f.rms * f.rms * f.rms * f.rms;
  f.kurtosis = (rms4 > 1e-18) ? ((sumFourth / SAMPLES) / rms4) : 0;

  // 4. FFT
  FFT.windowing(FFTWindow::Hamming, FFTDirection::Forward);
  FFT.compute(FFTDirection::Forward);
  FFT.complexToMagnitude();

  double binToHz = SAMPLING_FREQ / SAMPLES;
  int p1 = nthLargestIndex(vReal, SAMPLES);
  int p2 = nthLargestIndex(vReal, SAMPLES, p1);
  int p3 = nthLargestIndex(vReal, SAMPLES, p1, p2);

  f.freqPeak1    = p1 * binToHz;
  f.freqPeak2    = p2 * binToHz;
  f.freqPeak3    = p3 * binToHz;
  f.freqPeakMag1 = vReal[p1];

  // Energi per band: 0-10, 10-25, 25-40, 40-50 Hz
  double bandLimits[5] = {0, 10, 25, 40, 50};
  for (int b = 0; b < 4; b++) {
    f.freqBandEnergy[b] = 0;
    for (int i = 0; i < SAMPLES / 2; i++) {
      double freq = i * binToHz;
      if (freq >= bandLimits[b] && freq < bandLimits[b + 1])
        f.freqBandEnergy[b] += vReal[i] * vReal[i];
    }
  }

  // 5. LOAD CELL
  if (!scale1.is_ready() || !scale2.is_ready()) {
    f.totalLoad = 0;
    f.ratio     = 0.5;
  } else {
    float R1 = scale1.get_units(10);
    float R2 = scale2.get_units(10);
    if (R1 < 0) R1 = 0;
    if (R2 < 0) R2 = 0;
    f.totalLoad = R1 + R2;
    f.ratio     = (f.totalLoad > 0.5f) ? (R1 / f.totalLoad) : 0.5f;
  }

  return true;
}

// ============================================================
//  KOMPUTASI STATISTIK BASELINE
// ============================================================
void computeBaselineStats() {
  baseline.rms.mean           = arrayMean(buf_rms,      N_BASELINE);
  baseline.rms.stddev         = arrayStddev(buf_rms,    N_BASELINE, baseline.rms.mean);
  baseline.crestFactor.mean   = arrayMean(buf_crest,    N_BASELINE);
  baseline.crestFactor.stddev = arrayStddev(buf_crest,  N_BASELINE, baseline.crestFactor.mean);
  baseline.kurtosis.mean      = arrayMean(buf_kurtosis, N_BASELINE);
  baseline.kurtosis.stddev    = arrayStddev(buf_kurtosis,N_BASELINE,baseline.kurtosis.mean);
  baseline.freqPeak1.mean     = arrayMean(buf_freq,     N_BASELINE);
  baseline.freqPeak1.stddev   = arrayStddev(buf_freq,   N_BASELINE, baseline.freqPeak1.mean);
  baseline.totalLoad.mean     = arrayMean(buf_load,     N_BASELINE);
  baseline.totalLoad.stddev   = arrayStddev(buf_load,   N_BASELINE, baseline.totalLoad.mean);
  baseline.ratio.mean         = arrayMean(buf_ratio,    N_BASELINE);
  baseline.ratio.stddev       = arrayStddev(buf_ratio,  N_BASELINE, baseline.ratio.mean);
  baseline.valid = true;
}

// ============================================================
//  NVS: SIMPAN & LOAD BASELINE
// ============================================================
void saveBaselineToNVS() {
  prefs.begin("shm_base", false);
  prefs.putDouble("rms_m",  baseline.rms.mean);
  prefs.putDouble("rms_s",  baseline.rms.stddev);
  prefs.putDouble("cf_m",   baseline.crestFactor.mean);
  prefs.putDouble("cf_s",   baseline.crestFactor.stddev);
  prefs.putDouble("kur_m",  baseline.kurtosis.mean);
  prefs.putDouble("kur_s",  baseline.kurtosis.stddev);
  prefs.putDouble("freq_m", baseline.freqPeak1.mean);
  prefs.putDouble("freq_s", baseline.freqPeak1.stddev);
  prefs.putDouble("load_m", baseline.totalLoad.mean);
  prefs.putDouble("load_s", baseline.totalLoad.stddev);
  prefs.putDouble("rat_m",  baseline.ratio.mean);
  prefs.putDouble("rat_s",  baseline.ratio.stddev);
  prefs.putBool("valid",    true);
  prefs.end();
  Serial.println("[NVS] Baseline saved.");
}

void loadBaselineFromNVS() {
  prefs.begin("shm_base", true);
  bool valid = prefs.getBool("valid", false);
  if (valid) {
    baseline.rms.mean           = prefs.getDouble("rms_m",  0);
    baseline.rms.stddev         = prefs.getDouble("rms_s",  1);
    baseline.crestFactor.mean   = prefs.getDouble("cf_m",   0);
    baseline.crestFactor.stddev = prefs.getDouble("cf_s",   1);
    baseline.kurtosis.mean      = prefs.getDouble("kur_m",  3);
    baseline.kurtosis.stddev    = prefs.getDouble("kur_s",  1);
    baseline.freqPeak1.mean     = prefs.getDouble("freq_m", 0);
    baseline.freqPeak1.stddev   = prefs.getDouble("freq_s", 1);
    baseline.totalLoad.mean     = prefs.getDouble("load_m", 0);
    baseline.totalLoad.stddev   = prefs.getDouble("load_s", 1);
    baseline.ratio.mean         = prefs.getDouble("rat_m",  0.5);
    baseline.ratio.stddev       = prefs.getDouble("rat_s",  0.1);
    baseline.valid = true;
    Serial.println("[NVS] Baseline loaded from storage.");
  } else {
    baseline.valid = false;
    Serial.println("[NVS] No saved baseline found.");
  }
  prefs.end();
}

// ============================================================
//  DETEKSI ANOMALI — 3 SIGMA + WEIGHTED SCORE + TREND
// ============================================================
int detectAnomaly(const SHMFeatures &f) {
  if (!baseline.valid) return 0;

  auto zScore = [](double val, double mean, double std) -> double {
    if (std < 1e-12) return 0;
    return fabs((val - mean) / std);
  };

  double z_rms   = zScore(f.rms,        baseline.rms.mean,         baseline.rms.stddev);
  double z_cf    = zScore(f.crestFactor, baseline.crestFactor.mean, baseline.crestFactor.stddev);
  double z_kurt  = zScore(f.kurtosis,   baseline.kurtosis.mean,    baseline.kurtosis.stddev);
  double z_freq  = zScore(f.freqPeak1,  baseline.freqPeak1.mean,   baseline.freqPeak1.stddev);
  double z_load  = zScore(f.totalLoad,  baseline.totalLoad.mean,   baseline.totalLoad.stddev);
  double z_ratio = zScore(f.ratio,      baseline.ratio.mean,       baseline.ratio.stddev);

  double score = z_rms   * 1.0
               + z_cf    * 1.5
               + z_kurt  * 2.0
               + z_freq  * 2.0
               + z_load  * 1.0
               + z_ratio * 1.5;

  // Trend rising check
  trend_rms[trend_idx] = f.rms;
  trend_idx = (trend_idx + 1) % TREND_WINDOW;
  if (trend_count < TREND_WINDOW) trend_count++;

  bool trendRising = false;
  if (trend_count >= TREND_WINDOW) {
    double sumOld = 0, sumNew = 0;
    int half = TREND_WINDOW / 2;
    for (int i = 0; i < half; i++) {
      sumOld += trend_rms[(trend_idx + i) % TREND_WINDOW];
      sumNew += trend_rms[(trend_idx + half + i) % TREND_WINDOW];
    }
    trendRising = (sumNew / half) > (sumOld / half) * 1.05;
  }

  // Simpan ke latest
  latest.z_rms       = z_rms;
  latest.z_cf        = z_cf;
  latest.z_kurt      = z_kurt;
  latest.z_freq      = z_freq;
  latest.z_load      = z_load;
  latest.z_ratio     = z_ratio;
  latest.score       = score;
  latest.trendRising = trendRising;

  if (score > 12.0 || z_kurt > 4.0 || (trendRising && score > 6.0)) return 2;
  if (score > 6.0  || z_kurt > 2.5)                                  return 1;
  return 0;
}

// ============================================================
//  UPDATE LATEST PAYLOAD
// ============================================================
void updateLatest(const SHMFeatures &f, int anomalyLevel) {
  latest.rms         = f.rms;
  latest.peak        = f.peak;
  latest.crestFactor = f.crestFactor;
  latest.kurtosis    = f.kurtosis;
  latest.freqPeak1   = f.freqPeak1;
  latest.freqPeak2   = f.freqPeak2;
  latest.freqPeak3   = f.freqPeak3;
  for (int i = 0; i < 4; i++) latest.bandEnergy[i] = f.freqBandEnergy[i];
  latest.totalLoad   = f.totalLoad;
  latest.ratio       = f.ratio;
  latest.anomalyLevel = anomalyLevel;
  latest.statusStr   = (anomalyLevel == 2) ? "ALERT"
                     : (anomalyLevel == 1) ? "WARNING"
                     :                       "NORMAL";
  latest.ts          = millis();
}

// ============================================================
//  WIFI & HTTP SERVER
// ============================================================
void addCORSHeaders() {
  server.sendHeader("Access-Control-Allow-Origin",  "*");
  server.sendHeader("Access-Control-Allow-Methods", "GET, POST, OPTIONS");
  server.sendHeader("Access-Control-Allow-Headers", "Content-Type");
  server.sendHeader("Cache-Control",                "no-cache");
}

// GET /
void handleRoot() {
  addCORSHeaders();
  String html = "<html><body style='font-family:monospace;padding:20px'>";
  html += "<h2>SHM ESP32 API</h2>";
  html += "<p>IP: " + WiFi.localIP().toString() + "</p>";
  html += "<p>Endpoints:</p><ul>";
  html += "<li>GET /status</li>";
  html += "<li>GET /data</li>";
  html += "<li>GET /baseline</li>";
  html += "<li>POST /cmd  (body: cmd=b|m|t|r)</li>";
  html += "</ul></body></html>";
  server.send(200, "text/html", html);
}

// GET /status
void handleStatus() {
  addCORSHeaders();
  StaticJsonDocument<200> doc;
  doc["mode"]          = (mode == MODE_IDLE) ? "IDLE"
                       : (mode == MODE_BASELINE) ? "BASELINE" : "MONITOR";
  doc["baselineValid"] = baseline.valid;
  doc["baselineCount"] = baseline_count;
  doc["baselineTotal"] = N_BASELINE;
  doc["ip"]            = WiFi.localIP().toString();
  doc["uptime_ms"]     = millis();
  String out;
  serializeJson(doc, out);
  server.send(200, "application/json", out);
}

// GET /data
void handleData() {
  addCORSHeaders();
  StaticJsonDocument<768> doc;

  doc["status"]      = latest.statusStr;
  doc["rms"]         = String(latest.rms,         4);
  doc["peak"]        = String(latest.peak,         4);
  doc["crestFactor"] = String(latest.crestFactor,  4);
  doc["kurtosis"]    = String(latest.kurtosis,     4);
  doc["freqPeak1"]   = String(latest.freqPeak1,    4);
  doc["freqPeak2"]   = String(latest.freqPeak2,    4);
  doc["freqPeak3"]   = String(latest.freqPeak3,    4);
  doc["totalLoad"]   = String(latest.totalLoad,    3);
  doc["ratio"]       = String(latest.ratio,        4);
  doc["z_rms"]       = String(latest.z_rms,        3);
  doc["z_cf"]        = String(latest.z_cf,         3);
  doc["z_kurt"]      = String(latest.z_kurt,       3);
  doc["z_freq"]      = String(latest.z_freq,       3);
  doc["z_load"]      = String(latest.z_load,       3);
  doc["z_ratio"]     = String(latest.z_ratio,      3);
  doc["score"]       = String(latest.score,        2);
  doc["trendRising"] = latest.trendRising;
  doc["timestamp"]   = latest.ts;
  doc["mode"]        = (mode == MODE_IDLE) ? "IDLE"
                     : (mode == MODE_BASELINE) ? "BASELINE" : "MONITOR";

  JsonArray band = doc.createNestedArray("bandEnergy");
  for (int i = 0; i < 4; i++) band.add(String(latest.bandEnergy[i], 2));

  // Progress baseline jika sedang berjalan
  if (mode == MODE_BASELINE) {
    doc["baselineProgress"] = baseline_count;
    doc["baselineTotal"]    = N_BASELINE;
  }

  String out;
  serializeJson(doc, out);
  server.send(200, "application/json", out);
}

// GET /baseline
void handleBaseline() {
  addCORSHeaders();
  StaticJsonDocument<512> doc;
  doc["valid"] = baseline.valid;
  if (baseline.valid) {
    JsonObject rms  = doc.createNestedObject("rms");
    rms["mean"]     = String(baseline.rms.mean,          4);
    rms["stddev"]   = String(baseline.rms.stddev,        4);
    JsonObject cf   = doc.createNestedObject("crestFactor");
    cf["mean"]      = String(baseline.crestFactor.mean,  4);
    cf["stddev"]    = String(baseline.crestFactor.stddev,4);
    JsonObject kurt = doc.createNestedObject("kurtosis");
    kurt["mean"]    = String(baseline.kurtosis.mean,     4);
    kurt["stddev"]  = String(baseline.kurtosis.stddev,   4);
    JsonObject freq = doc.createNestedObject("freqPeak1");
    freq["mean"]    = String(baseline.freqPeak1.mean,    4);
    freq["stddev"]  = String(baseline.freqPeak1.stddev,  4);
    JsonObject load = doc.createNestedObject("totalLoad");
    load["mean"]    = String(baseline.totalLoad.mean,    4);
    load["stddev"]  = String(baseline.totalLoad.stddev,  4);
    JsonObject rat  = doc.createNestedObject("ratio");
    rat["mean"]     = String(baseline.ratio.mean,        4);
    rat["stddev"]   = String(baseline.ratio.stddev,      4);
  }
  String out;
  serializeJson(doc, out);
  server.send(200, "application/json", out);
}

// POST /cmd  — body: cmd=b|m|t|r
void handleCmd() {
  addCORSHeaders();

  // Preflight OPTIONS
  if (server.method() == HTTP_OPTIONS) {
    server.send(204);
    return;
  }

  // Ambil parameter cmd dari form-data atau raw body
  String cmd = "";
  if (server.hasArg("cmd"))   cmd = server.arg("cmd");
  if (cmd == "" && server.hasArg("plain")) {
    String body = server.arg("plain");
    // parse "cmd=b"
    int idx = body.indexOf("cmd=");
    if (idx >= 0) cmd = body.substring(idx + 4, idx + 5);
  }
  cmd.trim();

  Serial.print("[API CMD] Received: ");
  Serial.println(cmd);

  if (cmd == "b") {
    memset(buf_rms,      0, sizeof(buf_rms));
    memset(buf_crest,    0, sizeof(buf_crest));
    memset(buf_kurtosis, 0, sizeof(buf_kurtosis));
    memset(buf_freq,     0, sizeof(buf_freq));
    memset(buf_load,     0, sizeof(buf_load));
    memset(buf_ratio,    0, sizeof(buf_ratio));
    baseline_count = 0;
    baseline.valid = false;
    latest.statusStr = "BASELINE";
    mode = MODE_BASELINE;
    server.send(200, "application/json", "{\"ok\":true,\"msg\":\"Baseline started\"}");

  } else if (cmd == "m") {
    if (!baseline.valid) {
      server.send(400, "application/json", "{\"ok\":false,\"msg\":\"Baseline not ready\"}");
      return;
    }
    trend_idx = 0; trend_count = 0;
    latest.statusStr = "NORMAL";
    mode = MODE_MONITOR;
    server.send(200, "application/json", "{\"ok\":true,\"msg\":\"Monitor started\"}");

  } else if (cmd == "t") {
    scale1.tare(); scale2.tare();
    server.send(200, "application/json", "{\"ok\":true,\"msg\":\"Tare done\"}");

  } else if (cmd == "r") {
    prefs.begin("shm_base", false);
    prefs.clear();
    prefs.end();
    baseline.valid   = false;
    latest.statusStr = "IDLE";
    mode = MODE_IDLE;
    server.send(200, "application/json", "{\"ok\":true,\"msg\":\"Baseline reset\"}");

  } else {
    server.send(400, "application/json", "{\"ok\":false,\"msg\":\"Unknown command\"}");
  }
}

// ============================================================
//  SETUP WIFI + SERVER
// ============================================================
void setupWiFi() {
  Serial.print("[WiFi] Connecting to: ");
  Serial.println(WIFI_SSID);

  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  int tries = 0;
  while (WiFi.status() != WL_CONNECTED && tries < 30) {
    delay(500);
    Serial.print(".");
    tries++;
  }
  Serial.println();

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("[WiFi] Connected!");
    Serial.print("[WiFi] IP Address : http://");
    Serial.println(WiFi.localIP());

    server.on("/",          HTTP_GET,     handleRoot);
    server.on("/status",    HTTP_GET,     handleStatus);
    server.on("/data",      HTTP_GET,     handleData);
    server.on("/baseline",  HTTP_GET,     handleBaseline);
    server.on("/cmd",       HTTP_POST,    handleCmd);
    server.on("/cmd",       HTTP_OPTIONS, handleCmd);

    server.begin();
    Serial.println("[WiFi] HTTP server started on port 80");
    Serial.println("------------------------------------------");
    Serial.print(">> Buka SHM_Dashboard.html, masukkan IP: ");
    Serial.println(WiFi.localIP());
    Serial.println("------------------------------------------");

  } else {
    Serial.println("[WiFi] GAGAL connect! Cek SSID/password.");
    Serial.println("[WiFi] Lanjut tanpa WiFi (Serial only).");
  }
}

// ============================================================
//  PROSES COMMAND DARI SERIAL MONITOR
// ============================================================
void checkSerialCmd() {
  if (!Serial.available()) return;
  char cmd = Serial.read();

  if (cmd == 'b') {
    memset(buf_rms,      0, sizeof(buf_rms));
    memset(buf_crest,    0, sizeof(buf_crest));
    memset(buf_kurtosis, 0, sizeof(buf_kurtosis));
    memset(buf_freq,     0, sizeof(buf_freq));
    memset(buf_load,     0, sizeof(buf_load));
    memset(buf_ratio,    0, sizeof(buf_ratio));
    baseline_count = 0;
    baseline.valid = false;
    mode = MODE_BASELINE;
    Serial.println("[MODE] BASELINE dimulai");

  } else if (cmd == 'm') {
    if (!baseline.valid) { Serial.println("[ERROR] Baseline belum ada!"); return; }
    trend_idx = 0; trend_count = 0;
    mode = MODE_MONITOR;
    Serial.println("[MODE] MONITOR dimulai");

  } else if (cmd == 't') {
    scale1.tare(); scale2.tare();
    Serial.println("[TARE] Load cell di-tare.");

  } else if (cmd == 'r') {
    prefs.begin("shm_base", false);
    prefs.clear();
    prefs.end();
    baseline.valid = false;
    mode = MODE_IDLE;
    Serial.println("[RESET] Baseline NVS dihapus.");

  } else if (cmd == 's') {
    Serial.print("[STATUS] Mode=");
    Serial.print(mode == MODE_IDLE ? "IDLE" : mode == MODE_BASELINE ? "BASELINE" : "MONITOR");
    Serial.print(" | Baseline=");
    Serial.println(baseline.valid ? "VALID" : "NOT SET");
    Serial.print("[STATUS] WiFi=");
    Serial.println(WiFi.status() == WL_CONNECTED ? WiFi.localIP().toString() : "NOT CONNECTED");
  }
}

// ============================================================
//  PRINT JSON KE SERIAL
// ============================================================
void printSerialJSON(const SHMFeatures &f, int anomaly) {
  String status = (anomaly == 2) ? "ALERT" : (anomaly == 1) ? "WARNING" : "NORMAL";
  Serial.print("{\"status\":\""); Serial.print(status);
  Serial.print("\",\"rms\":"); Serial.print(f.rms, 4);
  Serial.print(",\"cf\":"); Serial.print(f.crestFactor, 4);
  Serial.print(",\"kurt\":"); Serial.print(f.kurtosis, 4);
  Serial.print(",\"freq\":"); Serial.print(f.freqPeak1, 4);
  Serial.print(",\"load\":"); Serial.print(f.totalLoad, 3);
  Serial.print(",\"ratio\":"); Serial.print(f.ratio, 4);
  Serial.print(",\"score\":"); Serial.print(latest.score, 2);
  Serial.println("}");
}

// ============================================================
//  SETUP
// ============================================================
void setup() {
  Serial.begin(115200);
  delay(500);

  Serial.println("==========================================");
  Serial.println("  SHM SYSTEM — FULL INTEGRATED");
  Serial.println("==========================================");

  // MPU6050
  I2CMPU.begin(21, 22);
  if (!mpu.begin(0x68, &I2CMPU)) {
    Serial.println("[ERROR] MPU6050 tidak ditemukan! Cek wiring.");
    while (1) delay(500);
  }
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  Serial.println("[OK] MPU6050 ready");

  // Load Cells
  scale1.begin(LOADCELL1_DOUT, LOADCELL1_SCK);
  scale2.begin(LOADCELL2_DOUT, LOADCELL2_SCK);
  scale1.set_scale(calibFactor1);
  scale2.set_scale(calibFactor2);
  scale1.tare();
  scale2.tare();
  Serial.println("[OK] Load cells ready");

  // Load baseline dari NVS
  loadBaselineFromNVS();

  // WiFi + HTTP Server
  setupWiFi();

  Serial.println("------------------------------------------");
  Serial.println("SERIAL COMMANDS:");
  Serial.println("  b → baseline  m → monitor");
  Serial.println("  t → tare      r → reset    s → status");
  Serial.println("==========================================");
}

// ============================================================
//  LOOP UTAMA
// ============================================================
void loop() {
  // Handle HTTP request di setiap awal loop
  server.handleClient();

  // Cek command dari Serial Monitor
  checkSerialCmd();

  // ──────────────── MODE BASELINE ────────────────
  if (mode == MODE_BASELINE) {
    Serial.printf("[BASELINE] Sampel %d / %d\n", baseline_count + 1, N_BASELINE);

    SHMFeatures f;
    if (acquireFeatures(f)) {
      buf_rms[baseline_count]      = f.rms;
      buf_crest[baseline_count]    = f.crestFactor;
      buf_kurtosis[baseline_count] = f.kurtosis;
      buf_freq[baseline_count]     = f.freqPeak1;
      buf_load[baseline_count]     = f.totalLoad;
      buf_ratio[baseline_count]    = f.ratio;
      baseline_count++;

      // Update latest agar dashboard bisa pantau progress
      latest.rms         = f.rms;
      latest.crestFactor = f.crestFactor;
      latest.kurtosis    = f.kurtosis;
      latest.freqPeak1   = f.freqPeak1;
      latest.totalLoad   = f.totalLoad;
      latest.ratio       = f.ratio;
      latest.statusStr   = "BASELINE";
      latest.ts          = millis();

      Serial.printf("  rms=%.4f cf=%.4f kurt=%.4f freq=%.3f\n",
                    f.rms, f.crestFactor, f.kurtosis, f.freqPeak1);

      if (baseline_count >= N_BASELINE) {
        computeBaselineStats();
        saveBaselineToNVS();

        Serial.println("\n=== BASELINE SELESAI ===");
        Serial.printf("RMS      mean=%.4f std=%.4f\n", baseline.rms.mean,        baseline.rms.stddev);
        Serial.printf("CrestFac mean=%.4f std=%.4f\n", baseline.crestFactor.mean, baseline.crestFactor.stddev);
        Serial.printf("Kurtosis mean=%.4f std=%.4f\n", baseline.kurtosis.mean,    baseline.kurtosis.stddev);
        Serial.printf("FreqPeak mean=%.4f std=%.4f\n", baseline.freqPeak1.mean,   baseline.freqPeak1.stddev);
        Serial.printf("Load     mean=%.4f std=%.4f\n", baseline.totalLoad.mean,   baseline.totalLoad.stddev);
        Serial.printf("Ratio    mean=%.4f std=%.4f\n", baseline.ratio.mean,       baseline.ratio.stddev);
        Serial.println("========================");
        Serial.println("Ketik 'm' atau klik Monitor di dashboard.");

        latest.statusStr = "IDLE";
        mode = MODE_IDLE;
      }
    }
    delay(100);
  }

  // ──────────────── MODE MONITOR ────────────────
  else if (mode == MODE_MONITOR) {
    SHMFeatures f;
    if (acquireFeatures(f)) {
      int anomaly = detectAnomaly(f);
      updateLatest(f, anomaly);
      printSerialJSON(f, anomaly);

      if (anomaly == 2) Serial.println("!!! ALERT — PERIKSA STRUKTUR !!!");
      else if (anomaly == 1) Serial.println("!! WARNING !!");
    }
    delay(50);
  }

  // ──────────────── MODE IDLE ────────────────
  else {
    // Tetap handle client, tidak memblokir
    delay(10);
  }
}
