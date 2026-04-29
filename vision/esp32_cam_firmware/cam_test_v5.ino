// ============================================================================
// cam_test_v5.ino
//
// CHANGES vs v4:
//   - CONNECTED-COMPONENTS LABELING. Instead of treating all dark pixels as
//     one giant blob, trace them into separate connected components, evaluate
//     each one's shape independently, and pick the most circle-like one.
//     This is what fixes the "scattered" failure mode where dark stuff in the
//     scene was getting lumped in with the marker.
//
//     Algorithm: two-pass union-find labeling, 4-connected.
//       Pass 1: scan top-to-bottom, left-to-right. For each dark pixel, look
//               at its already-labeled left + up neighbors. If both labeled,
//               union them. If one labeled, copy that label. If neither,
//               assign a new label.
//       Pass 2: resolve labels (every label points to its root) and gather
//               per-component stats (area, bbox, perimeter, centroid).
//
//   - For each candidate that meets size + shape thresholds, score it by:
//       score = fill_ratio * (2 - aspect) * circularity
//     Higher = more circle-like. Pick the highest-scoring component.
//
//   - 'h' BRIGHTNESS HISTOGRAM command for diagnosing exposure / threshold.
//
//   - Memory: uses ~965KB of PSRAM for the label map (uint16_t per pixel).
//     Allocated once at boot. SVGA = 480k pixels. Up to 65535 components.
//
// ============================================================================

#include "esp_camera.h"
#include "EEPROM.h"
#include "esp_heap_caps.h"
#include <math.h>

// Forward-declare struct so Arduino's auto-prototypes don't choke
struct ComponentStats;
struct BlobStats;

// ---------------------------------------------------------------------------
// AI-Thinker ESP32-CAM pin map
// ---------------------------------------------------------------------------
#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27
#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5
#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22
#define LED_GPIO_NUM       4

// ---------------------------------------------------------------------------
// Frame size — SVGA (800x600)
// ---------------------------------------------------------------------------
const int FRAME_W = 800;
const int FRAME_H = 600;
const int FRAME_PIXELS = FRAME_W * FRAME_H;

// ---------------------------------------------------------------------------
// Detection settings (runtime-tunable, EEPROM-saved)
// ---------------------------------------------------------------------------
int   threshold     = 60;
int   min_area      = 80;
int   max_area      = 150000;
bool  debug_ascii   = false;

// Shape filtering
float min_circularity = 0.40f;
float max_aspect      = 2.00f;
float min_fill        = 0.55f;

// ---------------------------------------------------------------------------
// Camera intrinsics + marker
// ---------------------------------------------------------------------------
float MARKER_DIAMETER_MM = 50.0f;
float FOCAL_LENGTH_PX    = 550.0f;
float OPTICAL_CENTER_X   = FRAME_W / 2.0f;
float OPTICAL_CENTER_Y   = FRAME_H / 2.0f;

// ---------------------------------------------------------------------------
// LED pulse on detection state change
// ---------------------------------------------------------------------------
const unsigned long LED_PULSE_MS = 30;
unsigned long led_off_at = 0;
bool prev_found = false;

// ---------------------------------------------------------------------------
// Last-frame stash for the calibration command
// ---------------------------------------------------------------------------
long  last_area = 0;
float last_cx   = 0;
float last_cy   = 0;
bool  have_last = false;

// ---------------------------------------------------------------------------
// FPS
// ---------------------------------------------------------------------------
unsigned long last_fps_print = 0;
unsigned int  frames_since_print = 0;
float         current_fps = 0;

// ---------------------------------------------------------------------------
// Exposure lock state
// ---------------------------------------------------------------------------
bool exposure_locked = false;
int  saved_aec_value = -1;
int  saved_agc_gain  = -1;

// ---------------------------------------------------------------------------
// Connected-components storage (allocated once in setup, in PSRAM)
// ---------------------------------------------------------------------------
const uint16_t MAX_COMPONENTS = 4000;  // enough for very noisy scenes
uint16_t* g_labels   = nullptr;        // FRAME_PIXELS uint16_t in PSRAM
uint16_t* g_parent   = nullptr;        // MAX_COMPONENTS+1, union-find parents
struct ComponentStats {
  uint32_t area;
  uint32_t sum_x;
  uint32_t sum_y;
  uint16_t bb_x0, bb_y0, bb_x1, bb_y1;
  uint32_t perimeter;
};
ComponentStats* g_stats = nullptr;     // MAX_COMPONENTS+1

// ===========================================================================
// EEPROM (same as v4)
// ===========================================================================
const uint32_t CAM_EEPROM_MAGIC = 0x43414D32;
const size_t   CAM_EEPROM_SIZE  = 256;

void cam_eeprom_save() {
  size_t a = 0;
  EEPROM.put(a, CAM_EEPROM_MAGIC); a += sizeof(uint32_t);
  EEPROM.put(a, threshold);        a += sizeof(int32_t);
  EEPROM.put(a, min_area);         a += sizeof(int32_t);
  EEPROM.put(a, max_area);         a += sizeof(int32_t);
  EEPROM.put(a, FOCAL_LENGTH_PX);  a += sizeof(float);
  EEPROM.put(a, MARKER_DIAMETER_MM); a += sizeof(float);
  EEPROM.put(a, min_circularity);  a += sizeof(float);
  EEPROM.put(a, max_aspect);       a += sizeof(float);
  EEPROM.put(a, min_fill);         a += sizeof(float);
  uint8_t lock = exposure_locked ? 1 : 0;
  EEPROM.put(a, lock);             a += sizeof(uint8_t);
  EEPROM.put(a, saved_aec_value);  a += sizeof(int32_t);
  EEPROM.put(a, saved_agc_gain);   a += sizeof(int32_t);
  if (EEPROM.commit()) Serial.println("OK  saved to EEPROM.");
  else Serial.println("ERR EEPROM commit failed");
}

bool cam_eeprom_load() {
  size_t a = 0;
  uint32_t magic = 0;
  EEPROM.get(a, magic); a += sizeof(uint32_t);
  if (magic != CAM_EEPROM_MAGIC) return false;

  int32_t i32; float f32; uint8_t u8;
  EEPROM.get(a, i32); a += sizeof(int32_t); threshold = i32;
  EEPROM.get(a, i32); a += sizeof(int32_t); min_area = i32;
  EEPROM.get(a, i32); a += sizeof(int32_t); max_area = i32;
  EEPROM.get(a, f32); a += sizeof(float);   FOCAL_LENGTH_PX = f32;
  EEPROM.get(a, f32); a += sizeof(float);   MARKER_DIAMETER_MM = f32;
  EEPROM.get(a, f32); a += sizeof(float);   min_circularity = f32;
  EEPROM.get(a, f32); a += sizeof(float);   max_aspect = f32;
  EEPROM.get(a, f32); a += sizeof(float);   min_fill = f32;
  EEPROM.get(a, u8);  a += sizeof(uint8_t); exposure_locked = (u8 != 0);
  EEPROM.get(a, i32); a += sizeof(int32_t); saved_aec_value = i32;
  EEPROM.get(a, i32); a += sizeof(int32_t); saved_agc_gain = i32;

  if (threshold < 0 || threshold > 255) return false;
  if (min_area < 1 || min_area > 1000000) return false;
  if (max_area < min_area) return false;
  if (!isfinite(FOCAL_LENGTH_PX) || FOCAL_LENGTH_PX < 1.0f) return false;
  if (!isfinite(MARKER_DIAMETER_MM) || MARKER_DIAMETER_MM < 0.1f) return false;
  if (!isfinite(min_circularity) || min_circularity < 0 || min_circularity > 1) return false;
  if (!isfinite(max_aspect) || max_aspect < 1.0f || max_aspect > 10.0f) return false;
  if (!isfinite(min_fill) || min_fill < 0 || min_fill > 1) return false;
  return true;
}

void cam_eeprom_clear() {
  for (size_t i = 0; i < CAM_EEPROM_SIZE; i++) EEPROM.write(i, 0xFF);
  if (EEPROM.commit()) Serial.println("OK  EEPROM wiped. Reset to apply defaults.");
  else Serial.println("ERR EEPROM commit failed");
}

void apply_saved_exposure() {
  if (!exposure_locked || saved_aec_value < 0 || saved_agc_gain < 0) return;
  sensor_t * s = esp_camera_sensor_get();
  if (!s) return;
  s->set_exposure_ctrl(s, 0);
  s->set_aec2(s, 0);
  s->set_gain_ctrl(s, 0);
  s->set_aec_value(s, saved_aec_value);
  s->set_agc_gain(s, saved_agc_gain);
  Serial.print("[exp] restored aec="); Serial.print(saved_aec_value);
  Serial.print(" agc="); Serial.println(saved_agc_gain);
}

void capture_current_exposure() {
  sensor_t * s = esp_camera_sensor_get();
  if (!s) { saved_aec_value = -1; saved_agc_gain = -1; return; }
  saved_aec_value = s->status.aec_value;
  saved_agc_gain  = s->status.agc_gain;
}

// ===========================================================================
// CONNECTED-COMPONENTS LABELING
// ===========================================================================
// Two-pass union-find. Labels start at 1 (0 = "not dark").
//
// Performance: ~80ms on ESP32 at 240MHz for SVGA. Most of the cost is the
// Pass 1 scan. We use uint16_t labels which is enough for many thousands
// of small noise specks.

// Find root of label `x` with path compression
static inline uint16_t uf_find(uint16_t x) {
  while (g_parent[x] != x) {
    g_parent[x] = g_parent[g_parent[x]];  // path compression (one step)
    x = g_parent[x];
  }
  return x;
}

static inline void uf_union(uint16_t a, uint16_t b) {
  a = uf_find(a);
  b = uf_find(b);
  if (a == b) return;
  // Lower label wins (keeps labels small for cache locality)
  if (a < b) g_parent[b] = a;
  else       g_parent[a] = b;
}

// Pass 1: assign provisional labels with union-find for merges.
// Returns number of provisional labels (could be more than final count).
uint16_t pass1_label(const uint8_t *frame) {
  uint16_t next_label = 1;
  // Set up parent[0] = 0 (background)
  g_parent[0] = 0;

  for (int y = 0; y < FRAME_H; y++) {
    const uint8_t *row = frame + y * FRAME_W;
    for (int x = 0; x < FRAME_W; x++) {
      int idx = y * FRAME_W + x;
      if (row[x] >= threshold) {
        g_labels[idx] = 0;  // background
        continue;
      }
      // dark pixel - get neighbors' labels (4-connected: left + up)
      uint16_t leftL = (x > 0) ? g_labels[idx - 1]      : 0;
      uint16_t upL   = (y > 0) ? g_labels[idx - FRAME_W] : 0;

      if (leftL == 0 && upL == 0) {
        // new component
        if (next_label >= MAX_COMPONENTS) {
          // ran out of labels; merge into label 1 to avoid overflow
          g_labels[idx] = 1;
        } else {
          g_labels[idx] = next_label;
          g_parent[next_label] = next_label;
          next_label++;
        }
      } else if (leftL != 0 && upL == 0) {
        g_labels[idx] = leftL;
      } else if (leftL == 0 && upL != 0) {
        g_labels[idx] = upL;
      } else {
        // both dark - union them and use the smaller
        uf_union(leftL, upL);
        g_labels[idx] = (leftL < upL) ? leftL : upL;
      }
    }
  }
  return next_label;
}

// Pass 2: resolve each pixel's label to its root, gather per-component stats.
// Returns number of components that have any pixels.
void pass2_stats(const uint8_t *frame, uint16_t n_labels) {
  // zero out stats
  for (uint16_t i = 0; i <= n_labels && i < MAX_COMPONENTS; i++) {
    g_stats[i].area = 0;
    g_stats[i].sum_x = 0;
    g_stats[i].sum_y = 0;
    g_stats[i].bb_x0 = FRAME_W;
    g_stats[i].bb_y0 = FRAME_H;
    g_stats[i].bb_x1 = 0;
    g_stats[i].bb_y1 = 0;
    g_stats[i].perimeter = 0;
  }

  for (int y = 0; y < FRAME_H; y++) {
    const uint8_t *row = frame + y * FRAME_W;
    for (int x = 0; x < FRAME_W; x++) {
      int idx = y * FRAME_W + x;
      uint16_t L = g_labels[idx];
      if (L == 0) continue;
      // resolve to root
      L = uf_find(L);
      g_labels[idx] = L;  // collapse for downstream lookups (optional)

      ComponentStats &cs = g_stats[L];
      cs.area++;
      cs.sum_x += x;
      cs.sum_y += y;
      if (x < cs.bb_x0) cs.bb_x0 = x;
      if (y < cs.bb_y0) cs.bb_y0 = y;
      if (x > cs.bb_x1) cs.bb_x1 = x;
      if (y > cs.bb_y1) cs.bb_y1 = y;

      // perimeter: dark pixel with at least one non-this-component 4-neighbor
      bool boundary = false;
      if (x == 0 || row[x - 1] >= threshold) boundary = true;
      else if (x == FRAME_W - 1 || row[x + 1] >= threshold) boundary = true;
      else if (y == 0 || frame[idx - FRAME_W] >= threshold) boundary = true;
      else if (y == FRAME_H - 1 || frame[idx + FRAME_W] >= threshold) boundary = true;
      if (boundary) cs.perimeter++;
    }
  }
}

// ===========================================================================
// SETUP
// ===========================================================================
void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println();
  Serial.println("===== ESP32-CAM step 5 (connected components) =====");

  pinMode(LED_GPIO_NUM, OUTPUT);
  digitalWrite(LED_GPIO_NUM, LOW);

  if (!EEPROM.begin(CAM_EEPROM_SIZE)) {
    Serial.println("[eeprom] init FAILED, settings will not persist");
  }

  // PSRAM allocation for connected-components arrays.
  // labels: 480000 * 2 bytes = 960KB
  // parent: 4001 * 2 bytes ≈ 8KB
  // stats:  4001 * sizeof(ComponentStats) ≈ 88KB
  g_labels = (uint16_t*)heap_caps_malloc(FRAME_PIXELS * sizeof(uint16_t), MALLOC_CAP_SPIRAM);
  g_parent = (uint16_t*)heap_caps_malloc((MAX_COMPONENTS + 1) * sizeof(uint16_t), MALLOC_CAP_SPIRAM);
  g_stats  = (ComponentStats*)heap_caps_malloc((MAX_COMPONENTS + 1) * sizeof(ComponentStats), MALLOC_CAP_SPIRAM);
  if (!g_labels || !g_parent || !g_stats) {
    Serial.println("PSRAM ALLOC FAILED — check PSRAM is enabled in Tools menu.");
    while (true) { digitalWrite(LED_GPIO_NUM, !digitalRead(LED_GPIO_NUM)); delay(150); }
  }
  Serial.printf("[mem] PSRAM used: labels=%d KB  parent=%d KB  stats=%d KB\n",
                (FRAME_PIXELS * 2) / 1024,
                ((MAX_COMPONENTS + 1) * 2) / 1024,
                ((MAX_COMPONENTS + 1) * (int)sizeof(ComponentStats)) / 1024);

  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer   = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM; config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM; config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM; config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM; config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk  = XCLK_GPIO_NUM;  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM; config.pin_href = HREF_GPIO_NUM;
  config.pin_sccb_sda = SIOD_GPIO_NUM; config.pin_sccb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn  = PWDN_GPIO_NUM;  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_GRAYSCALE;
  config.frame_size   = FRAMESIZE_SVGA;
  config.jpeg_quality = 12;
  config.fb_count     = 2;
  config.fb_location  = CAMERA_FB_IN_PSRAM;
  config.grab_mode    = CAMERA_GRAB_LATEST;

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init FAILED: 0x%x\n", err);
    while (true) { digitalWrite(LED_GPIO_NUM, !digitalRead(LED_GPIO_NUM)); delay(150); }
  }
  Serial.println("Camera init OK at SVGA 800x600.");

  sensor_t * s = esp_camera_sensor_get();
  if (s) {
    s->set_brightness(s, 0);
    s->set_contrast(s, 1);
    s->set_saturation(s, 0);
    s->set_whitebal(s, 1);
    s->set_awb_gain(s, 1);
    s->set_exposure_ctrl(s, 1);
    s->set_aec2(s, 1);
    s->set_gain_ctrl(s, 1);
    s->set_gainceiling(s, (gainceiling_t)2);
    s->set_hmirror(s, 0);
    s->set_vflip(s, 0);
  }

  if (cam_eeprom_load()) {
    Serial.println("[eeprom] loaded saved settings.");
    if (exposure_locked) apply_saved_exposure();
  } else {
    Serial.println("[eeprom] no valid saved settings, using defaults.");
  }

  Serial.printf("Marker D = %.2f mm   focal_px = %.1f\n", MARKER_DIAMETER_MM, FOCAL_LENGTH_PX);
  Serial.printf("Shape: min_circ=%.2f max_aspect=%.2f min_fill=%.2f\n",
                min_circularity, max_aspect, min_fill);
  Serial.printf("Exposure: %s\n", exposure_locked ? "LOCKED" : "auto");
  Serial.println("Type ? for help.");
  Serial.println();
}

// ===========================================================================
// LOOP
// ===========================================================================
String inputBuf = "";

void loop() {
  handle_serial();

  camera_fb_t * fb = esp_camera_fb_get();
  if (!fb) {
    Serial.println("ERR  capture failed");
    delay(50);
    return;
  }

  frames_since_print++;
  unsigned long now = millis();
  if (now - last_fps_print >= 1000) {
    current_fps = frames_since_print * 1000.0f / (now - last_fps_print);
    frames_since_print = 0;
    last_fps_print = now;
  }

  // Pass 1: label dark pixels
  uint16_t n_labels = pass1_label(fb->buf);

  // Pass 2: resolve labels and gather stats
  pass2_stats(fb->buf, n_labels);

  // Find best circle-like candidate
  uint16_t best_label = 0;
  float    best_score = 0.0f;
  uint16_t n_candidates = 0;
  for (uint16_t L = 1; L < n_labels && L < MAX_COMPONENTS; L++) {
    // Skip merged-out labels (they have area=0 because we only count pixels
    // by their root label)
    ComponentStats &cs = g_stats[L];
    if (cs.area == 0) continue;
    if ((int)cs.area < min_area || (int)cs.area > max_area) continue;

    int bb_w = cs.bb_x1 - cs.bb_x0 + 1;
    int bb_h = cs.bb_y1 - cs.bb_y0 + 1;
    if (bb_w <= 0 || bb_h <= 0) continue;

    float aspect = (bb_w > bb_h) ? (float)bb_w / bb_h : (float)bb_h / bb_w;
    float fill   = (float)cs.area / ((float)bb_w * (float)bb_h);
    float circ   = (cs.perimeter > 0)
                   ? (4.0f * (float)M_PI * cs.area) / ((float)cs.perimeter * cs.perimeter)
                   : 0.0f;
    if (circ > 1.0f) circ = 1.0f;

    if (aspect > max_aspect)        continue;
    if (fill   < min_fill)          continue;
    if (circ   < min_circularity)   continue;

    n_candidates++;
    // score: higher is better. Prefer round, square-bbox, well-filled blobs.
    float score = circ * fill * (2.0f - aspect);
    if (score > best_score) {
      best_score = score;
      best_label = L;
    }
  }

  if (best_label != 0) {
    ComponentStats &cs = g_stats[best_label];
    float cx = (float)cs.sum_x / cs.area;
    float cy = (float)cs.sum_y / cs.area;
    int bb_w = cs.bb_x1 - cs.bb_x0 + 1;
    int bb_h = cs.bb_y1 - cs.bb_y0 + 1;
    float aspect = (bb_w > bb_h) ? (float)bb_w / bb_h : (float)bb_h / bb_w;
    float fill   = (float)cs.area / ((float)bb_w * (float)bb_h);
    float circ   = (4.0f * (float)M_PI * cs.area) / ((float)cs.perimeter * cs.perimeter);
    if (circ > 1.0f) circ = 1.0f;

    float d_px = 2.0f * sqrtf((float)cs.area / (float)M_PI);
    float Z_mm = (MARKER_DIAMETER_MM * FOCAL_LENGTH_PX) / d_px;
    float X_mm = (cx - OPTICAL_CENTER_X) * Z_mm / FOCAL_LENGTH_PX;
    float Y_mm = -(cy - OPTICAL_CENTER_Y) * Z_mm / FOCAL_LENGTH_PX;

    last_area = cs.area;
    last_cx   = cx;
    last_cy   = cy;
    have_last = true;

    if (!prev_found) {
      digitalWrite(LED_GPIO_NUM, HIGH);
      led_off_at = millis() + LED_PULSE_MS;
    }

    Serial.printf("FOUND  XYZ=(%7.1f, %7.1f, %7.1f) mm  area=%6lu  fill=%.2f  circ=%.2f  asp=%.2f  cands=%d  fps=%4.1f%s\n",
                  X_mm, Y_mm, Z_mm, (unsigned long)cs.area, fill, circ, aspect,
                  n_candidates, current_fps, exposure_locked ? "  [EX]" : "");
    prev_found = true;
  } else {
    // Find the largest dark blob (any size, any shape) for diagnostics
    uint16_t big_label = 0;
    uint32_t big_area  = 0;
    for (uint16_t L = 1; L < n_labels && L < MAX_COMPONENTS; L++) {
      if (g_stats[L].area > big_area) {
        big_area = g_stats[L].area;
        big_label = L;
      }
    }
    if (big_label != 0 && big_area >= 10) {
      ComponentStats &cs = g_stats[big_label];
      int bb_w = cs.bb_x1 - cs.bb_x0 + 1;
      int bb_h = cs.bb_y1 - cs.bb_y0 + 1;
      float aspect = (bb_w > bb_h) ? (float)bb_w / bb_h : (float)bb_h / bb_w;
      float fill   = (float)cs.area / ((float)bb_w * (float)bb_h);
      float circ   = (cs.perimeter > 0) ? (4.0f * (float)M_PI * cs.area) / ((float)cs.perimeter * cs.perimeter) : 0.0f;
      if (circ > 1.0f) circ = 1.0f;
      const char *reason = "no candidates";
      if ((int)cs.area < min_area)        reason = "biggest too small";
      else if ((int)cs.area > max_area)   reason = "biggest too big";
      else if (aspect > max_aspect)       reason = "biggest elongated";
      else if (fill   < min_fill)         reason = "biggest scattered";
      else if (circ   < min_circularity)  reason = "biggest not round";
      Serial.printf("LOST   biggest: area=%6lu fill=%.2f circ=%.2f asp=%.2f  fps=%4.1f  %s\n",
                    (unsigned long)cs.area, fill, circ, aspect, current_fps, reason);
    } else {
      Serial.printf("LOST   no dark blobs  fps=%4.1f\n", current_fps);
    }
    prev_found = false;
  }

  if (led_off_at && millis() >= led_off_at) {
    digitalWrite(LED_GPIO_NUM, LOW);
    led_off_at = 0;
  }

  if (debug_ascii) dump_ascii(fb);

  esp_camera_fb_return(fb);
}

// ===========================================================================
// Serial command handling
// ===========================================================================
void handle_serial() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (inputBuf.length() > 0) { run_cmd(inputBuf); inputBuf = ""; }
    } else if (inputBuf.length() < 64) {
      inputBuf += c;
    }
  }
}

void run_cmd(String line) {
  line.trim();
  if (line.length() == 0) return;

  if (line == "save")  { cam_eeprom_save();  return; }
  if (line == "load") {
    if (cam_eeprom_load()) {
      Serial.println("OK  reloaded from EEPROM.");
      if (exposure_locked) apply_saved_exposure();
    } else {
      Serial.println("ERR no valid EEPROM data");
    }
    print_status();
    return;
  }
  if (line == "clear") { cam_eeprom_clear(); return; }

  char c = line.charAt(0);
  String arg = line.substring(1);

  if (c == '?') print_help();
  else if (c == 't') { int v = arg.toInt(); if (v>=0 && v<=255) { threshold=v; Serial.printf("OK  threshold=%d\n", threshold); } else Serial.println("ERR 0-255"); }
  else if (c == 'm') { int v = arg.toInt(); if (v>=1) { min_area=v; Serial.printf("OK  min_area=%d\n", min_area); } else Serial.println("ERR >=1"); }
  else if (c == 'M') { int v = arg.toInt(); if (v>=1) { max_area=v; Serial.printf("OK  max_area=%d\n", max_area); } else Serial.println("ERR >=1"); }
  else if (c == 'F') { float v = arg.toFloat(); if (v>1.0f) { FOCAL_LENGTH_PX=v; Serial.printf("OK  focal_px=%.2f\n", FOCAL_LENGTH_PX); } else Serial.println("ERR >1.0"); }
  else if (c == 'D') { float v = arg.toFloat(); if (v>0.1f) { MARKER_DIAMETER_MM=v; Serial.printf("OK  marker_D=%.2f mm\n", MARKER_DIAMETER_MM); } else Serial.println("ERR >0.1"); }
  else if (c == 'C') {
    float known_Z_mm = arg.toFloat();
    if (known_Z_mm < 50.0f || known_Z_mm > 5000.0f) Serial.println("ERR 50-5000 mm");
    else if (!have_last || last_area < min_area) Serial.println("ERR no recent FOUND. Hold marker, wait for FOUND, then C.");
    else {
      float d_px = 2.0f * sqrtf((float)last_area / (float)M_PI);
      float new_f = d_px * known_Z_mm / MARKER_DIAMETER_MM;
      Serial.printf("CAL  area=%ld d_px=%.2f Z=%.1f => focal=%.2f (was %.2f)\n",
                    last_area, d_px, known_Z_mm, new_f, FOCAL_LENGTH_PX);
      FOCAL_LENGTH_PX = new_f;
      Serial.println("OK  Type 'save' to persist.");
    }
  }
  else if (c == 'X') {
    sensor_t * s = esp_camera_sensor_get();
    if (s) {
      s->set_exposure_ctrl(s, 0); s->set_aec2(s, 0); s->set_gain_ctrl(s, 0);
      capture_current_exposure();
      exposure_locked = true;
      Serial.printf("OK  exposure LOCKED (aec=%d agc=%d). 'save' to persist.\n",
                    saved_aec_value, saved_agc_gain);
    }
  }
  else if (c == 'A') {
    sensor_t * s = esp_camera_sensor_get();
    if (s) {
      s->set_exposure_ctrl(s, 1); s->set_aec2(s, 1); s->set_gain_ctrl(s, 1);
      exposure_locked = false; saved_aec_value=-1; saved_agc_gain=-1;
      Serial.println("OK  auto exposure re-enabled.");
    }
  }
  else if (c == 's' && arg.length() > 1 && arg.charAt(0) == 'a') {
    String num = arg.substring(1); num.trim();
    float v = num.toFloat();
    if (v < 0.0f || v > 1.0f) Serial.println("ERR 0..1");
    else { min_circularity = v; Serial.printf("OK  min_circularity=%.2f\n", min_circularity); }
  }
  else if (c == 's' && arg.length() > 1 && arg.charAt(0) == 'b') {
    String num = arg.substring(1); num.trim();
    float v = num.toFloat();
    if (v < 1.0f || v > 10.0f) Serial.println("ERR 1.0..10.0");
    else { max_aspect = v; Serial.printf("OK  max_aspect=%.2f\n", max_aspect); }
  }
  else if (c == 's' && arg.length() > 1 && arg.charAt(0) == 'f') {
    String num = arg.substring(1); num.trim();
    float v = num.toFloat();
    if (v < 0.0f || v > 1.0f) Serial.println("ERR 0..1");
    else { min_fill = v; Serial.printf("OK  min_fill=%.2f\n", min_fill); }
  }
  else if (c == 'd') {
    debug_ascii = !debug_ascii;
    Serial.printf("OK  debug_ascii=%s\n", debug_ascii ? "ON" : "OFF");
  }
  else if (c == 'h') {
    // Brightness histogram
    camera_fb_t * fb = esp_camera_fb_get();
    if (!fb) { Serial.println("ERR capture failed"); return; }
    long bins[16] = {0};
    for (int i = 0; i < FRAME_PIXELS; i++) bins[fb->buf[i] >> 4]++;
    long total = (long)FRAME_PIXELS;
    long maxBin = 1;
    for (int i = 0; i < 16; i++) if (bins[i] > maxBin) maxBin = bins[i];
    Serial.println("--- brightness histogram ---");
    for (int i = 0; i < 16; i++) {
      int barLen = (int)(60L * bins[i] / maxBin);
      Serial.printf("  %3d-%3d: %6ld (%4.1f%%) %.*s\n",
                    i*16, i*16+15, bins[i], 100.0f * bins[i] / total,
                    barLen, "############################################################");
    }
    Serial.println("Look for a small spike at the dark end (top) - that's your marker.");
    Serial.println("Set threshold just ABOVE that spike. e.g. spike at 32-47 -> t50.");
    esp_camera_fb_return(fb);
  }
  else if (c == 'l') {
    // List top components in latest frame for tuning
    Serial.println("--- top 10 components by area ---");
    // Insertion sort top-10 (cheap for this)
    uint16_t top[10] = {0};
    uint32_t top_a[10] = {0};
    for (uint16_t L = 1; L < MAX_COMPONENTS; L++) {
      if (g_stats[L].area == 0) continue;
      uint32_t a = g_stats[L].area;
      for (int k = 0; k < 10; k++) {
        if (a > top_a[k]) {
          // shift down
          for (int j = 9; j > k; j--) { top_a[j] = top_a[j-1]; top[j] = top[j-1]; }
          top_a[k] = a; top[k] = L;
          break;
        }
      }
    }
    for (int k = 0; k < 10 && top[k] != 0; k++) {
      ComponentStats &cs = g_stats[top[k]];
      int bb_w = cs.bb_x1 - cs.bb_x0 + 1;
      int bb_h = cs.bb_y1 - cs.bb_y0 + 1;
      float aspect = (bb_w > bb_h) ? (float)bb_w/bb_h : (float)bb_h/bb_w;
      float fill = (float)cs.area / ((float)bb_w * bb_h);
      float circ = (cs.perimeter > 0) ? (4.0f * (float)M_PI * cs.area) / ((float)cs.perimeter * cs.perimeter) : 0;
      if (circ > 1.0f) circ = 1.0f;
      Serial.printf("  L%4d: area=%6lu bb=%dx%d at(%d,%d)  fill=%.2f circ=%.2f asp=%.2f\n",
                    top[k], (unsigned long)cs.area, bb_w, bb_h,
                    cs.bb_x0, cs.bb_y0, fill, circ, aspect);
    }
  }
  else if (c == 'i') print_status();
  else Serial.printf("ERR unknown cmd '%s'. Type ? for help.\n", line.c_str());
}

void print_status() {
  Serial.println();
  Serial.printf("threshold=%d  area=[%d..%d]\n", threshold, min_area, max_area);
  Serial.printf("focal_px=%.2f  marker_D=%.2fmm\n", FOCAL_LENGTH_PX, MARKER_DIAMETER_MM);
  Serial.printf("shape: min_circ=%.2f  max_aspect=%.2f  min_fill=%.2f\n",
                min_circularity, max_aspect, min_fill);
  Serial.printf("exposure=%s", exposure_locked ? "LOCKED" : "auto");
  if (exposure_locked) Serial.printf(" (aec=%d agc=%d)", saved_aec_value, saved_agc_gain);
  Serial.println();
  Serial.println();
}

void print_help() {
  Serial.println();
  Serial.println("DETECTION:");
  Serial.println("  t<n>       threshold 0-255");
  Serial.println("  m<n>       MIN blob area in px");
  Serial.println("  M<n>       MAX blob area in px");
  Serial.println("SHAPE FILTER (per connected component):");
  Serial.println("  sa<n>      min circularity 0..1 (default 0.40)");
  Serial.println("  sb<n>      max bbox aspect 1..10 (default 2.00)");
  Serial.println("  sf<n>      min fill ratio 0..1 (default 0.55)");
  Serial.println("CALIBRATION:");
  Serial.println("  F<n>       focal length px");
  Serial.println("  D<n>       marker diameter mm");
  Serial.println("  C<mm>      calibrate at known distance");
  Serial.println("EXPOSURE:");
  Serial.println("  X          LOCK / A          auto");
  Serial.println("EEPROM:");
  Serial.println("  save / load / clear");
  Serial.println("DIAGNOSTIC:");
  Serial.println("  h          BRIGHTNESS HISTOGRAM (pick threshold)");
  Serial.println("  l          LIST top 10 components in frame");
  Serial.println("  i          status     d   ASCII frame dump");
  Serial.println();
  print_status();
}

// ASCII frame dump (debug)
void dump_ascii(camera_fb_t * fb) {
  const int OUT_W = 80;
  const int OUT_H = 30;
  const int dx = FRAME_W / OUT_W;
  const int dy = FRAME_H / OUT_H;
  const char * shades = " .:-=+*#%@";
  Serial.println("---- frame ----");
  for (int oy = 0; oy < OUT_H; oy++) {
    char line[OUT_W + 1];
    for (int ox = 0; ox < OUT_W; ox++) {
      uint8_t p = fb->buf[(oy * dy) * FRAME_W + (ox * dx)];
      int idx = 9 - (p * 10 / 256);
      if (idx < 0) idx = 0;
      if (idx > 9) idx = 9;
      line[ox] = shades[idx];
    }
    line[OUT_W] = 0;
    Serial.println(line);
  }
  Serial.println("---------------");
}
