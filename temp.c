/*
 * VL53L4CX Multi-Object Speed Detector + Microphone Siren Detector for ESP32
 * ---------------------------------------------------------------------------
 * Detects approaching objects using a VL53L4CX ToF sensor AND detects sirens
 * using an I2S microphone + FFT analysis. Both run concurrently in the main
 * loop without blocking each other.
 *
 * Key design decisions:
 *   - Sensor poll is NON-BLOCKING: checked once per loop(), skipped if not ready.
 *   - fftloop() fills the audio buffer INCREMENTALLY (I2S_READ_LEN samples per
 *     call) so it never stalls. FFT only executes when all FFT_NUM_SAMPLES are
 *     ready, matching the sensor's 50 ms timing budget.
 *   - Because ticks_to_wait=0 is passed to i2s_read(), it returns immediately
 *     with whatever is in the DMA buffer and never blocks the loop.
 *
 * Wiring (ESP32):
 *   Sensor 1 XSHUT → GPIO 19     Sensor 2 XSHUT → GPIO 18
 *   Both SDA → GPIO 21            Both SCL → GPIO 22
 *   Mic BCLK → GPIO 27            Mic WS   → GPIO 26
 *   Mic SD   → GPIO 32
 *   Speed alert LED → GPIO 2 (built-in)
 */

/* ── Includes ────────────────────────────────────────────────────────────── */
#include <Wire.h>
#include <vl53l4cx_class.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <assert.h>
#include <stdlib.h>
#include <driver/i2s.h>
#include "FFT.h"

// WiFi library from lab — must come before WirelessCommunication.h
#include "sharedVariable.h"
#include "WirelessCommunication.h"

/* ── Pin Definitions ─────────────────────────────────────────────────────── */
#define SerialPort Serial
#define SDA_PIN 21
#define SCL_PIN 22
#define SENSOR_1 19            // give senor a ID just in case there is multiple sensors, it also is the Xshut pin
#define SENSOR_2 18
#define TRIGGER_PIN 33
#define LED_RED 4
#define LED_YELLOW 15
#define LED_GREEN 5

#ifndef LED_BUILTIN
  #define LED_BUILTIN 2
#endif
#define LedPin LED_BUILTIN

/* ── Sensor Setup ────────────────────────────────────────────────────────── */
// Components.
// use library function to set up VL53L4CX, component name is sensor_vl53l4cx_sat
#define DEV_I2C Wire          // define name for I2C bus
VL53L4CX sensor_1(&DEV_I2C, SENSOR_1);   //0x52

VL53L4CX sensor_2(&DEV_I2C, SENSOR_2);   // 0x54

/*--------------------WIFI--------------------------------*/
volatile shared_uint32 x;
volatile bool reset_flag = false;
static const uint32_t ALERT_COOLDOWN_MS  = 50;
static uint32_t       alert_cooldown_end = 0;

/* ── Tunable Constants: Distance Sensor ─────────────────────────────────── */
// Detection thresholds
static const float    DIST_THRESHOLD_MM    = 5000.0f;  // Only care about objects within 5 m
static const float    SPEED_THRESHOLD_MS   = 0.5f;     // Alert threshold: 0.6 m/s
static const float    MATCH_GATE_MM        = 400.0f;   // Max jump allowed between frames (mm)

// Debounce / hit window
static const int      REQUIRED_HITS        = 2;        // Consecutive fast frames needed
static const uint32_t HIT_WINDOW_MS        = 1000;      // All hits must land within 600 ms

// Moving average
static const int      MA_WINDOW_SIZE          = 4;        // Window size for smoothing

// Tracker patience
static const int      MAX_MISSED_FRAMES    = 4;        // Frames to wait before dropping an object
static const uint32_t TRACKER_TIMEOUT_MS = 500; // Wipe if unseen for 500 ms (~10 frames)

// LED hold
static const uint32_t LED_HOLD_MS          = 200;     // Keep LED on for 0.2 s after trigger

// Sensor timing budget (µs) — 50 ms = ~20 Hz
static const uint32_t TIMING_BUDGET_US     = 50000;

// Maximum number of objects we track simultaneously (sensor hardware limit is 4)
static const int      MAX_OBJECTS          = 4;

/* ── Tracker State ───────────────────────────────────────────────────────── */
struct ObjectTracker {
  // Moving average
  float    history[MA_WINDOW_SIZE]; // Circular buffer of raw distances
  float    dist_sum;             // Running sum for the average
  int      idx;                  // Next write position in the circular buffer
  int      count;                // How many valid samples are in the buffer
  float    smoothed;             // Current smoothed distance
  float    prev_smoothed;        // Previous smoothed distance (for speed)

  // Debounce
  int      hit_counter;
  uint32_t first_hit_time;

  // Patience
  int      missed_frames;
  uint32_t last_seen_ms;
};

// Independent tracker arrays — one per sensor FOV
static ObjectTracker trackers_1[MAX_OBJECTS];
static ObjectTracker trackers_2[MAX_OBJECTS];

/* ── Per-Sensor Timing ───────────────────────────────────────────────────── */
static uint32_t prev_time_us_1 = 0, curr_time_us_1 = 0;
static uint32_t prev_time_us_2 = 0, curr_time_us_2 = 0;

/* ── Per-Sensor First-Frame Flag ─────────────────────────────────────────── */
static bool first_frame_1 = true;
static bool first_frame_2 = true;

/* ── LED Hold ────────────────────────────────────────────────────────────── */
static uint32_t led_off_time_ms = 0;

/* ── Misc ────────────────────────────────────────────────────────────────── */
static int sensor_status = 0;

/* ── Distance LED state (shared across both sensors) ─────────────────────── */
// Updated each loop from whichever sensor gives the closer valid reading
static bool  led_obj_valid = false;
static float led_obj_dist  = 99999.0f;

/* -------------------------------------------------- Microphone --------------------------------------------- */

#define FFT_NUM_SAMPLES   512   // Must match FFT.h NUM_SAMPLES
// SAMPLING_FREQ is used directly from FFT.h

#define POWER_RATIO_THRESHOLD            0.3
#define TARGET_TO_LOWER_RATIO_THRESHOLD 15.00

#define I2S_READ_LEN                128   // Samples read from DMA per fftloop() call
#define REQUIRED_CONSECUTIVE_FRAMES   5   // Method 1: consecutive detections to confirm
#define REQUIRED_CONSECUTIVE_FRAMES_2 10  // Method 2: consecutive detections to confirm

// Method 2 peak-detection parameters
#define MY_MIN_MAG         3000000   // Minimum FFT magnitude for a peak to count
#define MY_PEAK_FACTOR           2   // Peak must be >= this multiple of the band average
#define MY_MIN_PEAK_SPACING  ((int)(300.0 * FFT_NUM_SAMPLES / SAMPLING_FREQ))  // ~300 Hz in bins

// Method 1: Target Band (900-1600 Hz)
#define M1_TARGET_LO  29
#define M1_TARGET_HI  51
// Method 1: Lower Band (100-899 Hz)
#define M1_LOWER_LO   3
#define M1_LOWER_HI   28

// Method 2: Siren frequency range (600-3000 Hz)
#define SIREN_FFT_BIN_LO  19
#define SIREN_FFT_BIN_HI  96

/* ── Microphone / FFT Variables ──────────────────────────────────────────── */
i2s_port_t I2S_PORT = I2S_NUM_0;

int PIN_I2S_BCLK = 27;
int PIN_I2S_WS   = 26;
int PIN_I2S_SD   = 32;

int32_t  sampleBuffer[I2S_READ_LEN];
double   vReal[FFT_NUM_SAMPLES];
double   vImag[FFT_NUM_SAMPLES];

// rawAudio keeps a safe copy because FFT.compute() overwrites vReal in-place
static double rawAudio[FFT_NUM_SAMPLES];

// Detection results — read by WiFi task or other modules
bool sirenDetected  = false;   // Method 1 result
bool sirenDetected2 = false;   // Method 2 result

ArduinoFFT<double> FFT = ArduinoFFT<double>(vReal, vImag, FFT_NUM_SAMPLES, SAMPLING_FREQ);

// Internal state: how many samples have been collected in the current window
static int fft_sample_count = 0;

// Consecutive-frame counters for both detection methods
static int method1_consec = 0;
static int method2_consec = 0;
/* ── Forward Declarations ────────────────────────────────────────────────── */
void reset_tracker_slot(ObjectTracker& t);
void push_sample_slot(ObjectTracker& t, float raw_d);
bool process_objects(int object_count, float raw_dist[], int raw_statuses[],
                     ObjectTracker trackers[], uint32_t curr_us, uint32_t prev_us,
                     const char* label);
void seed_first_frame(int capped_count, float raw_dist[], int raw_status[],
                      ObjectTracker trackers[]);
void fire_combined_alert(bool dist_alert);
void update_led(bool object_visible, float closest_dist);
void fftsetup();
void fftloop();

/* Setup ---------------------------------------------------------------------*/

void setup()
{
  // Led.
  pinMode(LedPin, OUTPUT);
  pinMode(LED_RED, OUTPUT);
  pinMode(LED_YELLOW, OUTPUT);
  pinMode(LED_GREEN, OUTPUT);

  // Initialize serial for output.
  SerialPort.begin(115200);

  while (!Serial);

  INIT_SHARED_VARIABLE(x, 0);   // Start trigger counter at 0
  init_wifi_task();

  SerialPort.println("Starting...");

  // Initialize I2C bus.
  DEV_I2C.begin();

  // Initialise all tracker slots for both sensors
  for (int i = 0; i < MAX_OBJECTS; i++) {
    reset_tracker_slot(trackers_1[i]);
    reset_tracker_slot(trackers_2[i]);
  }

  // ── Sensor boot sequence ──────────────────────────────────────────────────
  // Both XSHUT pins must be controlled as outputs.
  // Hold BOTH sensors in reset first, then boot one at a time to assign
  // each a unique I2C address before the other is allowed on the bus.
  pinMode(SENSOR_1, OUTPUT); digitalWrite(SENSOR_1, LOW);
  pinMode(SENSOR_2, OUTPUT); digitalWrite(SENSOR_2, LOW);
  delay(10);

  sensor_2.begin();                        // sets up the I2C reference in the library
  int status = sensor_2.InitSensor(0x54); // resets S2 via XSHUT_2, then assigns 0x54
  if (status != 0) {
    Serial.printf("[ERROR] Sensor 2 InitSensor failed, status=%d\n", status);
    while (true) { delay(1000); }
  }
  Serial.println("[OK] Sensor 2 initialised at 0x54");

  sensor_1.begin();
  status = sensor_1.InitSensor(0x52);
  if (status != 0) {
    Serial.printf("[ERROR] Sensor 1 InitSensor failed, status=%d\n", status);
    while (true) { delay(1000); }
  }
  Serial.println("[OK] Sensor 1 initialised at 0x52");

  // ── Set timing budget for both ────────────────────────────────────────────
  status = sensor_1.VL53L4CX_SetMeasurementTimingBudgetMicroSeconds(TIMING_BUDGET_US);
  if (status != 0) {
    Serial.printf("[ERROR] Sensor 1 SetTimingBudget failed, status=%d\n", status);
    while (true) { delay(1000); }
  }
  status = sensor_2.VL53L4CX_SetMeasurementTimingBudgetMicroSeconds(TIMING_BUDGET_US);
  if (status != 0) {
    Serial.printf("[ERROR] Sensor 2 SetTimingBudget failed, status=%d\n", status);
    while (true) { delay(1000); }
  }
  // ── Start both measurements at (roughly) the same time ───────────────────
  // From this point both sensors are measuring in parallel internally.
  // I2C is only used briefly to read results — the 50 ms laser-on phase
  // runs inside the sensor chip with no bus activity needed.
  status = sensor_1.VL53L4CX_StartMeasurement();
  if (status != 0) {
    Serial.printf("[ERROR] Sensor 1 StartMeasurement failed, status=%d\n", status);
    while (true) { delay(1000); }
  }
  status = sensor_2.VL53L4CX_StartMeasurement();
  if (status != 0) {
    Serial.printf("[ERROR] Sensor 2 StartMeasurement failed, status=%d\n", status);
    while (true) { delay(1000); }
  }

  Serial.println("[OK] Both sensors measuring. Monitoring...\n");

  // set up for the microphone
  fftsetup();
}

void loop()
{
  // ── Handle reboot request from server ──
  if (reset_flag) {
    reset_flag = false;
    Serial.println("[WiFi] Reboot requested by server — restarting...");
    delay(100);
    ESP.restart();
  }

  bool dist_alert = false;

  // Reset distance LED candidates each loop — will be updated below
  led_obj_valid = false;
  led_obj_dist  = 99999.0f;

  // ── SENSOR 1 (non-blocking poll) ──────────────────────────────────────────
  {
    uint8_t ready = 0;
    sensor_status = sensor_1.VL53L4CX_GetMeasurementDataReady(&ready);
    if (sensor_status != 0)
      Serial.println("[WARN] Sensor 1 GetMeasurementDataReady failed");

    if (ready) {
      prev_time_us_1 = curr_time_us_1;
      curr_time_us_1 = micros();

      VL53L4CX_MultiRangingData_t data;
      sensor_status = sensor_1.VL53L4CX_GetMultiRangingData(&data);
      if (sensor_status != 0) {
        Serial.printf("[ERROR] Sensor 1 GetMultiRangingData failed, status=%d\n", sensor_status);
      } else {
        int   cnt  = min((int)data.NumberOfObjectsFound, MAX_OBJECTS);
        float dist [MAX_OBJECTS] = {};
        int   stat [MAX_OBJECTS] = {};

        // print
        // Serial.printf("[S1] Frame %u | Objects=%d\n", data.StreamCount, cnt);
        for (int j = 0; j < cnt; j++) {
          stat[j] = data.RangeData[j].RangeStatus;
          dist[j] = data.RangeData[j].RangeMilliMeter;
          // Print
          //Serial.printf("  [S1:%d] status=%d  dist=%.0f mm  signal=%.2f Mcps\n",
          //  j, stat[j], dist[j],
          //  (float)data.RangeData[j].SignalRateRtnMegaCps / 65536.0f);
        }

        if (!first_frame_1 && cnt > 0) {
          bool alert = process_objects(cnt, dist, stat, trackers_1,
                                       curr_time_us_1, prev_time_us_1, "S1");
          if (alert) dist_alert = true;
        }
        if (first_frame_1) {
          seed_first_frame(cnt, dist, stat, trackers_1);
          first_frame_1 = false;
        }

        // Distance LED candidate from sensor 1 object[0]
        if (cnt > 0 && stat[0] == 0) {
          led_obj_valid = true;
          led_obj_dist  = dist[0]; // Will be compared with sensor 2 below
        }
      }
      sensor_1.VL53L4CX_ClearInterruptAndStartMeasurement();
    }
  }

  // ── SENSOR 2 (non-blocking poll) ──────────────────────────────────────────
  {
    uint8_t ready = 0;
    sensor_status = sensor_2.VL53L4CX_GetMeasurementDataReady(&ready);
    if (sensor_status != 0)
      Serial.println("[WARN] Sensor 2 GetMeasurementDataReady failed");

    if (ready) {
      prev_time_us_2 = curr_time_us_2;
      curr_time_us_2 = micros();

      VL53L4CX_MultiRangingData_t data;
      sensor_status = sensor_2.VL53L4CX_GetMultiRangingData(&data);
      if (sensor_status != 0) {
        Serial.printf("[ERROR] Sensor 2 GetMultiRangingData failed, status=%d\n", sensor_status);
      } else {
        int   cnt  = min((int)data.NumberOfObjectsFound, MAX_OBJECTS);
        float dist [MAX_OBJECTS] = {};
        int   stat [MAX_OBJECTS] = {};

        // print
        // Serial.printf("[S2] Frame %u | Objects=%d\n", data.StreamCount, cnt);
        for (int j = 0; j < cnt; j++) {
          stat[j] = data.RangeData[j].RangeStatus;
          dist[j] = data.RangeData[j].RangeMilliMeter;
          // print
          // Serial.printf("  [S2:%d] status=%d  dist=%.0f mm  signal=%.2f Mcps\n",
          //  j, stat[j], dist[j],
          //  (float)data.RangeData[j].SignalRateRtnMegaCps / 65536.0f);
        }

        if (!first_frame_2 && cnt > 0) {
          bool alert = process_objects(cnt, dist, stat, trackers_2,
                                       curr_time_us_2, prev_time_us_2, "S2");
          if (alert) dist_alert = true;
        }
        if (first_frame_2) {
          seed_first_frame(cnt, dist, stat, trackers_2);
          first_frame_2 = false;
        }

        // Distance LED — keep the closer object[0] between the two sensors
        if (cnt > 0 && stat[0] == 0) {
          if (!led_obj_valid || dist[0] < led_obj_dist) {
            led_obj_valid = true;
            led_obj_dist  = dist[0];
          }
        }
      }
      sensor_2.VL53L4CX_ClearInterruptAndStartMeasurement();
    }
  }

  // ── Distance LED update (closest reading from either sensor) ──────────────
  update_led(led_obj_valid, led_obj_dist);

  // ── Speed alert (either sensor) ───────────────────────────────────────────
  fire_combined_alert(dist_alert);
  // microphone processing
  fftloop();

  // ── Combined siren alert (checked every loop iteration after FFT updates flags) ──
  if (sirenDetected || sirenDetected2) {
    fire_combined_alert(false);   // dist_alert=false; siren flags carry the trigger
  }
}

/* ═══════════════════════════════════════════════════════════════════════════
   HELPERS — Distance Tracker
   ═══════════════════════════════════════════════════════════════════════════ */

   // Takes a direct reference to one tracker slot — works for either sensor's array
void reset_tracker_slot(ObjectTracker& t)
{
  memset(t.history, 0, sizeof(t.history));
  t.dist_sum       = 0.0f;
  t.idx            = 0;
  t.count          = 0;
  t.smoothed       = 0.0f;
  t.prev_smoothed  = 0.0f;
  t.hit_counter    = 0;
  t.first_hit_time = 0;
  t.missed_frames  = 0;
  t.last_seen_ms   = 0;
}

void push_sample_slot(ObjectTracker& t, float raw_d)
{
  if (t.count == MA_WINDOW_SIZE) {
    t.dist_sum -= t.history[t.idx];
  } else {
    t.count++;
  }
  t.history[t.idx] = raw_d;
  t.dist_sum      += raw_d;
  t.idx            = (t.idx + 1) % MA_WINDOW_SIZE;
  t.prev_smoothed  = t.smoothed;
  t.smoothed       = t.dist_sum / (float)t.count;
}

// Seeds the very first frame for a given sensor's tracker array
void seed_first_frame(int capped_count, float raw_dist[], int raw_status[],
                      ObjectTracker trackers[])
{
  for (int j = 0; j < capped_count; j++) {
    if (raw_status[j] == 0) {
      reset_tracker_slot(trackers[j]);
      trackers[j].history[0]    = raw_dist[j];
      trackers[j].dist_sum      = raw_dist[j];
      trackers[j].count         = 1;
      trackers[j].idx           = 1;
      trackers[j].smoothed      = raw_dist[j];
      trackers[j].prev_smoothed = raw_dist[j];
      trackers[j].last_seen_ms  = millis();
    }
  }
}

/* ══════════════════════════════════════════════════════════════════════════
   CORE TRACKING + SPEED LOGIC
   Works identically for both sensors — caller passes the correct tracker
   array and timing values. Returns true if a speed alert fired.
   ══════════════════════════════════════════════════════════════════════════ */
bool process_objects(int object_count, float raw_dist[], int raw_statuses[],
                     ObjectTracker trackers[], uint32_t curr_us, uint32_t prev_us,
                     const char* label)
{
  float delta_t = (float)(curr_us - prev_us) / 1e6f;
  Serial.printf(" time change: %f\n", delta_t);
  if (delta_t <= 0.0f || delta_t > 1.0f) {
    Serial.printf("[WARN] %s Implausible delta_t, skipping frame.\n", label);
    return false;
  }

  bool trigger_alert = false;
  bool reading_claimed[MAX_OBJECTS] = {};

  // ── PHASE 1: Match existing trackers ─────────────────────────────────────
  for (int t = 0; t < MAX_OBJECTS; t++) {
    if (trackers[t].count == 0) continue;

    float best_diff = MATCH_GATE_MM;
    int   best_r    = -1;

    for (int r = 0; r < object_count; r++) {
      if (reading_claimed[r] || raw_statuses[r] != 0) continue;
      float diff = fabsf(trackers[t].smoothed - raw_dist[r]);
      if (diff < best_diff) { best_diff = diff; best_r = r; }
    }

    if (best_r != -1) {
      reading_claimed[best_r]   = true;
      trackers[t].missed_frames = 0;
      trackers[t].last_seen_ms  = millis();

      push_sample_slot(trackers[t], raw_dist[best_r]);

      float speed = 0.0f;
      if (trackers[t].prev_smoothed > 0.0f && trackers[t].count > 1)
        speed = (trackers[t].prev_smoothed - trackers[t].smoothed) / 1000.0f / delta_t;

      // print
      // Serial.printf("  [%s] Tracker[%d] smoothed=%.0f mm  speed=%.3f m/s\n",
      //              label, t, trackers[t].smoothed, speed);

      if (speed > SPEED_THRESHOLD_MS && trackers[t].smoothed < DIST_THRESHOLD_MM) {
        uint32_t now = millis();
        if (trackers[t].hit_counter == 0) {
          trackers[t].first_hit_time = now;
          trackers[t].hit_counter    = 1;
        } else if (now - trackers[t].first_hit_time <= HIT_WINDOW_MS) {
          trackers[t].hit_counter++;
          if (trackers[t].hit_counter >= REQUIRED_HITS) {
            // print
            // Serial.printf("  *** ALERT [%s]: Tracker[%d] at %.3f m/s ***\n", label, t, speed);
            trigger_alert           = true;
            trackers[t].hit_counter = 0;
          }
        } else {
          trackers[t].first_hit_time = now;
          trackers[t].hit_counter    = 1;
        }
      } else {
        trackers[t].hit_counter = 0;
      }

    } else {
      trackers[t].missed_frames++;
    }
  }

  // ── PHASE 2: Newcomers ────────────────────────────────────────────────────
  for (int r = 0; r < object_count; r++) {
    if (reading_claimed[r] || raw_statuses[r] != 0) continue;
    for (int t = 0; t < MAX_OBJECTS; t++) {
      if (trackers[t].count == 0) {
        reset_tracker_slot(trackers[t]);
        trackers[t].history[0]    = raw_dist[r];
        trackers[t].dist_sum      = raw_dist[r];
        trackers[t].count         = 1;
        trackers[t].idx           = 1;
        trackers[t].smoothed      = raw_dist[r];
        trackers[t].prev_smoothed = raw_dist[r];
        trackers[t].last_seen_ms  = millis();
        reading_claimed[r]        = true;
        // print
        // Serial.printf("  [%s] New object → Tracker[%d] at %.0f mm\n", label, t, raw_dist[r]);
        break;
      }
    }
  }

  // ── PHASE 3: Reaper ───────────────────────────────────────────────────────
  for (int t = 0; t < MAX_OBJECTS; t++) {
    if (trackers[t].count == 0) continue;
    bool timed_out       = trackers[t].last_seen_ms > 0 &&
                           millis() - trackers[t].last_seen_ms > TRACKER_TIMEOUT_MS;
    bool missed_too_long = trackers[t].missed_frames >= MAX_MISSED_FRAMES;
    if (timed_out || missed_too_long) {
      // print
      // Serial.printf("  [%s] Tracker[%d] expired — %s\n", label, t,
      //              timed_out ? "timeout" : "missed frames");
      reset_tracker_slot(trackers[t]);
    }
  }

  return trigger_alert;
}

/* ── Combined alert: any source (distance sensor 1, sensor 2, or siren) ── */
void fire_combined_alert(bool dist_alert)
{
  bool any_alert = dist_alert || sirenDetected || sirenDetected2;
  if (!any_alert) return;

  uint32_t now = millis();
  led_off_time_ms = now + LED_HOLD_MS;
  digitalWrite(LedPin, HIGH);

  if (now >= alert_cooldown_end) {
    alert_cooldown_end = now + ALERT_COOLDOWN_MS;
    LOCK_SHARED_VARIABLE(x);
    x.value++;
    UNLOCK_SHARED_VARIABLE(x);
  }

  // print
  // Serial.printf("[ALERT] dist=%d  siren1=%d  siren2=%d  → LED HIGH + WiFi sent\n",
  //              (int)dist_alert, (int)sirenDetected, (int)sirenDetected2);
}

/* ── Distance zone LEDs ──────────────────────────────────────────────────── */
void update_led(bool object_visible, float closest_dist)
{
  if (millis() < led_off_time_ms) {
    digitalWrite(LedPin, HIGH);
  } else {
    digitalWrite(LedPin, LOW);
  }

  if (object_visible) {
    if (closest_dist < 1000.0f) {
      digitalWrite(LED_GREEN,  LOW); digitalWrite(LED_YELLOW, LOW); digitalWrite(LED_RED,    HIGH);
    } else if (closest_dist < 2500.0f) {
      digitalWrite(LED_GREEN,  LOW); digitalWrite(LED_RED,    LOW); digitalWrite(LED_YELLOW, HIGH);
    } else {
      digitalWrite(LED_RED,    LOW); digitalWrite(LED_YELLOW, LOW); digitalWrite(LED_GREEN,  HIGH);
    }
  } else {
    digitalWrite(LED_RED,    LOW); digitalWrite(LED_YELLOW, LOW); digitalWrite(LED_GREEN,  HIGH);
  }
}


/* ══════════════════════════════════════════════════════════════════════════
   HELPERS - Microphone
   ══════════════════════════════════════════════════════════════════════════ */

// set up for microphone
void fftsetup() {
  Serial.begin(115200);
  delay(1000);

  const i2s_config_t i2s_config = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
    .sample_rate = SAMPLING_FREQ,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT,
    .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
    .communication_format = I2S_COMM_FORMAT_I2S,
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
    .dma_buf_count = 4,
    .dma_buf_len = I2S_READ_LEN,
    .use_apll = false,
    .tx_desc_auto_clear = false,
    .fixed_mclk = 0
  };

  const i2s_pin_config_t pin_config = {
    .bck_io_num = PIN_I2S_BCLK,
    .ws_io_num = PIN_I2S_WS,
    .data_out_num = I2S_PIN_NO_CHANGE,
    .data_in_num = PIN_I2S_SD
  };

  i2s_driver_install(I2S_PORT, &i2s_config, 0, NULL);
  i2s_set_pin(I2S_PORT, &pin_config);
  i2s_zero_dma_buffer(I2S_PORT);
}

void fftloop()
{
  // ── Pull one DMA chunk (non-blocking) ──
  size_t bytes_read = 0;
  i2s_read(I2S_PORT,
           sampleBuffer,
           sizeof(sampleBuffer),
           &bytes_read,
           0);                    // ticks_to_wait = 0  →  non-blocking

  int samples_read = (int)(bytes_read / sizeof(int32_t));

  // ── Append to accumulation buffer ──
  for (int i = 0; i < samples_read && fft_sample_count < FFT_NUM_SAMPLES; i++) {
    // INMP441 outputs 24-bit data left-aligned in a 32-bit word; shift right to normalise.
    rawAudio[fft_sample_count++] = (double)(sampleBuffer[i] >> 8);
  }

  // Buffer not yet full — return early, no computation this iteration
  if (fft_sample_count < FFT_NUM_SAMPLES) return;

  // ── Full window ready: reset counter for next collection cycle ──
  fft_sample_count = 0;

  // ── Copy rawAudio → FFT working arrays ──
  // (rawAudio is kept intact; FFT.compute() will overwrite vReal in-place)
  for (int i = 0; i < FFT_NUM_SAMPLES; i++) {
    vReal[i] = rawAudio[i];
    vImag[i] = 0.0;
  }

  // ── Compute FFT ──
  FFT.windowing(FFT_WIN_TYP_HAMMING, FFT_FORWARD);
  FFT.compute(FFT_FORWARD);
  FFT.complexToMagnitude();   // Results land in vReal[0 .. FFT_NUM_SAMPLES/2]

  // ── Build 16 coarse bins ──
  // Each bin spans (SAMPLING_FREQ / 32) = 256 Hz.
  // In FFT buckets: (FFT_NUM_SAMPLES/2) / 16 = 16 buckets per bin.
  static const int BIN_WIDTH = (FFT_NUM_SAMPLES / 2) / 16;   // = 16
  int bins[16] = {};
  for (int b = 0; b < 16; b++) {
    double sum = 0.0;
    for (int k = 0; k < BIN_WIDTH; k++) {
      sum += vReal[b * BIN_WIDTH + k];
    }
    bins[b] = (int)sum;
  }

  /* ── METHOD 1: Power-ratio detection ──────────────────────────────────── */
  double target_energy = 0.0, lower_energy = 0.0, total_energy = 0.0;

  for (int i = 1; i < (FFT_NUM_SAMPLES / 2); i++) {
    double power = vReal[i] * vReal[i];
    total_energy += power;

    if (i >= M1_TARGET_LO && i <= M1_TARGET_HI) {
      target_energy += power;
    } else if (i >= M1_LOWER_LO && i <= M1_LOWER_HI) {
      lower_energy += power;
    }
  }

  double ratio_total = (total_energy > 0.0) ? (target_energy / total_energy) : 0.0;
  double ratio_lower = (lower_energy > 0.0) ? (target_energy / lower_energy) : 0.0;

  if (ratio_total > POWER_RATIO_THRESHOLD && ratio_lower > TARGET_TO_LOWER_RATIO_THRESHOLD) {
    method1_consec++;
  } else {
    method1_consec = 0;
  }
  sirenDetected = (method1_consec >= REQUIRED_CONSECUTIVE_FRAMES);

  /* ── METHOD 2: Prominent-peak detection ───────────────────────────────── */
  // Compute average magnitude in the siren frequency range for the prominence check
  double band_sum = 0.0;
  int    band_cnt = 0;
  for (int k = SIREN_FFT_BIN_LO; k <= SIREN_FFT_BIN_HI; k++) {
    band_sum += vReal[k];
    band_cnt++;
  }
  double band_avg = (band_cnt > 0) ? (band_sum / band_cnt) : 1.0;

  int peak_count    = 0;
  int last_peak_bin = SIREN_FFT_BIN_LO - MY_MIN_PEAK_SPACING - 1;

  for (int k = SIREN_FFT_BIN_LO + 1; k < SIREN_FFT_BIN_HI; k++) {
    bool is_local_max = (vReal[k] > vReal[k - 1]) && (vReal[k] > vReal[k + 1]);
    bool above_min    = (vReal[k] > (double)MY_MIN_MAG);
    bool prominent    = (vReal[k] > (double)MY_PEAK_FACTOR * band_avg);
    bool well_spaced  = ((k - last_peak_bin) >= MY_MIN_PEAK_SPACING);

    if (is_local_max && above_min && prominent && well_spaced) {
      peak_count++;
      last_peak_bin = k;
    }
  }

  if (peak_count >= 2) {
    method2_consec++;
  } else {
    method2_consec = 0;
  }
  sirenDetected2 = (method2_consec >= REQUIRED_CONSECUTIVE_FRAMES_2);

  // ── Debug output (uncomment printBins for full table) ──
  // printBins(bins);
  Serial.printf("[FFT] ratio_total=%.2f ratio_lower=%.2f consec1=%d | peaks=%d consec2=%d | siren1=%d siren2=%d\n",
                ratio_total, ratio_lower, method1_consec, peak_count, method2_consec,
                (int)sirenDetected, (int)sirenDetected2);
}
