#include <Arduino.h>

#include <time.h>
#include <ctype.h>
#include <stddef.h>

#include <stdarg.h>

#include <esp_sleep.h>
#include <esp_system.h>

#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>

#include "secrets.h"

#ifndef MQTT_SUBSCRIBE_TOPIC
#error "MQTT_SUBSCRIBE_TOPIC must be defined in include/secrets.h"
#endif
#include "HT_lCMEN2R13EFC1.h"

#ifndef FW_VERSION
#define FW_VERSION "dev"
#endif

#define AI_LOG_SCHEMA_VERSION 1

#ifndef DEEPSLEEP_ENABLE
#define DEEPSLEEP_ENABLE 0
#endif

#ifndef DEEPSLEEP_LEARN_ENABLE
#define DEEPSLEEP_LEARN_ENABLE 1
#endif

#ifndef DEEPSLEEP_MIN_SEC
#define DEEPSLEEP_MIN_SEC 60
#endif

#ifndef DEEPSLEEP_MAX_SEC
#define DEEPSLEEP_MAX_SEC (30 * 60)
#endif

#ifndef DEEPSLEEP_LISTEN_WINDOW_SEC
#define DEEPSLEEP_LISTEN_WINDOW_SEC 90
#endif

#ifndef DEEPSLEEP_FALLBACK_SEC
#define DEEPSLEEP_FALLBACK_SEC (5 * 60)
#endif

#ifndef DEEPSLEEP_CONFIDENCE_MIN
#define DEEPSLEEP_CONFIDENCE_MIN 45
#endif

#ifndef DEEPSLEEP_MIN_INTERVAL_SAMPLES
#define DEEPSLEEP_MIN_INTERVAL_SAMPLES 2
#endif

#ifndef DEEPSLEEP_LEARN_MIN_INTERVAL_SEC
#define DEEPSLEEP_LEARN_MIN_INTERVAL_SEC 120
#endif

#ifndef DEEPSLEEP_RETAIN_MODE_TOPIC
#define DEEPSLEEP_RETAIN_MODE_TOPIC ""
#endif

#ifndef DEEPSLEEP_RETAIN_POLL_SEC
#define DEEPSLEEP_RETAIN_POLL_SEC (15 * 60)
#endif

#ifndef DEEPSLEEP_MIN_AWAKE_SEC
#define DEEPSLEEP_MIN_AWAKE_SEC 30
#endif

#ifndef DEEPSLEEP_MAX_EMPTY_SLEEP_CYCLES
#define DEEPSLEEP_MAX_EMPTY_SLEEP_CYCLES 12
#endif

#define CADENCE_MODEL_VERSION 1
#define CADENCE_RING_SIZE 8
#define CADENCE_MAX_INTERVAL_SEC (6 * 60 * 60)
#define CADENCE_SAFETY_FLOOR_SEC 45
#define CADENCE_RECONNECT_BUDGET_SEC 20
#define CADENCE_MAD_MULTIPLIER 3
#define CADENCE_CONFIDENCE_INC_HIT 8
#define CADENCE_CONFIDENCE_DEC_MISS 15
#define CADENCE_MAX_CONSECUTIVE_MISSES 3

enum class SleepMode : uint8_t {
  Adaptive = 0,
  RetainPoll15m = 1,
};

// Heltec Wireless Paper v1.1 e-ink (2.13" BW) pin mapping from Heltec example:
// HT_ICMEN2R13EFC1 display(rst, dc, cs, busy, sck, mosi, miso, frequency);
static HT_ICMEN2R13EFC1 display(6, 5, 4, 7, 3, 2, -1, 6000000);

static WiFiClient wifiClient;
static PubSubClient mqttClient(wifiClient);

static float lastTemperatureC = NAN;
static float lastPressureHpa = NAN;
static float lastHumidityPct = NAN;
static unsigned long lastMqttMessageMs = 0;
static unsigned long lastTempUpdateMs = 0;

static int lastWifiRssiDbm = 0;
static bool lastWifiRssiValid = false;

static uint32_t lastMqttTimestampEpoch = 0;
static bool lastMqttTimestampValid = false;

static float lastBatteryVoltageV = NAN;
static unsigned long lastBatteryReadMs = 0;

static unsigned long lastDisplayRefreshMs = 0;
static bool displayDirty = false;

static bool displayInitialized = false;

static volatile bool wifiDisconnectedEvent = false;
static unsigned long logLineSeq = 0;
static unsigned long wakeStartMs = 0;
static bool mappedUpdateSeenThisWake = false;
static bool sleepStateHandled = false;
static bool trainingWaitLogged = false;
static bool mqttConnectedThisWake = false;
static bool retainFlagSeenThisWake = false;
static bool sleepInhibitLogged = false;

struct CadenceModel {
  uint32_t lastMappedTs = 0;
  uint32_t mappedIntervals[CADENCE_RING_SIZE] = {0};
  uint8_t mappedCount = 0;
  uint8_t mappedHead = 0;

  uint32_t medianIntervalSec = 0;
  uint32_t madSec = 0;

  uint8_t confidence = 0;
  uint8_t missedWindows = 0;

  uint32_t lastPlanSleepSec = 0;
  uint8_t lastPlanAdaptive = 0;

  uint8_t mode = (uint8_t)SleepMode::Adaptive;
  uint8_t modeSourceBroker = 0;
  uint16_t emptySleepCycles = 0;
  uint8_t sleepInhibit = 0;

  uint8_t version = CADENCE_MODEL_VERSION;
  uint32_t checksum = 0;
};

RTC_DATA_ATTR static CadenceModel cadenceModel;

static void VextON();
static void logAi(const char *eventName, const char *fmt, ...);
static void cadenceSave();

static const char *sleepModeToStr(SleepMode mode) {
  return (mode == SleepMode::RetainPoll15m) ? "retain_poll_15m" : "adaptive";
}

static SleepMode cadenceMode() {
  return (cadenceModel.mode == (uint8_t)SleepMode::RetainPoll15m) ? SleepMode::RetainPoll15m : SleepMode::Adaptive;
}

static bool hasRetainModeTopic() {
  return strlen(DEEPSLEEP_RETAIN_MODE_TOPIC) > 0;
}

static bool isRetainModeTopic(const char *topic) {
  if (!hasRetainModeTopic() || topic == nullptr) return false;
  return strcmp(topic, DEEPSLEEP_RETAIN_MODE_TOPIC) == 0;
}

static void cadenceResetLearningData() {
  cadenceModel.lastMappedTs = 0;
  cadenceModel.mappedCount = 0;
  cadenceModel.mappedHead = 0;
  memset(cadenceModel.mappedIntervals, 0, sizeof(cadenceModel.mappedIntervals));
  cadenceModel.medianIntervalSec = 0;
  cadenceModel.madSec = 0;
  cadenceModel.confidence = 0;
  cadenceModel.missedWindows = 0;
  trainingWaitLogged = false;
}

static void setSleepMode(SleepMode newMode, bool sourceBroker, bool resetLearning, const char *reason) {
  const SleepMode oldMode = cadenceMode();
  const bool oldSourceBroker = cadenceModel.modeSourceBroker != 0;

  cadenceModel.mode = (uint8_t)newMode;
  cadenceModel.modeSourceBroker = sourceBroker ? 1 : 0;

  if (resetLearning) {
    cadenceResetLearningData();
  }

  cadenceSave();

  if (oldMode != newMode || oldSourceBroker != sourceBroker || resetLearning) {
    logAi("mode_switch", "from=%s to=%s source=%s action=%s reason=%s",
          sleepModeToStr(oldMode), sleepModeToStr(newMode),
          sourceBroker ? "broker" : "default",
          resetLearning ? "reset_learning" : "keep_state",
          reason);
  }
}

static void handleRetainModePayload(const String &payloadStr) {
  String val = payloadStr;
  val.trim();
  val.toLowerCase();

  retainFlagSeenThisWake = true;

  if (val == "retain_poll_15m") {
    setSleepMode(SleepMode::RetainPoll15m, true, false, "broker_flag");
    logAi("mode_flag", "value=retain_poll_15m source=broker");
    return;
  }

  if (val == "adaptive") {
    setSleepMode(SleepMode::Adaptive, true, true, "broker_flag");
    logAi("mode_flag", "value=adaptive source=broker");
    return;
  }

  logAi("mode_flag", "value=unknown source=broker raw=%s", val.c_str());
}

static uint32_t clampU32(uint32_t v, uint32_t lo, uint32_t hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

static uint32_t cadenceChecksum(const CadenceModel &model) {
  CadenceModel copy = model;
  copy.checksum = 0;
  const uint8_t *bytes = reinterpret_cast<const uint8_t *>(&copy);
  uint32_t hash = 2166136261u;
  for (size_t i = 0; i < sizeof(CadenceModel); i++) {
    hash ^= bytes[i];
    hash *= 16777619u;
  }
  return hash;
}

static void cadenceSave() {
  cadenceModel.version = CADENCE_MODEL_VERSION;
  cadenceModel.checksum = cadenceChecksum(cadenceModel);
}

static void cadenceReset() {
  memset(&cadenceModel, 0, sizeof(cadenceModel));
  cadenceModel.version = CADENCE_MODEL_VERSION;
  cadenceSave();
}

static bool cadenceIsValid() {
  if (cadenceModel.version != CADENCE_MODEL_VERSION) return false;
  return cadenceModel.checksum == cadenceChecksum(cadenceModel);
}

static void sortU32(uint32_t *arr, uint8_t n) {
  for (uint8_t i = 1; i < n; i++) {
    uint32_t key = arr[i];
    int8_t j = (int8_t)i - 1;
    while (j >= 0 && arr[j] > key) {
      arr[j + 1] = arr[j];
      j--;
    }
    arr[j + 1] = key;
  }
}

static uint32_t medianU32(const uint32_t *arr, uint8_t n) {
  if (n == 0) return 0;
  if ((n & 1) == 1) return arr[n / 2];
  const uint32_t a = arr[(n / 2) - 1];
  const uint32_t b = arr[n / 2];
  return (a + b) / 2;
}

static void cadenceRecomputeStats() {
  if (cadenceModel.mappedCount == 0) {
    cadenceModel.medianIntervalSec = 0;
    cadenceModel.madSec = 0;
    return;
  }

  uint32_t sorted[CADENCE_RING_SIZE];
  for (uint8_t i = 0; i < cadenceModel.mappedCount; i++) {
    sorted[i] = cadenceModel.mappedIntervals[i];
  }
  sortU32(sorted, cadenceModel.mappedCount);
  const uint32_t med = medianU32(sorted, cadenceModel.mappedCount);

  uint32_t dev[CADENCE_RING_SIZE];
  for (uint8_t i = 0; i < cadenceModel.mappedCount; i++) {
    const uint32_t v = cadenceModel.mappedIntervals[i];
    dev[i] = (v >= med) ? (v - med) : (med - v);
  }
  sortU32(dev, cadenceModel.mappedCount);

  cadenceModel.medianIntervalSec = med;
  cadenceModel.madSec = medianU32(dev, cadenceModel.mappedCount);
}

static void cadencePushInterval(uint32_t intervalSec) {
  cadenceModel.mappedIntervals[cadenceModel.mappedHead] = intervalSec;
  cadenceModel.mappedHead = (uint8_t)((cadenceModel.mappedHead + 1) % CADENCE_RING_SIZE);
  if (cadenceModel.mappedCount < CADENCE_RING_SIZE) {
    cadenceModel.mappedCount++;
  }
  cadenceRecomputeStats();
}

static void cadenceOnObservedTimestamp(uint32_t tsEpoch) {
  cadenceModel.sleepInhibit = 0;
  cadenceModel.emptySleepCycles = 0;
  if (!DEEPSLEEP_LEARN_ENABLE) return;
  if (cadenceMode() != SleepMode::Adaptive) return;
  if (tsEpoch == 0) return;

  if (cadenceModel.lastMappedTs == 0) {
    cadenceModel.lastMappedTs = tsEpoch;
    cadenceSave();
    return;
  }

  if (tsEpoch <= cadenceModel.lastMappedTs) {
    return;
  }

  const uint32_t intervalSec = tsEpoch - cadenceModel.lastMappedTs;
  if (intervalSec < DEEPSLEEP_LEARN_MIN_INTERVAL_SEC) {
    // Important: this only skips cadence learning for burst companions
    // (e.g. second message after 1-2s). Message processing still happens normally.
    return;
  }

  if (intervalSec <= CADENCE_MAX_INTERVAL_SEC) {
    cadencePushInterval(intervalSec);
    cadenceModel.missedWindows = 0;
    if (cadenceModel.confidence < 100) {
      uint16_t nextConfidence = (uint16_t)cadenceModel.confidence + 8;
      cadenceModel.confidence = (uint8_t)((nextConfidence > 100) ? 100 : nextConfidence);
    }
    logAi("cadence_update", "interval_sec=%lu median_sec=%lu mad_sec=%lu samples=%u",
          (unsigned long)intervalSec,
          (unsigned long)cadenceModel.medianIntervalSec,
          (unsigned long)cadenceModel.madSec,
          (unsigned)cadenceModel.mappedCount);
    cadenceModel.lastMappedTs = tsEpoch;
    cadenceSave();
  }
}

struct SleepPlan {
  uint32_t sleepSec;
  bool adaptive;
  const char *reason;
};

static SleepPlan makeSleepPlan() {
  SleepPlan plan{};

  if (cadenceMode() == SleepMode::RetainPoll15m) {
    plan.sleepSec = clampU32(DEEPSLEEP_RETAIN_POLL_SEC, DEEPSLEEP_MIN_SEC, DEEPSLEEP_MAX_SEC);
    plan.adaptive = false;
    plan.reason = "retain_poll_15m";
    return plan;
  }

  plan.sleepSec = clampU32(DEEPSLEEP_FALLBACK_SEC, DEEPSLEEP_MIN_SEC, DEEPSLEEP_MAX_SEC);
  plan.adaptive = false;
  plan.reason = "fallback_training";

  if (!DEEPSLEEP_LEARN_ENABLE) {
    plan.reason = "fallback_learning_disabled";
    return plan;
  }

  if (cadenceModel.missedWindows >= CADENCE_MAX_CONSECUTIVE_MISSES) {
    plan.reason = "fallback_misses";
    return plan;
  }

  const bool enoughSamples = cadenceModel.mappedCount >= DEEPSLEEP_MIN_INTERVAL_SAMPLES;
  const bool confidenceOk = cadenceModel.confidence >= DEEPSLEEP_CONFIDENCE_MIN;

  if (!enoughSamples) {
    plan.reason = "fallback_training";
    return plan;
  }
  if (!confidenceOk) {
    plan.reason = "fallback_confidence";
    return plan;
  }

  // Plan wake-up so expected environmental payload lands near the middle
  // of active listening time after reconnect.
  const uint32_t centerOffsetSec = (uint32_t)DEEPSLEEP_LISTEN_WINDOW_SEC / 2U;
  const uint32_t guardSec = max<uint32_t>(CADENCE_SAFETY_FLOOR_SEC,
                                          (CADENCE_MAD_MULTIPLIER * cadenceModel.madSec) +
                                            CADENCE_RECONNECT_BUDGET_SEC + centerOffsetSec);
  uint32_t adaptiveSec = DEEPSLEEP_MIN_SEC;
  if (cadenceModel.medianIntervalSec > guardSec) {
    adaptiveSec = cadenceModel.medianIntervalSec - guardSec;
  }

  plan.sleepSec = clampU32(adaptiveSec, DEEPSLEEP_MIN_SEC, DEEPSLEEP_MAX_SEC);
  plan.adaptive = true;
  plan.reason = "adaptive_median";
  return plan;
}

static void cadenceInitFromRtc() {
  if (!cadenceIsValid()) {
    cadenceReset();
  }
}

static void cadenceApplyWakeOutcome() {
  if (cadenceMode() != SleepMode::Adaptive) return;
  const bool expectedAdaptive = cadenceModel.lastPlanAdaptive != 0;
  if (!expectedAdaptive) return;

  if (mappedUpdateSeenThisWake) {
    cadenceModel.missedWindows = 0;
    uint16_t nextConfidence = (uint16_t)cadenceModel.confidence + CADENCE_CONFIDENCE_INC_HIT;
    cadenceModel.confidence = (uint8_t)((nextConfidence > 100) ? 100 : nextConfidence);
  } else {
    cadenceModel.missedWindows = (cadenceModel.missedWindows < 255) ? (uint8_t)(cadenceModel.missedWindows + 1) : (uint8_t)255;
    cadenceModel.confidence = (cadenceModel.confidence > CADENCE_CONFIDENCE_DEC_MISS)
                                ? (uint8_t)(cadenceModel.confidence - CADENCE_CONFIDENCE_DEC_MISS)
                                : 0;
    logAi("cadence_miss", "misses=%u confidence=%u",
          (unsigned)cadenceModel.missedWindows,
          (unsigned)cadenceModel.confidence);
  }
  cadenceSave();
}

static void maybeEnterDeepSleep(unsigned long nowMs) {
  if (!DEEPSLEEP_ENABLE) return;
  if (sleepStateHandled) return;

  const unsigned long minAwakeMs = (unsigned long)DEEPSLEEP_MIN_AWAKE_SEC * 1000UL;
  if ((nowMs - wakeStartMs) < minAwakeMs) return;

  if (!mqttConnectedThisWake) {
    if (!sleepInhibitLogged) {
      sleepInhibitLogged = true;
      logAi("sleep_inhibit", "reason=no_mqtt_connection");
    }
    return;
  }

  if (cadenceModel.sleepInhibit != 0) {
    if (!sleepInhibitLogged) {
      sleepInhibitLogged = true;
      logAi("sleep_inhibit", "reason=empty_sleep_cycle_guard empty_cycles=%u threshold=%u",
            (unsigned)cadenceModel.emptySleepCycles,
            (unsigned)DEEPSLEEP_MAX_EMPTY_SLEEP_CYCLES);
    }
    return;
  }

  if (hasRetainModeTopic() && cadenceModel.modeSourceBroker != 0 && !retainFlagSeenThisWake) {
    setSleepMode(SleepMode::Adaptive, false, true, "flag_missing");
    logAi("mode_flag", "value=missing source=broker action=reset_to_adaptive");
  }

  const unsigned long listenWindowMs = (unsigned long)DEEPSLEEP_LISTEN_WINDOW_SEC * 1000UL;
  if ((nowMs - wakeStartMs) < listenWindowMs) return;

  if (cadenceMode() == SleepMode::Adaptive && cadenceModel.mappedCount < DEEPSLEEP_MIN_INTERVAL_SAMPLES) {
    if (!trainingWaitLogged) {
      trainingWaitLogged = true;
      logAi("sleep_wait_training", "samples=%u required=%u mode=continuous_listen",
            (unsigned)cadenceModel.mappedCount,
            (unsigned)DEEPSLEEP_MIN_INTERVAL_SAMPLES);
    }
    return;
  }

  // If we repeatedly miss expected windows, reset learning and return
  // to continuous listening so the model can re-lock to current cadence.
  if (cadenceMode() == SleepMode::Adaptive &&
      cadenceModel.missedWindows >= CADENCE_MAX_CONSECUTIVE_MISSES &&
      !mappedUpdateSeenThisWake) {
    const uint8_t missesBeforeReset = cadenceModel.missedWindows;
    cadenceResetLearningData();
    cadenceModel.emptySleepCycles = 0;
    cadenceModel.sleepInhibit = 0;
    cadenceSave();
    logAi("cadence_retrain", "reason=consecutive_misses misses=%u threshold=%u action=continuous_listen",
          (unsigned)missesBeforeReset,
          (unsigned)CADENCE_MAX_CONSECUTIVE_MISSES);
    return;
  }

  // Sample threshold reached; we can start sleep planning from now on.
  trainingWaitLogged = false;

  cadenceApplyWakeOutcome();

  const SleepPlan plan = makeSleepPlan();

  if (!mappedUpdateSeenThisWake && cadenceMode() == SleepMode::Adaptive) {
    uint16_t nextEmpty = (uint16_t)(cadenceModel.emptySleepCycles + 1);
    if (nextEmpty >= DEEPSLEEP_MAX_EMPTY_SLEEP_CYCLES) {
      cadenceModel.sleepInhibit = 1;
      cadenceModel.emptySleepCycles = nextEmpty;
      cadenceSave();
      logAi("sleep_inhibit", "reason=empty_sleep_cycle_guard empty_cycles=%u threshold=%u",
            (unsigned)cadenceModel.emptySleepCycles,
            (unsigned)DEEPSLEEP_MAX_EMPTY_SLEEP_CYCLES);
      return;
    }
    cadenceModel.emptySleepCycles = nextEmpty;
  } else if (mappedUpdateSeenThisWake) {
    cadenceModel.emptySleepCycles = 0;
  }

  sleepStateHandled = true;
  cadenceModel.lastPlanAdaptive = plan.adaptive ? 1 : 0;
  cadenceModel.lastPlanSleepSec = plan.sleepSec;
  cadenceSave();

  logAi("sleep_plan", "mode=%s reason=%s sleep_sec=%lu median_sec=%lu mad_sec=%lu confidence=%u samples=%u misses=%u",
        plan.adaptive ? "adaptive" : "fallback", plan.reason,
        (unsigned long)plan.sleepSec,
        (unsigned long)cadenceModel.medianIntervalSec,
        (unsigned long)cadenceModel.madSec,
        (unsigned)cadenceModel.confidence,
        (unsigned)cadenceModel.mappedCount,
        (unsigned)cadenceModel.missedWindows);
  logAi("sleep_enter", "sleep_sec=%lu", (unsigned long)plan.sleepSec);

  delay(50);
  Serial.flush();
  esp_sleep_enable_timer_wakeup((uint64_t)plan.sleepSec * 1000000ULL);
  esp_deep_sleep_start();
}

static unsigned long nextLogSeq() {
  return ++logLineSeq;
}

static void logf(const char *tag, const char *fmt, ...) {
  va_list args;
  va_start(args, fmt);
  char msg[256];
  vsnprintf(msg, sizeof(msg), fmt, args);
  va_end(args);
  const unsigned long seq = nextLogSeq();
  Serial.printf("[%10lu|%06lu] %s: %s\n", millis(), seq, tag, msg);
}

static void logAi(const char *eventName, const char *fmt, ...) {
  char msg[256];
  msg[0] = '\0';

  va_list args;
  va_start(args, fmt);
  vsnprintf(msg, sizeof(msg), fmt, args);
  va_end(args);

  const unsigned long seq = nextLogSeq();
  if (msg[0] != '\0') {
    Serial.printf("[%10lu|%06lu] AI: event=%s %s\n", millis(), seq, eventName, msg);
  } else {
    Serial.printf("[%10lu|%06lu] AI: event=%s\n", millis(), seq, eventName);
  }
}

static const char *resetReasonToStr(esp_reset_reason_t reason) {
  switch (reason) {
    case ESP_RST_UNKNOWN: return "unknown";
    case ESP_RST_POWERON: return "poweron";
    case ESP_RST_EXT: return "ext";
    case ESP_RST_SW: return "sw";
    case ESP_RST_PANIC: return "panic";
    case ESP_RST_INT_WDT: return "int_wdt";
    case ESP_RST_TASK_WDT: return "task_wdt";
    case ESP_RST_WDT: return "other_wdt";
    case ESP_RST_DEEPSLEEP: return "deepsleep";
    case ESP_RST_BROWNOUT: return "brownout";
    case ESP_RST_SDIO: return "sdio";
    default: return "?";
  }
}

static const char *wakeupCauseToStr(esp_sleep_wakeup_cause_t cause) {
  switch (cause) {
    case ESP_SLEEP_WAKEUP_UNDEFINED: return "undefined";
    case ESP_SLEEP_WAKEUP_ALL: return "all";
    case ESP_SLEEP_WAKEUP_EXT0: return "ext0";
    case ESP_SLEEP_WAKEUP_EXT1: return "ext1";
    case ESP_SLEEP_WAKEUP_TIMER: return "timer";
    case ESP_SLEEP_WAKEUP_TOUCHPAD: return "touchpad";
    case ESP_SLEEP_WAKEUP_ULP: return "ulp";
    case ESP_SLEEP_WAKEUP_GPIO: return "gpio";
    case ESP_SLEEP_WAKEUP_UART: return "uart";
    case ESP_SLEEP_WAKEUP_WIFI: return "wifi";
    case ESP_SLEEP_WAKEUP_COCPU: return "cocpu";
    case ESP_SLEEP_WAKEUP_COCPU_TRAP_TRIG: return "cocpu_trap";
    case ESP_SLEEP_WAKEUP_BT: return "bt";
    default: return "?";
  }
}

static void logBootInfo() {
  const esp_reset_reason_t rr = esp_reset_reason();
  const esp_sleep_wakeup_cause_t wc = esp_sleep_get_wakeup_cause();
  logf("BOOT", "reset_reason=%s wakeup_cause=%s", resetReasonToStr(rr), wakeupCauseToStr(wc));
  logf("BOOT", "sdk=%s cpu_mhz=%u heap_free=%u", ESP.getSdkVersion(), (unsigned)ESP.getCpuFreqMHz(), (unsigned)ESP.getFreeHeap());
  logAi("boot_info", "reset_reason=%s wakeup_cause=%s sdk=%s cpu_mhz=%u heap_free=%u",
        resetReasonToStr(rr), wakeupCauseToStr(wc), ESP.getSdkVersion(),
        (unsigned)ESP.getCpuFreqMHz(), (unsigned)ESP.getFreeHeap());
}

static void initDisplayIfNeeded() {
  if (displayInitialized) return;
  logf("EINK", "initializing (deferred until first sensor data)");
  VextON();
  delay(100);
  display.init();
  displayInitialized = true;
  logf("EINK", "init done");
}

struct Trend3 {
  float v[3] = {NAN, NAN, NAN};
  uint8_t count = 0;

  void push(float value) {
    if (isnan(value)) return;
    v[0] = v[1];
    v[1] = v[2];
    v[2] = value;
    if (count < 3) count++;
  }

  bool hasDelta() const { return count >= 2 && !isnan(v[2]) && !isnan(v[1]); }

  float deltaFromBaseline() const {
    if (!hasDelta()) return 0.0f;
    if (count >= 3 && !isnan(v[0])) {
      const float baseline = (v[0] + v[1]) * 0.5f;
      return v[2] - baseline;
    }
    return v[2] - v[1];
  }
};

static Trend3 tempTrend;
static Trend3 humidityTrend;
static Trend3 pressureTrend;

enum class TrendDir : int8_t { Flat = 0, Down = -1, Up = 1 };

static TrendDir trendDirection(float delta, float deadband) {
  if (fabsf(delta) < deadband) return TrendDir::Flat;
  return (delta > 0) ? TrendDir::Up : TrendDir::Down;
}

static int16_t trendArrowSize(float absDelta, float small, float medium) {
  if (absDelta >= medium) return 14;
  if (absDelta >= small) return 10;
  return 6;
}

static void drawArrow(int16_t cx, int16_t cy, int16_t size, TrendDir dir) {
  if (dir == TrendDir::Flat || size <= 0) {
    // small dash
    display.drawHorizontalLine(cx - 4, cy, 8);
    return;
  }

  const int16_t half = size / 2;
  if (dir == TrendDir::Up) {
    // shaft
    display.drawLine(cx, cy + half, cx, cy - half);
    // head
    display.drawLine(cx, cy - half, cx - half, cy - half + half / 2);
    display.drawLine(cx, cy - half, cx + half, cy - half + half / 2);
  } else {
    display.drawLine(cx, cy - half, cx, cy + half);
    display.drawLine(cx, cy + half, cx - half, cy + half - half / 2);
    display.drawLine(cx, cy + half, cx + half, cy + half - half / 2);
  }
}

struct Seg7 {
  bool a;
  bool b;
  bool c;
  bool d;
  bool e;
  bool f;
  bool g;
};

static Seg7 seg7ForChar(char ch) {
  switch (ch) {
    case '0':
      return {true, true, true, true, true, true, false};
    case '1':
      return {false, true, true, false, false, false, false};
    case '2':
      return {true, true, false, true, true, false, true};
    case '3':
      return {true, true, true, true, false, false, true};
    case '4':
      return {false, true, true, false, false, true, true};
    case '5':
      return {true, false, true, true, false, true, true};
    case '6':
      return {true, false, true, true, true, true, true};
    case '7':
      return {true, true, true, false, false, false, false};
    case '8':
      return {true, true, true, true, true, true, true};
    case '9':
      return {true, true, true, true, false, true, true};
    case '-':
      return {false, false, false, false, false, false, true};
    default:
      return {false, false, false, false, false, false, false};
  }
}

static void drawSeg7Glyph(int16_t x, int16_t y, int16_t w, int16_t h, int16_t t, const Seg7 &s) {
  // Segment geometry
  const int16_t innerW = max<int16_t>(0, w - 2 * t);
  const int16_t halfH = h / 2;
  const int16_t vertLen = max<int16_t>(0, halfH - 2 * t);
  const int16_t midY = y + halfH - (t / 2);

  // a (top)
  if (s.a) display.fillRect(x + t, y, innerW, t);
  // d (bottom)
  if (s.d) display.fillRect(x + t, y + h - t, innerW, t);
  // g (middle)
  if (s.g) display.fillRect(x + t, midY, innerW, t);

  // f (upper-left)
  if (s.f) display.fillRect(x, y + t, t, vertLen);
  // b (upper-right)
  if (s.b) display.fillRect(x + w - t, y + t, t, vertLen);

  // e (lower-left)
  if (s.e) display.fillRect(x, y + halfH + (t / 2), t, vertLen);
  // c (lower-right)
  if (s.c) display.fillRect(x + w - t, y + halfH + (t / 2), t, vertLen);
}

static int16_t seg7CharWidth(int16_t digitW, int16_t gap, char ch) {
  if (ch == '.') return (gap + 6);
  if (ch == ' ') return gap;
  return digitW + gap;
}

static void formatEpochDdMmHm(uint32_t epochSeconds, char *out, size_t outSize) {
  if (out == nullptr || outSize == 0) return;
  if (epochSeconds == 0) {
    snprintf(out, outSize, "--");
    return;
  }

  time_t t = (time_t)epochSeconds;
  struct tm tmUtc;
  gmtime_r(&t, &tmUtc);
  snprintf(out, outSize, "%02d.%02d %02d:%02d", tmUtc.tm_mday, tmUtc.tm_mon + 1, tmUtc.tm_hour, tmUtc.tm_min);
}

static void formatFloatOrDash(char *out, size_t outSize, float value, uint8_t decimals, const char *dashText) {
  if (out == nullptr || outSize == 0) return;
  if (isnan(value)) {
    snprintf(out, outSize, "%s", dashText);
    return;
  }

  char fmt[8];
  snprintf(fmt, sizeof(fmt), "%%.%uf", (unsigned)decimals);
  snprintf(out, outSize, fmt, value);
}

static void appendFmt(char *dst, size_t dstSize, const char *fmt, ...) {
  if (dst == nullptr || dstSize == 0) return;
  const size_t used = strlen(dst);
  if (used >= (dstSize - 1)) return;

  va_list args;
  va_start(args, fmt);
  vsnprintf(dst + used, dstSize - used, fmt, args);
  va_end(args);
}

static float readBatteryVoltageV() {
#if defined(BATTERY_ADC_PIN)
  if (BATTERY_ADC_PIN < 0) return NAN;
  const unsigned long now = millis();
  if (!isnan(lastBatteryVoltageV) && (now - lastBatteryReadMs) < 2000) {
    return lastBatteryVoltageV;
  }

  lastBatteryReadMs = now;

#if defined(BATTERY_ADC_CTRL_PIN) && defined(BATTERY_ADC_CTRL_ENABLED)
  // Some Heltec boards gate the battery divider/measurement to reduce drain.
  pinMode(BATTERY_ADC_CTRL_PIN, OUTPUT);
  digitalWrite(BATTERY_ADC_CTRL_PIN, BATTERY_ADC_CTRL_ENABLED);
  delay(10);
#endif

  analogReadResolution(12);
#if defined(BATTERY_ADC_ATTENUATION)
  analogSetPinAttenuation(BATTERY_ADC_PIN, BATTERY_ADC_ATTENUATION);
#endif

  const int mv = analogReadMilliVolts(BATTERY_ADC_PIN);

#if defined(BATTERY_ADC_CTRL_PIN) && defined(BATTERY_ADC_CTRL_ENABLED)
  digitalWrite(BATTERY_ADC_CTRL_PIN, !BATTERY_ADC_CTRL_ENABLED);
#endif

  if (mv <= 0) {
    lastBatteryVoltageV = NAN;
    return NAN;
  }

  float v = (float)mv / 1000.0f;
#if defined(BATTERY_DIVIDER)
  v *= (float)BATTERY_DIVIDER;
#endif

#if defined(BATTERY_MIN_VALID_V) && defined(BATTERY_MAX_VALID_V)
  if (v < (float)BATTERY_MIN_VALID_V || v > (float)BATTERY_MAX_VALID_V) {
    lastBatteryVoltageV = NAN;
    return NAN;
  }
#else
  if (v < 2.5f || v > 5.5f) {
    lastBatteryVoltageV = NAN;
    return NAN;
  }
#endif

  lastBatteryVoltageV = v;
  return v;
#else
  return NAN;
#endif
}

static int batteryPercentFromVoltage(float v) {
  if (isnan(v)) return -1;
#if defined(BATTERY_V_EMPTY) && defined(BATTERY_V_FULL)
  const float emptyV = (float)BATTERY_V_EMPTY;
  const float fullV = (float)BATTERY_V_FULL;
  if (fullV <= emptyV) return -1;
  float pct = (v - emptyV) / (fullV - emptyV) * 100.0f;
  if (pct < 0.0f) pct = 0.0f;
  if (pct > 100.0f) pct = 100.0f;
  return (int)lroundf(pct);
#else
  float pct = (v - 3.3f) / (4.2f - 3.3f) * 100.0f;
  if (pct < 0.0f) pct = 0.0f;
  if (pct > 100.0f) pct = 100.0f;
  return (int)lroundf(pct);
#endif
}

static void drawBigTemperature(int16_t x, int16_t y, const char *tempDigits, int16_t digitW, int16_t digitH, int16_t thickness,
                               int16_t gap) {
  if (tempDigits == nullptr) return;

  int16_t cx = x;
  const size_t len = strlen(tempDigits);
  for (size_t i = 0; i < len; i++) {
    const char ch = tempDigits[i];
    if (ch == '.') {
      // Draw decimal dot near the bottom.
      display.fillRect(cx + 2, y + digitH - 10, 6, 6);
      cx += (gap + 6);
      continue;
    }
    const Seg7 s = seg7ForChar(ch);
    drawSeg7Glyph(cx, y, digitW, digitH, thickness, s);
    cx += digitW + gap;
  }
}



static void VextON() {
  pinMode(45, OUTPUT);
  digitalWrite(45, LOW);
}



static void drawTileValueWithTrend(int16_t x, int16_t y, int16_t w, int16_t h, const char *label,
                                   const char *digits, const char *unit,
                                   TrendDir dir, int16_t arrowSize) {
  display.drawRect(x, y, w, h);

  display.setTextAlignment(TEXT_ALIGN_LEFT);
  display.setFont(ArialMT_Plain_10);
  display.drawString(x + 6, y + 3, String(label));

  // Value (digits): use built-in Arial 24 for a clean ~50% increase vs Arial 16.
  const int16_t valueY = y + 10;
  display.setFont(ArialMT_Plain_24);
  display.drawString(x + 6, valueY, String(digits));

  // Unit small, right after digits
  const int16_t digitsW = display.getStringWidth(digits, strlen(digits));
  display.setFont(ArialMT_Plain_10);
  display.drawString(x + 6 + digitsW + 3, valueY + 12, String(unit));

  // Trend arrow
  const int16_t arrowCx = x + w - 12;
  const int16_t arrowCy = y + (h / 2) + 4;
  drawArrow(arrowCx, arrowCy, arrowSize, dir);
}

static void drawTemperatureScreen() {
  initDisplayIfNeeded();
  const bool wifiOk = (WiFi.status() == WL_CONNECTED);
  const bool mqttOk = mqttClient.connected();

  // Heltec library uses padded geometry for this panel (256x128) even though the
  // visible area is 250x122. If we use display.width()/height(), drawings can go
  // into the non-visible padded region and appear clipped.
  const int16_t w = 250;
  const int16_t h = 122;
  const int16_t margin = 2;
  const int16_t footerH = 13;  // ArialMT_Plain_10 height
  const int16_t headerH = 13;  // reserve one text line for device name

  char tempDigits[16];
  char humidityDigits[16];
  char pressureDigits[16];
  formatFloatOrDash(tempDigits, sizeof(tempDigits), lastTemperatureC, 1, "--.-");
  formatFloatOrDash(humidityDigits, sizeof(humidityDigits), lastHumidityPct, 1, "--.-");
  formatFloatOrDash(pressureDigits, sizeof(pressureDigits), lastPressureHpa, 1, "----.-");

  display.clear();
  display.update(BLACK_BUFFER);

  display.clear();

  // Big temperature: 7-seg digits (~2x, artifact-free)
  const int16_t digitH = 52;
  const int16_t digitW = 30;
  const int16_t digitGap = 6;
  const int16_t thickness = 5;
  const int16_t tempY = headerH + 1;

  int16_t tempDigitsW = 0;
  const size_t tempDigitsLen = strlen(tempDigits);
  for (size_t i = 0; i < tempDigitsLen; i++) {
    tempDigitsW += seg7CharWidth(digitW, digitGap, tempDigits[i]);
  }
  const int16_t unitReserve = 28;
  const int16_t totalW = tempDigitsW + unitReserve;
  const int16_t tempX = (w - totalW) / 2;

  // Device name (top-left), as high as possible.
  {
    const int16_t nameX = margin;
    const int16_t nameY = 0;
    const int16_t maxNameW = w - 2 * margin;
    if (maxNameW > 0) {
      char name[40];
      snprintf(name, sizeof(name), "%s", MQTT_DEVICE_NAME);
      display.setFont(ArialMT_Plain_10);
      display.setTextAlignment(TEXT_ALIGN_LEFT);
      display.setColor(WHITE);
      size_t nameLen = strlen(name);
      while (nameLen > 0 && display.getStringWidth(name, nameLen) > maxNameW) {
        name[--nameLen] = '\0';
      }
      if (nameLen > 0) {
        display.drawString(nameX, nameY, String(name));
      }
    }
  }

  display.setColor(WHITE);
  drawBigTemperature(tempX, tempY, tempDigits, digitW, digitH, thickness, digitGap);

  // Draw unit as a circle + small 'C' (avoids font artifacts for the degree glyph).
  {
    const int16_t unitX = tempX + tempDigitsW + 4;
    display.setColor(WHITE);
    display.drawCircle(unitX + 6, tempY + 12, 3);
    display.setFont(ArialMT_Plain_16);
    display.setTextAlignment(TEXT_ALIGN_LEFT);
    display.drawString(unitX + 12, tempY + 8, "C");
  }

  // Temp trend arrow near the right side of temp block
  {
    const float d = tempTrend.deltaFromBaseline();
    const TrendDir dir = trendDirection(d, 0.05f);
    const int16_t arrowSize = trendArrowSize(fabsf(d), 0.10f, 0.30f);
    drawArrow(w - 14, tempY + 18, arrowSize, dir);
  }

  // Tiles
  const int16_t tileY = tempY + digitH + 4;
  const int16_t tileGap = 6;
  const int16_t footerY = h - footerH;
  const int16_t tileH = max<int16_t>(26, (int16_t)(footerY - tileY - 2));
  const int16_t tileWLeft = (w - tileGap - (2 * margin)) / 2;
  const int16_t tileWRight = (w - (2 * margin)) - tileWLeft - tileGap;
  {
    const float d = humidityTrend.deltaFromBaseline();
    const TrendDir dir = trendDirection(d, 0.2f);
    const int16_t arrowSize = trendArrowSize(fabsf(d), 0.5f, 2.0f);
    drawTileValueWithTrend(margin, tileY, tileWLeft, tileH, "RH", humidityDigits, "%", dir, arrowSize);
  }
  {
    const float d = pressureTrend.deltaFromBaseline();
    const TrendDir dir = trendDirection(d, 0.2f);
    const int16_t arrowSize = trendArrowSize(fabsf(d), 0.5f, 1.0f);
    drawTileValueWithTrend(margin + tileWLeft + tileGap, tileY, tileWRight, tileH, "Press", pressureDigits, "hPa", dir,
                           arrowSize);
  }

  // Footer
  display.setFont(ArialMT_Plain_10);
  {
    // Battery on the right side (optional)
    const float vbat = readBatteryVoltageV();
    const int pct = batteryPercentFromVoltage(vbat);
    char bat[32];
    bat[0] = '\0';
    {
      char vbuf[16];
      if (!isnan(vbat)) {
        snprintf(vbuf, sizeof(vbuf), "%.2fV", vbat);
      } else {
        snprintf(vbuf, sizeof(vbuf), "--.--V");
      }

      if (pct >= 0) {
        snprintf(bat, sizeof(bat), "BAT %d%% %s", pct, vbuf);
      } else {
        snprintf(bat, sizeof(bat), "BAT --%% %s", vbuf);
      }
    }

    // Left side: only MQTT state + optional timestamp + RSSI (no msg/data ages)
    char left[96];
    left[0] = '\0';

    if (!mqttOk) {
      appendFmt(left, sizeof(left), "%s", "MQTT wait");
    } else {
      appendFmt(left, sizeof(left), "%s", "MQTT ok");
      if (lastMqttMessageMs == 0) {
        appendFmt(left, sizeof(left), "%s", " (no msg)");
      } else if (lastMqttTimestampValid) {
        char ts[18];
        formatEpochDdMmHm(lastMqttTimestampEpoch, ts, sizeof(ts));
        appendFmt(left, sizeof(left), " (%s)", ts);
      }
    }

    if (wifiOk) {
      const int rssi = lastWifiRssiValid ? lastWifiRssiDbm : WiFi.RSSI();
      appendFmt(left, sizeof(left), " rssi %d", rssi);
    }

    // Prevent overlap between left text and right-aligned BAT.
    const int16_t batW = display.getStringWidth(bat, strlen(bat));
    const int16_t gapPx = 6;
    const int16_t leftMaxW = (w - 2 * margin) - batW - gapPx;
    if (leftMaxW > 0) {
      size_t leftLen = strlen(left);
      while (leftLen > 0 && display.getStringWidth(left, leftLen) > leftMaxW) {
        left[--leftLen] = '\0';
      }
    }

    display.setTextAlignment(TEXT_ALIGN_LEFT);
    display.drawString(margin, footerY, String(left));

    display.setTextAlignment(TEXT_ALIGN_RIGHT);
    display.drawString(w - margin, footerY, String(bat));
  }

  display.update(BLACK_BUFFER);
  display.display();
}

static bool connectWiFi(unsigned long timeoutMs) {
  if (WiFi.status() == WL_CONNECTED) {
    return true;
  }

  const unsigned long start = millis();
  logf("WIFI", "connecting ssid=%s timeout_ms=%lu", WIFI_SSID, timeoutMs);
  // Clear any half-open state before (re)connecting.
  WiFi.disconnect(true);
  delay(50);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  while (WiFi.status() != WL_CONNECTED && (millis() - start) < timeoutMs) {
    delay(250);
  }

  if (WiFi.status() != WL_CONNECTED) {
    logf("WIFI", "connect timeout after %lums status=%d", (millis() - start), (int)WiFi.status());
    return false;
  }

  const String ipStr = WiFi.localIP().toString();
  logf("WIFI", "connected in %lums ip=%s rssi=%d", (millis() - start), ipStr.c_str(), WiFi.RSSI());
  return true;
}

static String mqttClientId() {
  uint64_t chipId = ESP.getEfuseMac();
  char buf[40];
  snprintf(buf, sizeof(buf), "%s-%08lx%08lx", MQTT_DEVICE_NAME,
           (uint32_t)(chipId >> 32), (uint32_t)(chipId & 0xFFFFFFFF));
  return String(buf);
}

static bool tryReadFloatByKey(JsonVariant obj, const char *key, float &out) {
  if (key == nullptr) return false;
  if (key[0] == '\0') return false;
  if (obj[key].isNull()) return false;
  out = obj[key].as<float>();
  return isfinite(out);
}

static bool payloadLooksLikeJson(const String &payload) {
  for (size_t i = 0; i < payload.length(); i++) {
    const unsigned char ch = (unsigned char)payload[i];
    if (!isspace(ch)) {
      return ch == '{' || ch == '[';
    }
  }
  return false;
}

static bool payloadHasEmbeddedNul(const byte *payload, unsigned int length) {
  if (payload == nullptr || length <= 1) return false;
  return memchr(payload, '\0', length - 1) != nullptr;
}

static void mqttCallback(char *topic, byte *payload, unsigned int length) {
  if (topic == nullptr || payload == nullptr) {
    logf("MQTT", "rx invalid frame (null topic/payload)");
    logAi("mqtt_rx_ignored", "reason=invalid_frame");
    return;
  }

  if (length == 0 && isRetainModeTopic(topic)) {
    setSleepMode(SleepMode::Adaptive, false, true, "flag_deleted");
    retainFlagSeenThisWake = false;
    logAi("mode_flag", "value=missing source=broker action=reset_to_adaptive");
    return;
  }

  if (length == 0) {
    logf("MQTT", "rx topic=%s len=0 (ignored)", topic);
    logAi("mqtt_rx_ignored", "reason=empty topic=%s", topic);
    return;
  }

  if (payloadHasEmbeddedNul(payload, length)) {
    logf("MQTT", "rx topic=%s len=%u contains embedded NUL (ignored)", topic, length);
    logAi("mqtt_rx_ignored", "reason=embedded_nul topic=%s len=%u", topic, length);
    return;
  }

  lastMqttMessageMs = millis();

  if (WiFi.status() == WL_CONNECTED) {
    lastWifiRssiDbm = WiFi.RSSI();
    lastWifiRssiValid = true;
  }

  // Construct String directly from pointer+length (avoids O(n²) char-by-char append).
  const String payloadStr((const char *)payload, length);

  if (isRetainModeTopic(topic)) {
    handleRetainModePayload(payloadStr);
    return;
  }

  char rssiBuf[16];
  if (lastWifiRssiValid) {
    snprintf(rssiBuf, sizeof(rssiBuf), "%d", lastWifiRssiDbm);
  } else {
    snprintf(rssiBuf, sizeof(rssiBuf), "?");
  }
  logf("MQTT", "rx topic=%s len=%u rssi=%s", topic, length, rssiBuf);
  logAi("mqtt_rx", "topic=%s len=%u rssi=%s", topic, length, rssiBuf);
  if (length <= 512) {
    logf("MQTT", "payload=%s", payloadStr.c_str());
  }

  float parsedTemp = NAN;
  float parsedPressure = NAN;
  float parsedHumidity = NAN;
  bool plainNumberParsed = false;
  const char *parseSource = "none";
  uint32_t parsedTimestampEpoch = 0;
  bool parsedTimestampValid = false;

  // 1) Try plain float (treat as temperature)
  {
    const char *start = payloadStr.c_str();
    char *endPtr = nullptr;
    parsedTemp = strtof(start, &endPtr);

    while (endPtr != nullptr && *endPtr != '\0' && isspace((unsigned char)*endPtr)) {
      endPtr++;
    }

    plainNumberParsed = (endPtr != nullptr) && (endPtr != start) && (*endPtr == '\0') && isfinite(parsedTemp);
    if (!plainNumberParsed) {
      parsedTemp = NAN;
    } else {
      parseSource = "plain_float";
    }
  }

  // 2) Try JSON. Supported shapes:
  // - {"temperature": 23.5, "barometric_pressure": 993.2}
  // - {"temp": 23.5}
  // - {"payload": {"temperature": 23.5, "barometric_pressure": 993.2}}
  if (!plainNumberParsed && payloadLooksLikeJson(payloadStr)) {
    JsonDocument doc;
    const DeserializationError err = deserializeJson(doc, payloadStr);
    if (!err) {
      parseSource = "json";
      JsonVariant root = doc.as<JsonVariant>();

      JsonVariant sensor = root;

      if (root["payload"].is<JsonObject>()) {
        sensor = root["payload"]; // your example uses root.payload.{temperature,barometric_pressure}
      }

      if (!root["timestamp"].isNull()) {
        parsedTimestampEpoch = root["timestamp"].as<uint32_t>();
        parsedTimestampValid = (parsedTimestampEpoch != 0);
        if (parsedTimestampValid) {
          lastMqttTimestampEpoch = parsedTimestampEpoch;
          lastMqttTimestampValid = true;
        }
      }

      if (parsedTimestampValid) {
        char ts[18];
        formatEpochDdMmHm(parsedTimestampEpoch, ts, sizeof(ts));
        logf("MQTT", "json_timestamp=%lu (%s)", (unsigned long)parsedTimestampEpoch, ts);
      }

      // Field mapping: can be customized in include/secrets.h
      // Defaults preserve the original behavior.
      (void)tryReadFloatByKey(sensor, JSON_TEMP_KEY, parsedTemp);
      (void)tryReadFloatByKey(sensor, JSON_PRESSURE_KEY, parsedPressure);
      (void)tryReadFloatByKey(sensor, JSON_HUMIDITY_KEY, parsedHumidity);
    } else {
      logf("MQTT", "json parse error: %s", err.c_str());
      logAi("mqtt_parse_error", "topic=%s error=%s", topic, err.c_str());
    }
  }

  bool anyUpdate = false;
  if (!isnan(parsedTemp)) {
    lastTemperatureC = parsedTemp;
    tempTrend.push(parsedTemp);
    anyUpdate = true;
  }
  if (!isnan(parsedPressure)) {
    lastPressureHpa = parsedPressure;
    pressureTrend.push(parsedPressure);
    anyUpdate = true;
  }
  if (!isnan(parsedHumidity)) {
    lastHumidityPct = parsedHumidity;
    humidityTrend.push(parsedHumidity);
    anyUpdate = true;
  }

  if (anyUpdate) {
    mappedUpdateSeenThisWake = true;
    if (parsedTimestampValid) {
      cadenceOnObservedTimestamp(parsedTimestampEpoch);
    }
    lastTempUpdateMs = millis();
    displayDirty = true;
    char tStr[16];
    char hStr[16];
    char pStr[16];
    formatFloatOrDash(tStr, sizeof(tStr), parsedTemp, 2, "-");
    formatFloatOrDash(hStr, sizeof(hStr), parsedHumidity, 2, "-");
    formatFloatOrDash(pStr, sizeof(pStr), parsedPressure, 2, "-");
    logf("DATA", "updated temp=%s hum=%s press=%s", tStr, hStr, pStr);
    logAi("data_update", "source=%s temp=%s hum=%s press=%s refresh=1", parseSource, tStr, hStr, pStr);
  } else {
    logf("DATA", "no mapped fields in message (no screen refresh)");
    logAi("data_noop", "source=%s reason=no_mapped_fields refresh=0", parseSource);
  }
}

static bool ensureMqttConnected() {
  if (mqttClient.connected()) {
    return true;
  }

  if (WiFi.status() != WL_CONNECTED) {
    logAi("mqtt_connect_skip", "reason=wifi_not_connected");
    return false;
  }

  static const String clientId = mqttClientId();
  const unsigned long start = millis();
  logf("MQTT", "connecting host=%s port=%u clientId=%s", MQTT_HOST, MQTT_PORT, clientId.c_str());

  bool ok;
  if (strlen(MQTT_USERNAME) > 0) {
    ok = mqttClient.connect(clientId.c_str(), MQTT_USERNAME, MQTT_PASSWORD);
  } else {
    ok = mqttClient.connect(clientId.c_str());
  }

  if (!ok) {
    logf("MQTT", "connect failed rc=%d after %lums", mqttClient.state(), (millis() - start));
    logAi("mqtt_connect", "result=failed rc=%d elapsed_ms=%lu", mqttClient.state(), (unsigned long)(millis() - start));
    return false;
  }

  logf("MQTT", "connected in %lums", (millis() - start));
  logAi("mqtt_connect", "result=ok elapsed_ms=%lu", (unsigned long)(millis() - start));
  mqttConnectedThisWake = true;

  if (mqttClient.subscribe(MQTT_SUBSCRIBE_TOPIC)) {
    logf("MQTT", "subscribed topic=%s", MQTT_SUBSCRIBE_TOPIC);
    logAi("mqtt_subscribe", "result=ok topic=%s", MQTT_SUBSCRIBE_TOPIC);
  } else {
    logf("MQTT", "subscribe failed topic=%s", MQTT_SUBSCRIBE_TOPIC);
    logAi("mqtt_subscribe", "result=failed topic=%s", MQTT_SUBSCRIBE_TOPIC);
  }

  if (hasRetainModeTopic()) {
    if (mqttClient.subscribe(DEEPSLEEP_RETAIN_MODE_TOPIC)) {
      logAi("mqtt_subscribe", "result=ok topic=%s", DEEPSLEEP_RETAIN_MODE_TOPIC);
    } else {
      logAi("mqtt_subscribe", "result=failed topic=%s", DEEPSLEEP_RETAIN_MODE_TOPIC);
    }
  }

  // Quiet boot: do not touch the display until we actually have sensor data.
  if (lastTempUpdateMs != 0) {
    displayDirty = true;
  }
  return true;
}

void setup() {
  Serial.begin(115200);
  delay(200);
  wakeStartMs = millis();
  cadenceInitFromRtc();

  logBootInfo();
  logf("BOOT", "quiet boot enabled: display init+draw deferred until sensor data");
  logAi("session_start", "schema=%u fw=%s topic=%s min_refresh_ms=%lu periodic_refresh_ms=%lu",
      (unsigned)AI_LOG_SCHEMA_VERSION, FW_VERSION, MQTT_SUBSCRIBE_TOPIC,
      (unsigned long)DISPLAY_MIN_REFRESH_MS, (unsigned long)DISPLAY_PERIODIC_REFRESH_MS);
    logAi("sleep_wake", "cause=%s deepsleep_enable=%u mode=%s source=%s last_plan_mode=%s last_plan_sleep_sec=%lu confidence=%u samples=%u empty_cycles=%u inhibit=%u",
        wakeupCauseToStr(esp_sleep_get_wakeup_cause()), (unsigned)DEEPSLEEP_ENABLE,
      sleepModeToStr(cadenceMode()), cadenceModel.modeSourceBroker ? "broker" : "default",
        cadenceModel.lastPlanAdaptive ? "adaptive" : "fallback",
        (unsigned long)cadenceModel.lastPlanSleepSec,
        (unsigned)cadenceModel.confidence,
      (unsigned)cadenceModel.mappedCount,
      (unsigned)cadenceModel.emptySleepCycles,
      (unsigned)cadenceModel.sleepInhibit);

  WiFi.onEvent([](WiFiEvent_t event, WiFiEventInfo_t info) {
    if (event == ARDUINO_EVENT_WIFI_STA_DISCONNECTED) {
      wifiDisconnectedEvent = true;
      logf("WIFI", "disconnected reason=%u", info.wifi_sta_disconnected.reason);
    } else if (event == ARDUINO_EVENT_WIFI_STA_GOT_IP) {
      const String ipStr = WiFi.localIP().toString();
      logf("WIFI", "got_ip ip=%s", ipStr.c_str());
    } else if (event == ARDUINO_EVENT_WIFI_STA_CONNECTED) {
      logf("WIFI", "sta_connected");
    }
  });

  // Quiet boot: keep the previous e-ink image (no init, no clear).
  // We'll initialize and draw only when we receive mapped sensor data.

  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  WiFi.setAutoReconnect(true);
  WiFi.persistent(false);

  (void)connectWiFi(20000);

  mqttClient.setServer(MQTT_HOST, MQTT_PORT);
  mqttClient.setCallback(mqttCallback);
  mqttClient.setKeepAlive(30);
  mqttClient.setSocketTimeout(5);
#if defined(MQTT_BUFFER_SIZE)
  mqttClient.setBufferSize(MQTT_BUFFER_SIZE);
#endif

  (void)ensureMqttConnected();
  lastDisplayRefreshMs = millis();
}

void loop() {
  if (wifiDisconnectedEvent) {
    wifiDisconnectedEvent = false;
    if (lastTempUpdateMs != 0) {
      displayDirty = true;
    }
  }

  // Keep WiFi alive
  if (WiFi.status() != WL_CONNECTED) {
    connectWiFi(10000);
    if (lastTempUpdateMs != 0) {
      displayDirty = true;
    }
  }

  // Keep MQTT alive
  if (!mqttClient.connected()) {
    ensureMqttConnected();
  }
  mqttClient.loop();

  // E-ink refresh policy (avoid too frequent refresh)
  const unsigned long now = millis();
  const unsigned long minRefreshMs = DISPLAY_MIN_REFRESH_MS;
  const unsigned long periodicRefreshMs = DISPLAY_PERIODIC_REFRESH_MS;

  // Don’t keep refreshing the screen while we’re still waiting for the first
  // sensor payload – e-ink flashing is noticeable and unnecessary.
  const bool hasAnyMqttData = (lastTempUpdateMs != 0);
  const bool periodicEnabled = (periodicRefreshMs > 0) && hasAnyMqttData;

  const bool timeForPeriodic = periodicEnabled && ((now - lastDisplayRefreshMs) >= periodicRefreshMs);
  const bool canRefreshNow = (now - lastDisplayRefreshMs) >= minRefreshMs;
  const bool refreshByDirty = hasAnyMqttData && displayDirty && canRefreshNow;
  const bool refreshByPeriodic = timeForPeriodic && canRefreshNow;

  if (refreshByDirty || refreshByPeriodic) {
    drawTemperatureScreen();
    lastDisplayRefreshMs = now;
    displayDirty = false;
    logAi("display_refresh", "reason=%s now_ms=%lu min_refresh_ms=%lu periodic_refresh_ms=%lu",
          refreshByDirty ? "dirty" : "periodic", (unsigned long)now,
          (unsigned long)minRefreshMs, (unsigned long)periodicRefreshMs);
  }

  maybeEnterDeepSleep(now);

  delay(10);
}
