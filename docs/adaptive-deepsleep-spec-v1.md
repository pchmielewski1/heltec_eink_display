# Adaptive Deep Sleep Spec v1

## Scope

This document defines implementation v1 of adaptive deep sleep for the Heltec Wireless Paper firmware.

Target outcome:
- reduce battery drain by minimizing active idle time,
- keep dashboard freshness acceptable,
- remain robust when broker cadence changes.

Non-goals (v1):
- no broker-side changes required,
- no dependency on retained messages,
- no machine-learning framework.

## Constraints and assumptions

- MQTT topic may include mixed message types (`telemetry`, `nodeinfo`, mapped weather payloads).
- Broker may not retain latest values.
- Missed messages during sleep are not replayed (unless retained or persistent session is added later).
- Device must wake before expected message window and listen for a bounded time.
- Cadence learning is meaningful only when sensor data arrives at approximately regular intervals.

## Regularity requirement and retained override mode

Adaptive cadence learning should be used only when the sensor publishes with a stable interval.

If sensor traffic is event-driven (for example data appears only when temperature changes),
cadence learning may never converge. In that case, firmware must support a broker-driven override:

- a dedicated **retained control flag** topic,
- when flag indicates retained mode, firmware stops learning and switches to fixed 15-minute wake polling,
- on each wake, firmware reconnects and reads latest retained state from broker,
- when the flag disappears (or is explicitly disabled), firmware resets cadence model and starts learning from scratch.

### Control flag contract (recommended)

- topic example: `device/<id>/mode/deepsleep`
- retained payload values:
   - `retain_poll_15m` -> force retained polling mode
   - `adaptive` -> force adaptive learning mode
   - empty/deleted retained message -> treat as `adaptive` and restart learning

The exact topic path is configurable; semantics above are required.

## Event classes used for cadence learning

From existing runtime behavior, classify incoming messages into:

1. `mapped_update`
   - message updates at least one mapped field: temperature, humidity, pressure.
   - source event in logs: `AI: event=data_update ...`

2. `telemetry_noop`
   - valid message from tracked topic but no mapped fields.
   - source event in logs: `AI: event=data_noop ...`

3. optional `post_update_burst` (derived)
   - no-op events shortly after `mapped_update` (nodeinfo/air-util bursts).
   - used only as a suppression hint, not a primary sleep anchor.

Primary anchor for sleep in v1: `mapped_update` cadence.

## Data model (RTC-persisted)

Persist in RTC memory so values survive deep sleep resets.

```c
struct CadenceModel {
  uint32_t lastMappedTs;            // epoch seconds from payload timestamp
  uint32_t mappedIntervals[8];      // ring buffer, seconds
  uint8_t mappedCount;              // valid intervals in ring
  uint8_t mappedHead;               // next write index

  uint32_t medianIntervalSec;       // robust center estimate
  uint32_t madSec;                  // median absolute deviation

  uint8_t confidence;               // 0..100
  uint8_t missedWindows;            // consecutive misses

  uint32_t lastPlanSleepSec;        // last planned sleep duration
  uint32_t lastWakeEpoch;           // best-effort wallclock checkpoint

   uint8_t mode;                     // 0=adaptive_learning, 1=retain_poll_15m
   uint8_t modeSource;               // 0=local_default, 1=broker_flag

  uint32_t crc32;                   // integrity guard
  uint8_t version;                  // model schema version
};
```

If CRC/version invalid: reset model and start conservative mode.

## Timing constants (v1 defaults)

```c
MIN_TRAINING_SAMPLES            = 2
MIN_SLEEP_SEC                   = 60
MAX_SLEEP_SEC                   = 30 * 60
DEFAULT_FALLBACK_WAKE_SEC       = 5 * 60
RECONNECT_BUDGET_SEC            = 20
LISTEN_WINDOW_SEC               = 90
SAFETY_FLOOR_SEC                = 45
MAD_MULTIPLIER                  = 3
CONFIDENCE_INC_HIT              = 8
CONFIDENCE_DEC_MISS             = 15
CONFIDENCE_MIN_FOR_ADAPTIVE     = 45
MAX_CONSECUTIVE_MISSES          = 3
POST_UPDATE_BURST_SUPPRESS_SEC  = 60
RETAIN_POLL_INTERVAL_SEC        = 15 * 60
```

Field-tested low-frequency profile (optional, deployment-specific):

```c
// For streams where mapped updates arrive roughly every 30-45 minutes:
MAX_SLEEP_SEC                   = 90 * 60
DEFAULT_FALLBACK_WAKE_SEC       = 30 * 60
CONFIDENCE_MIN_FOR_ADAPTIVE     = 16
```

This profile reduces fallback loops in slow streams while preserving safety bounds.

## Robust interval estimation

When a new `mapped_update` arrives with valid payload timestamp:
1. `interval = ts_now - lastMappedTs`.
2. Accept interval only if `30s <= interval <= 6h`.
3. Push into ring buffer.
4. Compute `medianIntervalSec` from valid intervals.
5. Compute `madSec = median(|interval_i - medianIntervalSec|)`.

Why median/MAD:
- robust against outliers,
- stable for mixed traffic.

## Sleep planning algorithm (v1)

Run planning after successful processing cycle and before entering deep sleep.

Pseudo logic:

```c
if (mode == retain_poll_15m) {
   sleepSec = RETAIN_POLL_INTERVAL_SEC;
   reason = "retain_poll_15m";
} else {
   if (mappedCount < MIN_TRAINING_SAMPLES || confidence < CONFIDENCE_MIN_FOR_ADAPTIVE) {
      sleepSec = DEFAULT_FALLBACK_WAKE_SEC;
      reason = "fallback_training";
   } else {
      guardSec = max(SAFETY_FLOOR_SEC, MAD_MULTIPLIER * madSec + RECONNECT_BUDGET_SEC);
      targetWakeDelta = max((int32_t)medianIntervalSec - (int32_t)guardSec, (int32_t)MIN_SLEEP_SEC);
      sleepSec = clamp(targetWakeDelta, MIN_SLEEP_SEC, MAX_SLEEP_SEC);
      reason = "adaptive_median";
   }
}
```

Burst suppression:
- if a no-op message arrives within `POST_UPDATE_BURST_SUPPRESS_SEC` after mapped update,
  do not re-plan a short sleep based on that no-op.

## Wake cycle behavior

On wake:
1. Bring up Wi-Fi and MQTT.
2. Read retained mode flag (if topic configured).
3. If mode is `retain_poll_15m`:
   - skip cadence learning,
   - consume latest retained telemetry snapshot,
   - sleep for fixed 15 minutes.
4. If mode is adaptive:
   - listen for up to `LISTEN_WINDOW_SEC`,
   - if `mapped_update` appears: update model, increase confidence, render, plan next sleep,
   - if no `mapped_update`: mark miss and fallback as needed.

Mode transition rules:
- adaptive -> retain_poll_15m: freeze learning state, stop confidence updates.
- retain_poll_15m -> adaptive (or flag deleted): clear cadence intervals/confidence and retrain from zero.

## Confidence update

- Hit in expected window: `confidence = min(100, confidence + CONFIDENCE_INC_HIT)`
- Missed expected window: `confidence = max(0, confidence - CONFIDENCE_DEC_MISS)`
- If `missedWindows >= MAX_CONSECUTIVE_MISSES`, force fallback until 2 consecutive hits.

## Integration points in firmware

- `mqttCallback(...)`
  - on `data_update`: call `cadenceOnMappedUpdate(timestamp)`.
  - on `data_noop`: call `cadenceOnNoop(timestamp)` (lightweight, optional).

- main control path (`setup/loop` split for deep-sleep mode)
  - after listen window completion: call `planNextSleep()`.
  - call `esp_sleep_enable_timer_wakeup(sleepSec * 1e6)` and `esp_deep_sleep_start()`.

- boot logging
  - include wake cause and model status summary.

## AI log additions required

Keep existing log format and add these events:

- `event=sleep_plan mode=fallback|adaptive reason=... sleep_sec=... median_sec=... mad_sec=... confidence=...`
- `event=sleep_enter sleep_sec=...`
- `event=sleep_wake cause=...`
- `event=cadence_update interval_sec=... median_sec=... mad_sec=... samples=...`
- `event=cadence_miss misses=... confidence=...`
- `event=mode_flag value=adaptive|retain_poll_15m source=broker|default`
- `event=mode_switch from=... to=... action=reset_learning|keep_state`

These logs are mandatory for evaluating model quality over time.

## Safety rules

- Never sleep if firmware is in explicit developer override mode (future config flag).
- Cap all computed times to sane min/max.
- Ignore payload timestamps that move backward strongly or jump unrealistically.
- If RTC model is corrupt, reset model and continue in fallback mode.

## Configuration additions (planned)

Add optional keys to `secrets.h` / `secrets.h.example`:

```c
#define DEEPSLEEP_ENABLE 0
#define DEEPSLEEP_LEARN_ENABLE 1
#define DEEPSLEEP_MIN_SEC 60
#define DEEPSLEEP_MAX_SEC 1800
#define DEEPSLEEP_LISTEN_WINDOW_SEC 90
#define DEEPSLEEP_FALLBACK_SEC 300
#define DEEPSLEEP_CONFIDENCE_MIN 45
#define DEEPSLEEP_MIN_INTERVAL_SAMPLES 2
#define DEEPSLEEP_LEARN_MIN_INTERVAL_SEC 120
#define DEEPSLEEP_RETAIN_MODE_TOPIC ""
#define DEEPSLEEP_RETAIN_POLL_SEC 900
#define DEEPSLEEP_MIN_AWAKE_SEC 30
#define DEEPSLEEP_MAX_EMPTY_SLEEP_CYCLES 12
```

Defaults keep current behavior when deep sleep is disabled.

## Acceptance criteria (v1)

Functional:
- Device can run at least 24h without lockup in adaptive mode.
- Deep sleep only activates when `DEEPSLEEP_ENABLE=1`.
- With stable cadence, median-based plan converges within first 3–5 mapped updates.
- With retained override flag enabled, device switches to fixed 15-minute polling mode.
- When retained override flag is removed, device resets model and retrains from scratch.

Quality:
- At least 70% of mapped updates are caught inside planned wake windows in stable conditions.
- Battery runtime improves versus always-on baseline.
- AI logs are sufficient to reconstruct sleep decisions post-factum.

## Validation plan

1. Baseline (always-on): collect 24h logs and power profile.
2. Adaptive mode: collect 24h logs with same topic.
3. Compare:
   - mapped update capture rate,
   - average active time per hour,
   - battery drop slope.
4. Stress cases:
   - cadence shift,
   - missing packets,
   - Wi-Fi reconnect delays,
   - broker restart.

## Future extension candidates (v2+)

- dual-model prediction (`mapped_update` + `telemetry_noop`) with priority arbitration,
- optional retained mirror topic for fast state recovery on wake,
- confidence-weighted dynamic listen window,
- broker-specific profile cache keyed by topic/device.
