#pragma once
#include <stdint.h>
#include <stdbool.h>

static bool tesla_preap_pedal_finite(float value) {
  union { float value; uint32_t bits; } word = {.value = value};
  return (word.bits & 0x7F800000U) != 0x7F800000U;
}
static float tesla_preap_pedal_abs(float value) { return (value < 0.0F) ? -value : value; }

// Fork-local startup USB envelope. No CAN packet configures these values.
#define TESLA_PREAP_PEDAL_CONFIG_WORDS 24U
// The startup capsule is shared by native mode admission and its USB owner.
typedef struct {
  uint16_t staged[24];
  uint16_t active_words[24];
  unsigned int next_word;
  bool staged_ready;
} TeslaPreapPedalStartupState;
static TeslaPreapPedalStartupState tesla_preap_pedal_startup = {0};
static bool tesla_preap_pedal_configured = false;
static float tesla_preap_pedal_factor = 0.0F;
static float tesla_preap_pedal_zero = 0.0F;
static unsigned int tesla_preap_pedal_bus = 0U;
static bool tesla_preap_pedal_feedback_seen = false;
static bool tesla_preap_pedal_feedback_healthy = false;
static uint32_t tesla_preap_pedal_feedback_ts = 0U;
static float tesla_preap_pedal_feedback_di = 0.0F;
static unsigned int tesla_preap_pedal_feedback_raw_one = 0U;
static unsigned int tesla_preap_pedal_feedback_raw_two = 0U;
static bool tesla_preap_pedal_tx_seen = false;
static bool tesla_preap_pedal_tx_enabled = false;
static unsigned int tesla_preap_pedal_last_raw_one = 0U;
static unsigned int tesla_preap_pedal_last_raw_two = 0U;
// Interceptor STATE of the last valid feedback frame, and the accepted 0x551 stream timing used to judge it.
static unsigned int tesla_preap_pedal_feedback_state = 15U;
static bool tesla_preap_pedal_tx_ts_seen = false;
static uint32_t tesla_preap_pedal_tx_last_ts = 0U;
static uint32_t tesla_preap_pedal_tx_stream_ts = 0U;

// 0x552 channel agreement. The two position sensors of an interceptor are not an exact 2:1 pair; on the
// retrofit this port was validated on (1,703,610 frames, 600 segments, pedal from rest to floor) the
// relationship is (one - 2*two) = 0.00844*one - 14.17 raw counts of channel 1, with every frame within
// 2.98 physical units of it (sd 0.86). The check allows 4.0 units (78.7 counts, 3.7 % of pedal travel)
// around that line, so an open, shorted or stuck channel on either side is still rejected.
#define TESLA_PREAP_PEDAL_AGREE_SLOPE 0.00844F
#define TESLA_PREAP_PEDAL_AGREE_OFFSET (-14.17F)
#define TESLA_PREAP_PEDAL_AGREE_TOLERANCE (4.0F / 0.050796813F)
// Both sensors read well above zero with the pedal released (lowest logged: 416 and about 210 counts). A pair
// reading near zero together (lost sensor supply) agrees with itself, so it needs its own floor.
#define TESLA_PREAP_PEDAL_MIN_RAW_ONE 300U
#define TESLA_PREAP_PEDAL_MIN_RAW_TWO 150U
// The interceptor reports STATE 5 (command timeout) while nothing is commanding it, and STATE 0 from
// 4-45 ms after the first 0x551 frame until 196-247 ms after the last one.
#define TESLA_PREAP_PEDAL_STATE_OK 0U
#define TESLA_PREAP_PEDAL_STATE_IDLE 5U
#define TESLA_PREAP_PEDAL_STATE_SETTLE_US 100000U
// BogPilot's absolute cap on an enabled command, kept on top of the calibrated bounds.
#define TESLA_PREAP_PEDAL_MAX_ENABLED_RAW 2500U

static float tesla_preap_pedal_word_float(const uint16_t *words) {
  union { uint32_t bits; float value; } decoded;
  decoded.bits = (uint32_t)words[0] | ((uint32_t)words[1] << 16U);
  return decoded.value;
}

static bool tesla_preap_pedal_config_valid(const uint16_t *words) {
  bool hash_present = false;
  for (unsigned int i = 8U; i < TESLA_PREAP_PEDAL_CONFIG_WORDS; i++) { hash_present |= words[i] != 0U; }
  const float factor = tesla_preap_pedal_word_float(&words[4]);
  const float zero = tesla_preap_pedal_word_float(&words[6]);
  bool valid = (words[0] == 1U) && ((words[1] == 0U) || (words[1] == 2U)) &&
    ((words[2] == 1U) || (words[2] == 3U) || (words[2] == 7U)) && (words[3] == 0xD051U) &&
    tesla_preap_pedal_finite(factor) && (tesla_preap_pedal_abs(factor) > 0.000001F) && tesla_preap_pedal_finite(zero) && hash_present;
  if (valid) {
    const float demands[2] = {-5.0F, 90.0F};
    for (unsigned int d = 0U; d < 2U; d++) {
      const float physical = zero + (demands[d] / factor) + 22.85856576F;
      const float raw_one = physical / 0.050796813F;
      const float raw_two = physical / 0.101593626F;
      const unsigned int one = (raw_one < 0.0F) ? 0U : ((raw_one > 65534.0F) ? 65534U : (unsigned int)raw_one);
      const unsigned int two = (raw_two < 0.0F) ? 0U : ((raw_two > 65534.0F) ? 65534U : (unsigned int)raw_two);
      const int difference = (int)one - (2 * (int)two);
      valid &= (difference >= 0) && (difference <= 1);
    }
  }
  return valid;
}

static void tesla_preap_pedal_mode_reset(uint16_t mode) {
  tesla_preap_pedal_configured = false;
  tesla_preap_pedal_feedback_seen = false;
  tesla_preap_pedal_feedback_healthy = false;
  tesla_preap_pedal_tx_seen = false;
  tesla_preap_pedal_tx_enabled = false;
  tesla_preap_pedal_last_raw_one = 0U;
  tesla_preap_pedal_last_raw_two = 0U;
  tesla_preap_pedal_feedback_state = 15U;
  tesla_preap_pedal_tx_ts_seen = false;
  if (mode != 39U) {
    tesla_preap_pedal_startup.staged_ready = false;
    tesla_preap_pedal_startup.next_word = 0U;
  }
}

static bool tesla_preap_pedal_profile_init(uint16_t param) {
  const bool good = tesla_preap_pedal_startup.staged_ready && (tesla_preap_pedal_startup.staged[2] == param) &&
    tesla_preap_pedal_config_valid(tesla_preap_pedal_startup.staged);
  tesla_preap_pedal_startup.staged_ready = false;  // a startup capsule is consumed exactly once
  if (good) {
    for (unsigned int i = 0U; i < TESLA_PREAP_PEDAL_CONFIG_WORDS; i++) {
      tesla_preap_pedal_startup.active_words[i] = tesla_preap_pedal_startup.staged[i];
    }
    tesla_preap_pedal_factor = tesla_preap_pedal_word_float(&tesla_preap_pedal_startup.active_words[4]);
    tesla_preap_pedal_zero = tesla_preap_pedal_word_float(&tesla_preap_pedal_startup.active_words[6]);
    tesla_preap_pedal_bus = tesla_preap_pedal_startup.active_words[1];
  }
  tesla_preap_pedal_configured = good;
  return good;
}

static float tesla_preap_pedal_di(unsigned int raw, float scale) {
  return (((float)raw * scale) - 22.85856576F - tesla_preap_pedal_zero) * tesla_preap_pedal_factor;
}

// STATE 5 is the interceptor's normal state only while it is not being commanded: never after an enabled
// command, and not once a 0x551 stream has been running long enough for it to have left the timeout.
static bool tesla_preap_pedal_state_valid(unsigned int state, uint32_t now) {
  bool valid = state == TESLA_PREAP_PEDAL_STATE_OK;
  if (state == TESLA_PREAP_PEDAL_STATE_IDLE) {
    const bool commanding = tesla_preap_pedal_tx_ts_seen &&
      ((uint32_t)(now - tesla_preap_pedal_tx_last_ts) <= TESLA_PREAP_PEDAL_STATE_SETTLE_US) &&
      ((uint32_t)(now - tesla_preap_pedal_tx_stream_ts) > TESLA_PREAP_PEDAL_STATE_SETTLE_US);
    valid = !tesla_preap_pedal_tx_enabled && !commanding;
  }
  return valid;
}

static bool tesla_preap_pedal_feedback(const uint8_t *data, unsigned int bus, uint32_t now) {
  bool next = false;
  if (tesla_preap_pedal_configured && (bus == tesla_preap_pedal_bus)) {
    static uint8_t tesla_preap_pedal_feedback_counter = 0U;
    const unsigned int one = ((unsigned int)data[0] << 8U) | data[1];
    const unsigned int two = ((unsigned int)data[2] << 8U) | data[3];
    const unsigned int state = data[4] >> 4U;
    const uint8_t counter = data[4] & 15U;
    unsigned int sum = 0x52U + 5U;
    for (unsigned int i = 0U; i < 5U; i++) { sum += data[i]; }
    const bool checksum = (sum & 255U) == data[5];
    const bool monotonic = !tesla_preap_pedal_feedback_seen || ((uint32_t)(now - tesla_preap_pedal_feedback_ts) > 0U);
    const float expected = (TESLA_PREAP_PEDAL_AGREE_SLOPE * (float)one) + TESLA_PREAP_PEDAL_AGREE_OFFSET;
    const float difference = (float)one - (2.0F * (float)two);
    const bool channels = (one < 65535U) && (two < 65535U) &&
      (one >= TESLA_PREAP_PEDAL_MIN_RAW_ONE) && (two >= TESLA_PREAP_PEDAL_MIN_RAW_TWO) &&
      (tesla_preap_pedal_abs(difference - expected) <= TESLA_PREAP_PEDAL_AGREE_TOLERANCE);
    const bool valid = checksum && tesla_preap_pedal_state_valid(state, now) && channels && monotonic;
    // The interceptor drops a single frame now and then (135 times in 1.7 M frames, never two in a row),
    // so the counter may advance by 1 or 2. A repeated, stalled or jumping counter is a fault.
    const uint8_t advance = (uint8_t)((counter - tesla_preap_pedal_feedback_counter) & 15U);
    next = tesla_preap_pedal_feedback_seen && valid && ((advance == 1U) || (advance == 2U));
    tesla_preap_pedal_feedback_healthy = next;
    tesla_preap_pedal_feedback_seen = valid;
    tesla_preap_pedal_feedback_counter = counter;
    tesla_preap_pedal_feedback_ts = now;
    tesla_preap_pedal_feedback_state = valid ? state : 15U;
    tesla_preap_pedal_feedback_raw_one = one;
    tesla_preap_pedal_feedback_raw_two = two;
    // Driver demand is channel 1, as in both references. Channel 2 is the agreement backstop above.
    tesla_preap_pedal_feedback_di = tesla_preap_pedal_di(one, 0.050796813F);
  }
  return next;
}

static bool tesla_preap_pedal_feedback_ready(uint32_t now) {
  return tesla_preap_pedal_configured && tesla_preap_pedal_feedback_healthy &&
    ((uint32_t)(now - tesla_preap_pedal_feedback_ts) <= 500000U);
}

static float tesla_preap_pedal_max_di(float speed) {
  const float bp[6] = {0.0F, 5.0F, 12.0F, 20.0F, 30.0F, 40.0F};
  const float values[6] = {50.0F, 58.0F, 66.0F, 74.0F, 82.0F, 90.0F};
  float result = values[5];
  for (unsigned int i = 1U; i < 6U; i++) {
    if (speed <= bp[i]) {
      const float bounded = (speed < 0.0F) ? 0.0F : speed;
      result = values[i - 1U] + ((bounded - bp[i - 1U]) / (bp[i] - bp[i - 1U])) * (values[i] - values[i - 1U]);
      break;
    }
  }
  return result;
}

static bool tesla_preap_pedal_channel_bounds(unsigned int raw, unsigned int previous, float scale, float speed) {
  const float first = (tesla_preap_pedal_zero + (-5.0F / tesla_preap_pedal_factor) + 22.85856576F) / scale;
  const float second = (tesla_preap_pedal_zero + (tesla_preap_pedal_max_di(speed) / tesla_preap_pedal_factor) + 22.85856576F) / scale;
  // Match the working Dom codec's0..65534 clamp, including a valid floor
  // saturation. Config admission separately rejects incompatible channel tops.
  const float bounded_first = (first < 0.0F) ? 0.0F : ((first > 65534.0F) ? 65534.0F : first);
  const float bounded_second = (second < 0.0F) ? 0.0F : ((second > 65534.0F) ? 65534.0F : second);
  const float low = (bounded_first < bounded_second) ? bounded_first : bounded_second;
  const float high = (bounded_first > bounded_second) ? bounded_first : bounded_second;
  // One ADC count covers Float32-vs-host quotient rounding without a DI-based
  // tolerance growing with calibration factor.
  const unsigned int floor_low = (unsigned int)low;
  const unsigned int lower = (floor_low > 0U) ? (floor_low - 1U) : 0U;
  const unsigned int upper = ((unsigned int)high < 65534U) ? ((unsigned int)high + 1U) : 65534U;
  const float unit = tesla_preap_pedal_abs(tesla_preap_pedal_factor) * scale;
  const float up = 5.0F / unit;
  const float down = 2.5F / unit;
  const unsigned int rate_up = (up >= 65534.0F) ? 65534U : ((unsigned int)up + 1U);
  const unsigned int rate_down = (down >= 65534.0F) ? 65534U : ((unsigned int)down + 1U);
  const int delta = (tesla_preap_pedal_factor > 0.0F) ? ((int)raw - (int)previous) : ((int)previous - (int)raw);
  return (raw >= lower) && (raw <= upper) && (delta <= (int)rate_up) && (delta >= -(int)rate_down);
}

static bool tesla_preap_pedal_tx(const uint8_t *data, unsigned int bus, uint32_t now,
                                 float speed, bool longitudinal_allowed) {
  bool good = false;
  if (tesla_preap_pedal_configured && (bus == tesla_preap_pedal_bus)) {
    static uint8_t tesla_preap_pedal_tx_counter = 0U;
    const unsigned int one = ((unsigned int)data[0] << 8U) | data[1];
    const unsigned int two = ((unsigned int)data[2] << 8U) | data[3];
    unsigned int sum = 0x51U + 5U;
    for (unsigned int i = 0U; i < 5U; i++) { sum += data[i]; }
    const bool enabled = (data[4] & 128U) != 0U;
    const uint8_t counter = data[4] & 15U;
    good = ((data[4] & 112U) == 0U) && ((sum & 255U) == data[5]) &&
      (!tesla_preap_pedal_tx_seen || (counter == ((tesla_preap_pedal_tx_counter + 1U) & 15U)));
    if (!enabled) {
      // A valid zero/disabled command may resynchronize a dropped TX counter.
      good = ((data[4] & 112U) == 0U) && ((sum & 255U) == data[5]) && (one == 0U) && (two == 0U);
    } else {
      const int difference = (int)one - (2 * (int)two);
      const unsigned int previous = tesla_preap_pedal_tx_enabled ? tesla_preap_pedal_last_raw_one : tesla_preap_pedal_feedback_raw_one;
      const unsigned int previous_two = tesla_preap_pedal_tx_enabled ? tesla_preap_pedal_last_raw_two : tesla_preap_pedal_feedback_raw_two;
      // An enabled command needs the interceptor already out of its idle timeout (a disabled frame first).
      good &= tesla_preap_pedal_tx_seen && longitudinal_allowed && tesla_preap_pedal_feedback_ready(now) &&
        (tesla_preap_pedal_feedback_state == TESLA_PREAP_PEDAL_STATE_OK) && (one <= TESLA_PREAP_PEDAL_MAX_ENABLED_RAW) &&
        (tesla_preap_pedal_feedback_di <= 2.0F) && tesla_preap_pedal_finite(speed) && (speed >= 0.0F) &&
        (difference >= 0) && (difference <= 1) && (one < 65535U) && (two < 65535U) &&
        tesla_preap_pedal_channel_bounds(one, previous, 0.050796813F, speed) &&
        tesla_preap_pedal_channel_bounds(two, previous_two, 0.101593626F, speed);
      if (good) { tesla_preap_pedal_last_raw_one = one; tesla_preap_pedal_last_raw_two = two; }
    }
    if (good) {
      tesla_preap_pedal_tx_seen = true;
      tesla_preap_pedal_tx_counter = counter;
      tesla_preap_pedal_tx_enabled = enabled;
      if (!tesla_preap_pedal_tx_ts_seen || ((uint32_t)(now - tesla_preap_pedal_tx_last_ts) > TESLA_PREAP_PEDAL_STATE_SETTLE_US)) {
        tesla_preap_pedal_tx_stream_ts = now;
      }
      tesla_preap_pedal_tx_last_ts = now;
      tesla_preap_pedal_tx_ts_seen = true;
    }
    // Rejected TX does not alter the last accepted rate/counter baseline.
  }
  return good;
}
