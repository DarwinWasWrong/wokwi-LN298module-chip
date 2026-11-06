// L298N dual H-bridge - Wokwi custom chip
//
// Pins (must match l298n.chip.json):
//   ENA IN1 IN2 M1 M2  ENA_5v   (channel A)
//   ENB IN3 IN4 M3 M4  ENB_5v   (channel B)
//   VCC GND Motor_V
//
// Behaviour (matches the real module):
//   EN low / 0 % duty      -> both outputs 0 V
//   IN1=H IN2=L            -> M1 = V * duty, M2 = 0   (forward)
//   IN1=L IN2=H            -> M1 = 0, M2 = V * duty   (reverse)
//   IN1 == IN2             -> both outputs 0 V        (stop/brake)
//
// ENA can be: jumpered to ENA_5v, held HIGH/LOW with digitalWrite,
// or driven with PWM (analogWrite / ledc). A steady level is detected by
// a watchdog timer (no edges for PWM_TIMEOUT_NS) and treated as 100 % / 0 %.
//
// Attribute "supplyVoltage" (default 12 V) is the motor supply.
// Attribute "voltageDrop"   (default 2 V) is the loss across the bridge.
// Output at 100 % duty = supplyVoltage - voltageDrop (never below 0).
// Set them in diagram.json  "attrs": { "supplyVoltage": "9", "voltageDrop": "2" }
// or add a live slider in chip.json "controls" (see bottom of this file).

#include "wokwi-api.h"
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <stdint.h>

#define DEFAULT_SUPPLY_V 12.0f        // motor supply voltage
#define DEFAULT_DROP_V    2.0f        // typical L298N bridge loss
#define WATCHDOG_US     10000         // watchdog period: 10 ms
#define PWM_TIMEOUT_NS  50000000ULL   // no EN edge for 50 ms => steady level

typedef struct {
  const char *name;
  pin_t en, in_a, in_b, out_a, out_b;

  bool     have_rise, have_fall;
  uint64_t last_rise, last_fall, last_edge;
  uint64_t high_ns, low_ns;

  uint32_t supply_attr;  // "supplyVoltage" attribute handle
  uint32_t drop_attr;    // "voltageDrop" attribute handle

  int   duty;       // 0..100 %
  int   last_duty;  // last value written to the outputs
  float last_v;     // last supply voltage used
  int last_dir;   // 1 fwd, -1 rev, 0 stop, -2 = nothing written yet
} channel_t;

typedef struct {
  channel_t a, b;
  pin_t ena_5v, enb_5v;
  pin_t vcc, gnd, motor_v;
  timer_t watchdog;
} chip_state_t;

// ---------------------------------------------------------------------------

static void update_outputs(channel_t *ch) {
  bool in_a = pin_read(ch->in_a);
  bool in_b = pin_read(ch->in_b);

  int dir = 0;
  if (ch->duty > 0 && in_a != in_b) dir = in_a ? 1 : -1;

  float drop   = attr_read_float(ch->drop_attr);
  if (drop < 0) drop = 0;
  float supply = attr_read_float(ch->supply_attr) - drop;   // usable voltage
  if (supply < 0) supply = 0;

  if (dir == ch->last_dir && ch->duty == ch->last_duty && supply == ch->last_v)
    return;  // nothing changed

  float v = supply * (float)ch->duty / 100.0f;
  pin_dac_write(ch->out_a, dir ==  1 ? v : 0.0f);
  pin_dac_write(ch->out_b, dir == -1 ? v : 0.0f);

  printf("L298N %s: %s %d%% (%.2f V)\n", ch->name,
         dir == 1 ? "forward" : dir == -1 ? "reverse" : "stop",
         ch->duty, dir ? v : 0.0f);

  ch->last_dir  = dir;
  ch->last_duty = ch->duty;
  ch->last_v    = supply;
}

// EN pin edge: measure PWM high/low times
static void on_enable_change(void *user_data, pin_t pin, uint32_t value) {
  channel_t *ch = (channel_t *)user_data;
  uint64_t now = get_sim_nanos();

  if (value) {
    if (ch->have_fall) ch->low_ns = now - ch->last_fall;
    ch->last_rise = now;
    ch->have_rise = true;
  } else {
    if (ch->have_rise) ch->high_ns = now - ch->last_rise;
    ch->last_fall = now;
    ch->have_fall = true;
  }
  ch->last_edge = now;

  // need one full high + low period before the duty is valid
  if (ch->high_ns && ch->low_ns) {
    uint64_t total = ch->high_ns + ch->low_ns;
    ch->duty = (int)((ch->high_ns * 100 + total / 2) / total);
    update_outputs(ch);
  }
}

// IN pin change: direction may have changed
static void on_input_change(void *user_data, pin_t pin, uint32_t value) {
  update_outputs((channel_t *)user_data);
}

// EN has not toggled for a while -> it is a steady level (jumper / digitalWrite)
static void check_steady(channel_t *ch, uint64_t now, bool force) {
  if (!force && now - ch->last_edge < PWM_TIMEOUT_NS) {
    update_outputs(ch);   // still PWM - just pick up any supply voltage change
    return;
  }
  ch->duty = pin_read(ch->en) ? 100 : 0;
  ch->high_ns = ch->low_ns = 0;
  ch->have_rise = ch->have_fall = false;
  update_outputs(ch);
}

static void on_watchdog(void *user_data) {
  chip_state_t *chip = (chip_state_t *)user_data;
  uint64_t now = get_sim_nanos();
  check_steady(&chip->a, now, false);
  check_steady(&chip->b, now, false);
}

// ---------------------------------------------------------------------------

static void channel_init(channel_t *ch, const char *name,
                         uint32_t supply_attr, uint32_t drop_attr,
                         const char *en, const char *in_a, const char *in_b,
                         const char *out_a, const char *out_b) {
  ch->name  = name;
  ch->en    = pin_init(en,    INPUT);
  ch->in_a  = pin_init(in_a,  INPUT);
  ch->in_b  = pin_init(in_b,  INPUT);
  ch->out_a = pin_init(out_a, ANALOG);
  ch->out_b = pin_init(out_b, ANALOG);
  ch->supply_attr = supply_attr;
  ch->drop_attr   = drop_attr;
  ch->last_dir = -2;   // force first write

  const pin_watch_config_t en_watch = {
    .edge = BOTH,
    .pin_change = on_enable_change,
    .user_data = ch,
  };
  pin_watch(ch->en, &en_watch);

  const pin_watch_config_t in_watch = {
    .edge = BOTH,
    .pin_change = on_input_change,
    .user_data = ch,
  };
  pin_watch(ch->in_a, &in_watch);
  pin_watch(ch->in_b, &in_watch);
}

void chip_init(void) {
  printf("*** L298N chip initialising...\n");
  chip_state_t *chip = calloc(1, sizeof(chip_state_t));   // zeroed state

  // 5 V pins for the enable jumpers
  chip->ena_5v = pin_init("ENA_5v", OUTPUT_HIGH);
  chip->enb_5v = pin_init("ENB_5v", OUTPUT_HIGH);

  chip->vcc     = pin_init("VCC",     INPUT);
  chip->gnd     = pin_init("GND",     INPUT);
  chip->motor_v = pin_init("Motor_V", INPUT);

  uint32_t supply = attr_init_float("supplyVoltage", DEFAULT_SUPPLY_V);
  uint32_t drop   = attr_init_float("voltageDrop",   DEFAULT_DROP_V);
  printf("L298N supply %.1f V, drop %.1f V\n",
         attr_read_float(supply), attr_read_float(drop));

  channel_init(&chip->a, "A", supply, drop, "ENA", "IN1", "IN2", "M1", "M2");
  channel_init(&chip->b, "B", supply, drop, "ENB", "IN3", "IN4", "M3", "M4");

  // initial state (picks up the jumpers straight away)
  check_steady(&chip->a, 0, true);
  check_steady(&chip->b, 0, true);

  const timer_config_t wd_config = {
    .callback = on_watchdog,
    .user_data = chip,
  };
  chip->watchdog = timer_init(&wd_config);
  timer_start(chip->watchdog, WATCHDOG_US, true);
}

/* ---- l298n.chip.json : optional live slider -------------------------------
  "controls": [
    {
      "id": "supplyVoltage",
      "label": "Supply voltage (V)",
      "type": "range",
      "min": 0,
      "max": 35,
      "step": 0.5
    },
    {
      "id": "voltageDrop",
      "label": "Bridge voltage drop (V)",
      "type": "range",
      "min": 0,
      "max": 4,
      "step": 0.1
    }
  ]
---------------------------------------------------------------------------- */
