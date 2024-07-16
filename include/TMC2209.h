// =============================================================================
// TMC2209 driver configuration
// =============================================================================
// License: MIT
// Author: Yuxuan Zhang (zhangyuxuan@ufl.edu)
// =============================================================================
#pragma once
#include <Arduino.h>
#include <hook.h>
#include <stdint.h>

namespace TMC2209 {

typedef struct Config_s {
  Stream *port;
  // Address of the driver, defined by the MS1 and MS2 pins
  uint8_t addr;
  // R_SENSE resistor value in mOhms, typically 0.11Ohms
  // Refer to datasheet for the exact value
  float R_SENSE;
  // Root mean square (RMS) current limit in mA
  uint16_t rms_current;
  // Micro-step resolution
  uint16_t micro_steps;
  // StallGuard threshold, 0-255
  // The higher the value, the more sensitive the stall detection
  uint8_t stall_sensitivity;
} Config;

Hook hook_init(Config *config);

} // namespace TMC2209