// =============================================================================
// Arduino Nano Stepper Controller Physical Connections
// =============================================================================
// License: MIT
// Author: Yuxuan Zhang (zhangyuxuan@ufl.edu)
// =============================================================================
#pragma once
#include <pins_arduino.h>

namespace PIN {
#if defined(BOARD_A)
// USB port for control commands.
#define SERIAL_UPSTREAM Serial
// UART port for communication with slave controller (Board B).
#define SERIAL_DOWNSTREAM Serial1
// UART port for communication with TMC2209 stepper driver.
#define SERIAL_TMC2209 Serial2
// Pin definition
enum {
  // Stepper Motor Control
  STEPPER_ENABLE = D12,
  // TMC2209 UART
  TMC_SERIAL_RX = D8,
  TMC_SERIAL_TX = D9,
};
#elif defined(BOARD_B)
// USB port for control commands.
#define SERIAL_UPSTREAM Serial0
// UART port for communication with TMC2209 stepper driver.
#define SERIAL_TMC2209 Serial2
    // Pin definition
    enum {
      // Stepper Motor Control
      STEPPER_ENABLE = D13,
      // TMC2209 UART
      TMC_SERIAL_RX = A0,
      TMC_SERIAL_TX = A1,
    };
#else
#error "Board type not defined"
#endif
} // namespace PIN
