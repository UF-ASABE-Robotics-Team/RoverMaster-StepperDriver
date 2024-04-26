// =============================================================================
// Arduino Nano Stepper Controller Physical Connections
// =============================================================================
// License: MIT
// Author: Yuxuan Zhang (zhangyuxuan@ufl.edu)
// =============================================================================
#ifndef BOARD_PINOUT_H
#define BOARD_PINOUT_H

#include <Arduino.h>
#include <pins_arduino.h>

#define PIN_STEPPER_ENABLE 8

// Joint 1 Stepper Driver
#define PIN_J1_DIR 2
#define PIN_J1_STEP 5

// Joint 1 Position limit switches
#define PIN_J1_SW1 12
#define PIN_J1_SW2 9

// Joint 2 Servo Pin
#define PIN_J2_SERVO 13

#endif
