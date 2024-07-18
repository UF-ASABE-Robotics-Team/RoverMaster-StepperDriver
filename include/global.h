// =============================================================================
// Global Variables and Function Declarations
// =============================================================================
// License: MIT
// Author: Yuxuan Zhang (zhangyuxuan@ufl.edu)
// =============================================================================
#pragma once

#include <Arduino.h> // IWYU pragma: export
#include <Vector.h>
#include <stdio.h> // IWYU pragma: export

#include <parser.h>
#include <scheduler.h>

template <typename T> int sgn(T val) { return (T(0) < val) - (val < T(0)); }

namespace global {

// extern HardwareSerial port;
// Enable state across all components
extern bool enabled;
// Flag to echo command back to upstream
extern bool echo;
// Motion signal, used for synchronization
extern bool sig_motion;
// Input buffer for parsing input string
extern char in_buf[128];
// Output buffer for formatting output string
extern char out_buf[128];
// Serial command handler
void handleSerialCommand(const Parser::Context *);
// List of all serial ports
extern Vector<Stream *> ports;
} // namespace global

#define printf(port, ...)                                                      \
  {                                                                            \
    snprintf(global::out_buf, sizeof(global::out_buf), __VA_ARGS__);           \
    (port).print(global::out_buf);                                             \
  }

#define cast_assign(var, val) var = static_cast<decltype(var)>(val)