#pragma once
#include <stdio.h> // IWYU pragma: export

template <typename T> int sgn(T val) { return (T(0) < val) - (val < T(0)); }

namespace global {

// Motion signal, used for synchronization
extern bool sig_motion;
// Input buffer for parsing input string
extern char in_buf[128];
// Output buffer for formating output string
extern char out_buf[128];

} // namespace global

#define printf(...)                                                            \
  {                                                                            \
    sprintf(global::out_buf, __VA_ARGS__);                                     \
    Serial.print(global::out_buf);                                             \
  }
