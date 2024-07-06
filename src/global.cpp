// =============================================================================
// Global Variables and Function Definitions
// =============================================================================
// License: MIT
// Author: Yuxuan Zhang (zhangyuxuan@ufl.edu)
// =============================================================================
#include <global.h>

#define NONE -1

namespace global {

// HardwareSerial port(Serial);

bool sig_motion = false;
bool enabled = false;
char in_buf[128];
char out_buf[128];

Stream *port_list[16];
Vector<Stream *> ports(port_list, 0);

} // namespace global
