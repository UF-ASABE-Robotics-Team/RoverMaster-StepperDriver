// =============================================================================
// Serial Command Handler
// =============================================================================
// License: MIT
// Author: Yuxuan Zhang (zhangyuxuan@ufl.edu)
// =============================================================================
#include <board_pinout.h>
#include <components.h>
#include <global.h>
#include <parser.h>

static inline void error_unknown_joint(const Parser::Context &ctx) {
  printf(ctx.port, "ERROR Executing Command %s: Unknown Joint %s\n", ctx.cmd,
         ctx.arg);
}

void global::handleSerialCommand(const Parser::Context *c) {
  static bool enabled = false;
  const auto &ctx = *c;
  // Commands that can be executed when disabled
  if (strcmp(ctx.cmd, "ENABLE") == 0) {
    digitalWrite(PIN_STEPPER_ENABLE, LOW);
    enabled = true;
    printf(ctx.port, "INFO  SYSTEM ENABLED\n");
  } else if (strcmp(ctx.cmd, "SPEED") == 0) {
    for (auto s : Components::steppers)
      if (strcmp(ctx.arg, s->name) == 0) {
        char *err;
        double val = strtod(ctx.val, &err);
        if (*err != 0) {
          printf(ctx.port,
                 "ERROR Parsing Command SPEED for Joint %s: Invalid Value. "
                 "%s\n",
                 s->name, err);
        } else if (val < 0) {
          printf(ctx.port,
                 "ERROR Parsing Command SPEED for Joint %s: Negative Value "
                 "(%f)\n",
                 s->name, val);
        } else {
          s->set_speed(val);
        }
        return;
      }
    // error_unknown_joint(*ctx);
  } else if (!enabled) {
    printf(ctx.port, "ERROR SYSTEM DISABLED.\n");
    return;
  } else if (strcmp(ctx.cmd, "DISABLE") == 0) {
    digitalWrite(PIN_STEPPER_ENABLE, HIGH);
    enabled = false;
    printf(ctx.port, "INFO  SYSTEM DISABLED\n");
  } else if (strcmp(ctx.cmd, "HOME") == 0) {
    for (auto s : Components::steppers)
      if (strcmp(ctx.arg, s->name) == 0) {
        s->home(0);
        return;
      }
    // error_unknown_joint(*ctx);
  } else if (strcmp(ctx.cmd, "MOVE") == 0) {
    for (auto s : Components::steppers)
      if (strcmp(ctx.arg, s->name) == 0) {
        char *err;
        double val = strtod(ctx.val, &err);
        if (*err != 0) {
          printf(ctx.port,
                 "ERROR Parsing Command SPEED for Joint %s: Invalid Value. "
                 "%s\n",
                 s->name, err);
        } else {
          s->move(val);
        }
        return;
      }
    // error_unknown_joint(*ctx);
  } else {
    printf(ctx.port, "ERROR Unknown Command");
  }
}