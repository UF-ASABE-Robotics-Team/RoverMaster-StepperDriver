// =============================================================================
// Serial Command Handler
// =============================================================================
// License: MIT
// Author: Yuxuan Zhang (zhangyuxuan@ufl.edu)
// =============================================================================
#include <components.h>
#include <global.h>
#include <parser.h>
#include <pinout.h>

#define STR_EQ(a, b) (strcmp(a, b) == 0)

void global::handleSerialCommand(const Parser::Context *c) {
  const auto &ctx = *c;
  // Commands that can be executed when disabled
  if (STR_EQ(ctx.cmd, "ENABLE")) {
    digitalWrite(PIN::STEPPER_ENABLE, LOW);
    for (auto s : Components::steppers)
      s->init();
    global::enabled = true;
    printf(SERIAL_UPSTREAM, "INFO  SYSTEM ENABLED\n");
  } else if (STR_EQ(ctx.cmd, "ECHO")) {
#ifdef BOARD_A
    if (STR_EQ(ctx.arg, "ON")) {
      global::echo = true;
      printf(SERIAL_UPSTREAM, "INFO  ECHO ON\n");
    } else if (STR_EQ(ctx.arg, "OFF")) {
      global::echo = false;
    } else
      printf(SERIAL_UPSTREAM, "ERROR Unknown Argument for ECHO: %s\n", ctx.arg);
#endif
  } else if (STR_EQ(ctx.cmd, "SPEED")) {
    for (auto s : Components::steppers)
      if (STR_EQ(ctx.arg, s->name)) {
        char *err;
        double val = strtod(ctx.val, &err);
        if (*err != 0) {
          printf(SERIAL_UPSTREAM,
                 "ERROR Parsing Command SPEED for Joint %s: Invalid Value. "
                 "%s\n",
                 s->name, err);
        } else if (val < 0) {
          printf(SERIAL_UPSTREAM,
                 "ERROR Parsing Command SPEED for Joint %s: Negative Value "
                 "(%f)\n",
                 s->name, val);
        } else {
          s->set_speed(val);
        }
        return;
      }
  } else if (!global::enabled) {
    printf(SERIAL_UPSTREAM, "ERROR SYSTEM DISABLED.\n");
    return;
  } else if (STR_EQ(ctx.cmd, "DISABLE")) {
    digitalWrite(PIN::STEPPER_ENABLE, HIGH);
    global::enabled = false;
    for (auto s : Components::steppers)
      s->reset();
    printf(SERIAL_UPSTREAM, "INFO  SYSTEM DISABLED\n");
  } else if (STR_EQ(ctx.cmd, "HOME")) {
    for (auto s : Components::steppers)
      if (STR_EQ(ctx.arg, s->name)) {
        s->home(0);
        return;
      }
  } else if (STR_EQ(ctx.cmd, "MOVE")) {
    for (auto s : Components::steppers)
      if (STR_EQ(ctx.arg, s->name)) {
        char *err;
        double val = strtod(ctx.val, &err);
        if (*err != 0) {
          printf(SERIAL_UPSTREAM,
                 "ERROR Parsing Command SPEED for Joint %s: Invalid Value. "
                 "%s\n",
                 s->name, err);
        } else {
          s->move(val);
        }
        return;
      }
  } else {
    printf(SERIAL_UPSTREAM, "ERROR Unknown Command");
  }
}