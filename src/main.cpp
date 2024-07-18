// =============================================================================
// Stepper Controller Firmware for the RoverMaster Project
// =============================================================================
// License: MIT
// Author: Yuxuan Zhang (zhangyuxuan@ufl.edu)
// =============================================================================
#include <components.h>
#include <global.h>
#include <pinout.h>

void sync(void *) {
  if (global::enabled && global::sig_motion) {
    global::sig_motion = false;
    printf(SERIAL_UPSTREAM, "SYNC ");
    for (auto s : Components::steppers)
      printf(SERIAL_UPSTREAM, " %s=%f", s->name,
             s->state.position * s->info.ratio);
    printf(SERIAL_UPSTREAM, "\n");
  }
}

#if defined(SERIAL_DOWNSTREAM)
static const unsigned long serial_forward_interval = 1000; // 1ms
void serial_forward(void *) {
  static char buf[256] = {0};
  static char *p = buf;
  while (SERIAL_DOWNSTREAM.available() > 0) {
    if (p >= buf + sizeof(buf))
      p = buf; // Buffer overflow, clear buffer
    const char c = *(p++) = SERIAL_DOWNSTREAM.read();
    if (c == '\n' || c == '\r') {
      SERIAL_UPSTREAM.write(buf, p - buf);
      p = buf;
    }
  }
}
#endif

void setup() {
  Components::init();
  for (auto s : Components::steppers)
    s->init();
  static const unsigned long parser_interval = 1000; // 1ms
  static const unsigned long sync_interval = 10000;  // 10ms
  Scheduler::tasks.push_back(new Scheduler::Task(
      Parser::uTask,
      (void *)new Parser::Context((void *)global::handleSerialCommand),
      &parser_interval));
  Scheduler::tasks.push_back(
      new Scheduler::Task(sync, &SERIAL_UPSTREAM, &sync_interval));
#ifdef SERIAL_DOWNSTREAM
  Scheduler::tasks.push_back(
      new Scheduler::Task(serial_forward, nullptr, &serial_forward_interval));
#endif
}

void loop() { Scheduler::tick(); }
