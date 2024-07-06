// =============================================================================
// Stepper Controller Firmware for the RoverMaster Project
// =============================================================================
// License: MIT
// Author: Yuxuan Zhang (zhangyuxuan@ufl.edu)
// =============================================================================
#include "USBCDC.h"
#include <board_pinout.h>
#include <components.h>
#include <global.h>

Stream *openHardwareSerial(int id, int baudrate = 115200) {
  // auto serial = new HardwareSerial(id);
  // serial->begin(baudrate);
  // global::ports.push_back(serial);
  // return serial;
  return nullptr;
}

void sync(void *) {
  if (global::enabled && global::sig_motion) {
    global::sig_motion = false;
    broadcast("SYNC ");
    for (auto s : Components::steppers)
      broadcast(" %s=%f", s->name, s->state.position * s->info.ratio);
    broadcast("\n");
  }
}

void setup() {
  pinMode(PIN_STEPPER_ENABLE, OUTPUT);
  digitalWrite(PIN_STEPPER_ENABLE, HIGH);
  for (auto s : Components::steppers) {
    s->init();
  }
  static const unsigned long parser_interval = 100; // 0.1ms
  static const unsigned long sync_interval = 1000;  // 1ms
  Scheduler::tasks.push_back(new Scheduler::Task(
      Parser::uTask,
      (void *)new Parser::Context(&Serial, (void *)global::handleSerialCommand),
      &parser_interval));
  Scheduler::tasks.push_back(
      new Scheduler::Task(sync, &Serial, &sync_interval));
  Serial.begin(115200);
  global::ports.push_back(&Serial);
}

void loop() { Scheduler::tick(); }
