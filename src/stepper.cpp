// =============================================================================
// Stepper Control API
// =============================================================================
// License: MIT
// Author: Yuxuan Zhang (zhangyuxuan@ufl.edu)
// =============================================================================
#include <Arduino.h>
#include <unistd.h>

#include "global.h"
#include "io_pin_remap.h"
#include "scheduler.h"
#include "stepper.h"

namespace MotorState {
using namespace Stepper;

void RESET(Motor *motor);
void HOME(Motor *motor);
void NORMAL(Motor *motor);

void RESET(Motor *motor) { return; }

void HOME(Motor *motor) {
  auto &state = motor->home_state;
  if (!state.sw->valid()) {
    motor->executor = MotorState::NORMAL;
    return;
  }
  switch (state.stage) {
  case HomingStage::FASTFEED:
    if (state.sw->triggered()) {
      // Initiate backoff sequence
      state.stage = HomingStage::BACKOFF;
      state.dir = state.dir == LOW ? HIGH : LOW;
      cast_assign(motor->home_state.counter,
                  rint(motor->info.backoff / motor->info.ratio));
    } else {
      motor->step(state.dir);
    }
    break;
  case HomingStage::SLOWFEED:
    if (state.sw->triggered()) {
      if (state.sw == &(motor->info.sw1)) {
        cast_assign(motor->state.range.min,
                    motor->state.position +
                        rint(motor->info.margin.left / motor->info.ratio));
        motor->state.homed.sw1 = true;
      }
      if (state.sw == &(motor->info.sw2)) {
        cast_assign(motor->state.range.max,
                    motor->state.position -
                        rint(motor->info.margin.right / motor->info.ratio));
        motor->state.homed.sw2 = true;
      }
      // Initiate DONE sequence
      state.stage = HomingStage::DONE;
      // Find new origin on this axis
      long offset;
      if (motor->state.homed.sw1 && motor->state.homed.sw2 &&
          motor->info.origin == HomingOrigin::MIDDLE) {
        offset = (motor->state.range.max + motor->state.range.min) / 2;
      } else if ((motor->state.homed.sw1 && !motor->state.homed.sw1) ||
                 motor->info.origin == HomingOrigin::START) {
        offset = motor->state.range.min;
      } else if ((motor->state.homed.sw2 && !motor->state.homed.sw2) ||
                 motor->info.origin == HomingOrigin::END) {
        offset = motor->state.range.max;
      }
      motor->state.range.min -= offset;
      motor->state.range.max -= offset;
      motor->state.position -= offset;
      motor->state.target = 0;
    } else {
      if (state.counter < 4)
        state.counter++;
      else {
        state.counter = 0;
        motor->step(state.dir);
      }
    }
    break;
  case HomingStage::BACKOFF:
    if (state.counter <= 0) {
      // Initiate slow feed sequence
      state.counter = 0;
      state.stage = HomingStage::SLOWFEED;
      state.dir = state.dir == LOW ? HIGH : LOW;
    } else {
      // Backoff <counter> steps after limit switch released
      if (!state.sw->triggered())
        state.counter--;
      motor->step(state.dir);
    }
    break;
  case HomingStage::DONE:
  default:
    if (motor->state.position == motor->state.target) {
      if (state.next_sw != NULL && state.next_sw->valid()) {
        // Initiate homing for next switch
        state.sw = state.next_sw;
        state.next_sw = NULL;
        state.stage = HomingStage::FASTFEED;
        state.dir = motor->info.sw_dir(state.sw);
      } else {
        // Homing complete
        motor->executor = MotorState::NORMAL;
        if (motor->ready()) {
          broadcast("RANGE %s=%f,%f\n", motor->name,
                 motor->state.range.min * motor->info.ratio,
                 motor->state.range.max * motor->info.ratio);
          broadcast("READY %s\n", motor->name);
        }
      }
    } else {
      motor->step();
    }
  }
}

void NORMAL(Motor *motor) { motor->step(); }

} // namespace MotorState

namespace Stepper {

void Motor::step() {
  const auto &state = this->state;
  if (state.position != state.target) {
    const auto dir = state.position < state.target ? HIGH : LOW;
    this->step(dir);
    global::sig_motion = true;
  }
}

void Motor::step(int direction) {
  if (digitalRead(this->info.dir_pin) != direction) {
    digitalWrite(this->info.dir_pin, direction);
    // Delay between direction change and step pulse
    // TMC220X requires 20ns
    // DRV8825 requires 650ns
    // 1us delay should be enough for all
    usleep(1);
  }
  digitalWrite(this->info.step_pin, HIGH);
  this->state.falling_edge = true;
}

void Motor::set_speed(double speed /* unit distance per second */,
                      bool change_base) {
  // Steps per second
  auto freq = speed / info.ratio;
  // Prevent near-zero speed
  if (freq < 1e-6)
    freq = 1e-6;
  // Update timer interval
  // Double the frequency, half used for triggering falling edge.
  cast_assign(this->interval, rint(1e6 / (2 * freq)));
  // Check if base speed needs to be updated
  if (change_base)
    this->base_speed = speed;
}

Motor::Motor(const char *name, const StepperInfo info, double speed)
    : name(name), info(info) {
  reset();
  set_speed(speed, true);
  Scheduler::tasks.push_back(
      new Scheduler::Task(uTask, this, &(this->interval)));
}

void Motor::init() {
  pinMode(info.dir_pin, OUTPUT);
  pinMode(info.step_pin, OUTPUT);
  if (info.sw1.valid())
    pinMode(info.sw1.pin, INPUT);
  if (info.sw2.valid())
    pinMode(info.sw2.pin, INPUT);
}

bool Motor::ready() {
  return this->executor == MotorState::NORMAL &&
         (!this->info.sw1.valid() || this->state.homed.sw1) &&
         (!this->info.sw2.valid() || this->state.homed.sw2);
}

void Motor::home(int direction) {
  this->locked = true;
  this->executor = MotorState::HOME;
  this->home_state.stage = HomingStage::FASTFEED;
  this->home_state.sw = direction <= 0 ? &(this->info.sw1) : &(this->info.sw2);
  this->home_state.next_sw = direction == 0 ? &(this->info.sw2) : NULL;
  this->home_state.dir = this->info.sw_dir(this->home_state.sw);
  this->locked = false;
}

void Motor::move(double pos) {
  this->locked = true;
  if (ready()) {
    auto target =
        static_cast<decltype(this->state.target)>(rint(pos / info.ratio));
    if (this->info.margin.enforce) {
      if (this->info.sw1.valid() && target < this->state.range.min) {
        target = this->state.range.min;
      } else if (this->info.sw2.valid() && target > this->state.range.max) {
        target = this->state.range.max;
      }
    }
    this->state.target = target;
    this->executor = MotorState::NORMAL;
  } else {
    broadcast("ERROR Motor %s not in NORMAL state\n", name);
  }
  this->locked = false;
}

void Motor::reset() {
  this->locked = true;
  this->state = {.falling_edge = false,
                 .position = 0,
                 .target = 0,
                 .range = {0, 0},
                 .homed = {false, false}};
  executor = MotorState::RESET;
  this->locked = false;
}

// Called by timer interrupt
void uTask(void *p) {
  auto &m = *static_cast<Motor *>(p);
  if (!m.locked) {
    if (m.state.falling_edge) {
      // Execute falling edge
      digitalWrite(m.info.step_pin, LOW);
      m.state.falling_edge = false;
      // Increment position
      const auto dir = digitalRead(m.info.dir_pin);
      m.state.position += (dir == HIGH) ? 1 : -1;
    } else {
      m.executor(&m);
    }
  }
}

} // namespace Stepper
