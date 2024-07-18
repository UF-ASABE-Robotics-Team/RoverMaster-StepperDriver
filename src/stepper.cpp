// =============================================================================
// Stepper Control API
// =============================================================================
// License: MIT
// Author: Yuxuan Zhang (zhangyuxuan@ufl.edu)
// =============================================================================
#include "esp32-hal-gpio.h"
#include <Arduino.h>
#include <unistd.h>

#include <global.h>
#include <pinout.h>
#include <scheduler.h>
#include <stepper.h>

namespace MotorState {
using namespace Stepper;

void RESET(Motor *motor);
void HOME(Motor *motor);
void NORMAL(Motor *motor);

void RESET(Motor *motor) { return; }

void HOME(Motor *motor) {
  auto &state = motor->home_state;
  // Validate end stop switch
  if (!state.sw->valid()) {
    motor->executor = MotorState::NORMAL;
    return;
  }
  // State machine
  switch (state.stage) {
  case HomingStage::FAST_FEED:
    if (state.sw->triggered()) {
      const long backoff_steps = rint(motor->info.backoff / motor->info.ratio);
      if (state.counter < backoff_steps) {
        // If move distance is too short, initiate untangle sequence
        state.stage = HomingStage::UNTANGLE;
        cast_assign(state.counter, backoff_steps);
      } else {
        // Initiate backoff sequence
        state.stage = HomingStage::BACKOFF;
        state.dir = state.dir == LOW ? HIGH : LOW;
        cast_assign(motor->home_state.counter,
                    rint(motor->info.backoff / motor->info.ratio));
      }
    } else {
      motor->step(state.dir);
      state.counter++;
    }
    break;
  case HomingStage::SLOW_FEED:
    if (state.sw->triggered() || state.sw->sensorless) {
      if (state.sw == &(motor->info.sw1)) {
        auto offset = rint(motor->info.margin.left / motor->info.ratio);
        if (state.sw->sensorless)
          offset -= rint(motor->info.backoff / motor->info.ratio);
        cast_assign(motor->state.range.min, motor->state.position + offset);
        motor->state.homed.sw1 = true;
      }
      if (state.sw == &(motor->info.sw2)) {
        auto offset = rint(motor->info.margin.right / motor->info.ratio);
        if (state.sw->sensorless)
          offset -= rint(motor->info.backoff / motor->info.ratio);
        cast_assign(motor->state.range.max, motor->state.position - offset);
        motor->state.homed.sw2 = true;
      }
      // Find new origin on this axis
      long offset = rint(motor->info.margin.offset / motor->info.ratio);
      if (motor->state.homed.sw1 && motor->state.homed.sw2 &&
          motor->info.origin == HomingOrigin::MIDDLE) {
        offset += (motor->state.range.max + motor->state.range.min) / 2;
      } else if ((motor->state.homed.sw1 && !motor->state.homed.sw1) ||
                 motor->info.origin == HomingOrigin::START) {
        offset += motor->state.range.min;
      } else if ((motor->state.homed.sw2 && !motor->state.homed.sw2) ||
                 motor->info.origin == HomingOrigin::END) {
        offset += motor->state.range.max;
      }
      // Adjust reference frame
      motor->state.range.min -= offset;
      motor->state.range.max -= offset;
      motor->state.position -= offset;
      // Initiate DONE sequence
      state.stage = HomingStage::DONE;
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
      state.stage = HomingStage::SLOW_FEED;
      state.dir = state.dir == LOW ? HIGH : LOW;
    } else {
      // Backoff <counter> steps after limit switch released
      if (!state.sw->triggered())
        state.counter--;
      motor->step(state.dir);
    }
    break;
  case HomingStage::UNTANGLE:
    // Untangle sequence, clear DIAG flag from stepper controller.
    if (state.counter > 0) {
      motor->step(state.dir);
      if (--state.counter <= 0)
        // Reverse untangle direction
        state.dir = state.dir == LOW ? HIGH : LOW;
    } else if (state.sw->sensorless && state.sw->triggered()) {
      double r = (motor->info.sw_dir(state.sw) == state.dir) ? 1.0 : 1.5;
      // Reload untangle step counter
      cast_assign(state.counter,
                  rint(r * motor->info.backoff / motor->info.ratio));
    } else {
      // Reload counter, prevent untangle sequence from being triggered again
      cast_assign(state.counter, rint(motor->info.backoff / motor->info.ratio));
      // Initiate fast feed sequence
      state.stage = HomingStage::FAST_FEED;
      state.dir = motor->info.sw_dir(state.sw);
    }
    break;
  case HomingStage::DONE:
  default:
    if (state.next_sw != NULL && state.next_sw->valid()) {
      // Initiate homing for next switch
      state.sw = state.next_sw;
      state.next_sw = NULL;
      state.stage = HomingStage::FAST_FEED;
      state.dir = motor->info.sw_dir(state.sw);
    } else {
      motor->executor = MotorState::NORMAL;
      // Homing complete
      if (motor->ready()) {
        printf(SERIAL_UPSTREAM, "RANGE %s=%f,%f\n", motor->name,
               motor->state.range.min * motor->info.ratio,
               motor->state.range.max * motor->info.ratio);
        printf(SERIAL_UPSTREAM, "READY %s\n", motor->name);
        // Move to zero position
        motor->state.target = 0;
      } else {
        // Stand still, wait for other motors to complete homing
        motor->state.target = motor->state.position;
      }
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
}

void Motor::set_speed(double speed /* unit distance per second */,
                      bool change_base) {
  if (speed != 0) { // Steps per second
    auto freq = abs(speed) / info.ratio;
    // Prevent near-zero speed
    if (freq < 1e-6)
      freq = 1e-6;
    // Update timer interval
    // Double the frequency, half used for triggering falling edge.
    cast_assign(this->interval, rint(1e6 / (2 * freq)));
  } else {
    this->interval = 0;
  }
  // Check if base speed needs to be updated
  if (change_base)
    this->base_speed = speed;
}

Motor::Motor(const char *name, const StepperInfo info, Hook after_init)
    : name(name), info(info), after_init(after_init) {
  reset();
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
  this->set_speed(0);
  after_init.call();
}

bool Motor::ready() {
  return this->executor == MotorState::NORMAL &&
         ((!this->info.sw1.valid()) || this->state.homed.sw1) &&
         ((!this->info.sw2.valid()) || this->state.homed.sw2);
}

void Motor::home(int direction) {
  this->executor = MotorState::HOME;
  this->state.target = this->state.position;
  this->home_state.stage = HomingStage::FAST_FEED;
  this->home_state.sw = direction <= 0 ? &(this->info.sw1) : &(this->info.sw2);
  this->home_state.next_sw = direction == 0 ? &(this->info.sw2) : NULL;
  this->home_state.dir = this->info.sw_dir(this->home_state.sw);
  this->home_state.counter = 0;
  this->set_speed(this->info.speed);
}

void Motor::move(double pos) {
  if (ready()) {
    this->set_speed(this->base_speed);
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
    printf(SERIAL_UPSTREAM, "ERROR Motor %s not in NORMAL state\n", name);
  }
}

void Motor::reset() {
  this->state = {
      .position = 0, .target = 0, .range = {0, 0}, .homed = {false, false}};
  executor = MotorState::RESET;
}

// Called by timer interrupt
void uTask(void *p) {
  auto &m = *static_cast<Motor *>(p);
  if (digitalRead(m.info.step_pin) == HIGH) {
    // Execute falling edge
    digitalWrite(m.info.step_pin, LOW);
    // Increment position
    const auto dir = digitalRead(m.info.dir_pin);
    m.state.position += (dir == HIGH) ? 1 : -1;
    // Signal motion if stepper is operation in NORMAL mode
    if (m.executor == MotorState::NORMAL) {
      global::sig_motion = true;
      // Check if target position is reached
      if (m.state.position == m.state.target)
        m.set_speed(0);
    }
  } else {
    m.executor(&m);
  }
}

} // namespace Stepper
