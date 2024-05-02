// Author: Yuxuan Zhang
#include <Arduino.h>

#include "global.h"
#include "stepper.h"

namespace MotorState {
using namespace stepper;

void RESET(Motor *motor);
void HOME(Motor *motor);
void NORMAL(Motor *motor);

void RESET(Motor *motor) { return; }

void HOME(Motor *motor) {
  auto &state = motor->home_state;
  switch (state.stage) {
  case HomingStage::INIT:
    motor->reset(); // set all internal variables to default
    // ==========================================
    // Initiate homing sequence for direction -1
    state.direction = -1;
    state.limit_switch = &(motor->pin.sw1);
    state.stage = HomingStage::FASTFEED;
    // ==========================================
    break;
  case HomingStage::FASTFEED:
  case HomingStage::SLOWFEED:
    if (!digitalRead(*(state.limit_switch))) {
      if (state.stage == HomingStage::FASTFEED) {
        // ==========================================
        // Initiate backoff sequence
        motor->pos.dst = motor->pos.cur + motor->backoff * state.direction;
        state.stage = HomingStage::BACKOFF;
        // ==========================================
      } else if (state.limit_switch == &(motor->pin.sw1)) {
        motor->pos.min = motor->pos.cur;
        // ==========================================
        // Initiate homing sequence for direction +1
        state.direction = +1;
        state.limit_switch = &(motor->pin.sw2);
        state.stage = HomingStage::FASTFEED;
        // ==========================================
      } else {
        motor->pos.max = motor->pos.cur;
        auto range = motor->pos.max - motor->pos.min;
        motor->pos.min = -range / 2;
        motor->pos.max = +range / 2;
        motor->pos.cur = motor->pos.max;
        // ==========================================
        // DONE: Move to neutral position
        motor->pos.dst = 0;
        state.stage = HomingStage::DONE;
        // ==========================================
      }
    } else {
      motor->pos.dst = motor->pos.cur + 10 * state.direction * motor->delta;
      auto const full_speed = motor->get_speed();
      if (state.stage == HomingStage::SLOWFEED)
        motor->set_speed(full_speed / 10);
      motor->step();
      motor->set_speed(full_speed);
    }
    break;
  case HomingStage::BACKOFF:
    if (!digitalRead(*(state.limit_switch))) {
      // Backoff until limit switch is released
      motor->pos.dst = motor->pos.cur + motor->backoff * state.direction;
      motor->step();
    } else if (motor->step()) {
      // ==========================================
      // Initiate slowfeed sequence
      state.stage = HomingStage::SLOWFEED;
      // ==========================================
    }
    break;
  case HomingStage::DONE:
  default:
    if (motor->step()) {
      motor->executor = MotorState::NORMAL;
      printf("RANGE %s=%f,%f\n", motor->name, motor->pos.min, motor->pos.max);
    }
    break;
  }
}

void NORMAL(Motor *motor) { motor->step(); }

} // namespace MotorState

namespace stepper {

void Motor::set_speed(double speed) {
  this->speed = speed;
  this->step_interval = round(1e6 * delta / speed);
}

double Motor::get_speed() { return speed; }

bool Motor::step() {
  if (abs(pos.cur - pos.dst) < epsilon)
    return true;
  const auto us = micros();
  // Check if next step is due
  if (us >= u_stamp) {
    const auto dir = pos.dst > pos.cur ? LOW : HIGH;
    // Update direction
    digitalWrite(pin.dir, dir);
    // Update step
    digitalWrite(pin.step, HIGH);
    // Update current position
    if (pos.dst > pos.cur)
      pos.cur += delta;
    else
      pos.cur -= delta;
    // Update time stamp
    u_stamp += step_interval;
    // Signal the motion
    global::sig_motion = true;
    // Reset step pin to LOW, t_step_low >= 100ns
    // Should be enough for the motor to detect the step
    // delayMicroseconds(1);
    digitalWrite(pin.step, LOW);
  }
  return false;
}

bool Motor::ready() { return executor == MotorState::NORMAL; }

Motor::Motor(const char *name, const Pinout pin, double delta)
    : name(name), pin(pin), delta(delta), epsilon(abs(delta * 0.6)) {
  reset(MotorState::RESET);
  pinMode(pin.dir, OUTPUT);
  pinMode(pin.step, OUTPUT);
  pinMode(pin.sw1, INPUT_PULLUP);
  pinMode(pin.sw2, INPUT_PULLUP);
}

void Motor::home(double speed) {
  set_speed(speed);
  home_state.stage = HomingStage::INIT;
  executor = MotorState::HOME;
}

void Motor::move(double pos, double speed) {
  if (ready()) {
    set_speed(speed);
    this->u_stamp = micros();
    this->pos.dst = min(this->pos.max, max(this->pos.min, pos));
  } else {
    printf("ERROR Motor %s not in NORMAL state\n", name);
  }
}

void Motor::reset(void (*state)(Motor *)) {
  pos.min = 0;
  pos.max = 0;
  pos.cur = 0;
  pos.dst = 0;
  u_stamp = micros();
  if (state != NULL)
    executor = state;
}

void Motor::uTask() { executor(this); }

} // namespace stepper
