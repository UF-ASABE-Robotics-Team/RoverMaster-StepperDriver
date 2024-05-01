// Author: Yuxuan Zhang
#include <Arduino.h>
#include <Servo.h>

#include "global.h"
#include "servo.h"

namespace servo {

bool Motor::step() {
  const unsigned long us = micros();
  const double dt = (double)(us - u_stamp) * 1e-6;
  const double dx = dt * speed;
  u_stamp = us;
  bool move = false;
  if (pos.cur != pos.dst) {
    if (abs(pos.cur - pos.dst) < dx) {
      pos.cur = pos.dst;
    } else {
      pos.cur += sgn(pos.dst - pos.cur) * dx;
    }
    move = true;
  }
  servo.writeMicroseconds(range.neutral + round(pos.cur / delta));
  return move;
}

Motor::Motor(const char *name, const int pin, const servo_range range)
    : range(range), name(name), pin(pin),
      delta(range.angle / (double)(range.max - range.min)), pos({0, 0}),
      speed(1.0), u_stamp(micros()) {}

void Motor::move(double angle, double speed) {
  this->speed = speed;
  this->move(angle);
}

void Motor::move(double angle) {
  pos.dst = angle;
  u_stamp = micros();
}

void Motor::enable() { servo.attach(pin, range.min, range.max); }

void Motor::disable() { servo.detach(); }

void Motor::uTask() {
  if (servo.attached())
    step();
}

} // namespace servo
