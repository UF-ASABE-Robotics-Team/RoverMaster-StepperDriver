#include "board_pinout.h"
#include <Arduino.h>
#include <Servo.h>

Servo tilt_servo;

int isJoint1AtLimit() {
  int result = 0;
  if (digitalRead(PIN_J1_SW1))
    result += 1;
  if (digitalRead(PIN_J1_SW2))
    result += 2;
  return result;
}

int home(int pin_dir, int pin_step, int pin_sw, int dir, double speed = 50.0,
         int back = 500) {
  if (speed < 0) {
    dir = !dir;
    speed = -speed;
  }
  int d = max(round(1000000 / (100 * speed)) / 2, 1);
  int steps = 0;
  // Forward until switch is triggerred
  digitalWrite(pin_dir, dir);
  while (digitalRead(pin_sw)) {
    digitalWrite(pin_step, HIGH);
    delayMicroseconds(d);
    digitalWrite(pin_step, LOW);
    delayMicroseconds(d);
    steps++;
  }
  // Backward for a few steps
  digitalWrite(pin_dir, !dir);
  for (int i = 0; i < back; i++) {
    digitalWrite(pin_step, HIGH);
    delayMicroseconds(d);
    digitalWrite(pin_step, LOW);
    delayMicroseconds(d);
    steps--;
  }
  // Slowly move forward until switch is triggerred
  d *= 8;
  digitalWrite(pin_dir, dir);
  while (digitalRead(pin_sw)) {
    digitalWrite(pin_step, HIGH);
    delayMicroseconds(d);
    digitalWrite(pin_step, LOW);
    delayMicroseconds(d);
    steps++;
  }
  return steps;
}

void move(int pin_dir, int pin_step, int dir, int steps, double speed = 1.0) {
  if (speed < 0) {
    dir = !dir;
    speed = -speed;
  }
  int d = max(round(1000000 / (100 * speed)) / 2, 1);
  digitalWrite(pin_dir, dir);
  for (; steps > 0; steps--) {
    digitalWrite(pin_step, HIGH);
    delayMicroseconds(d);
    digitalWrite(pin_step, LOW);
    delayMicroseconds(d);
  }
}

int main() {
  // Initialize Arduino
  init();
  // Initialize pins
  pinMode(PIN_STEPPER_ENABLE, OUTPUT);
  pinMode(PIN_J1_DIR, OUTPUT);
  pinMode(PIN_J1_STEP, OUTPUT);
  pinMode(PIN_J1_SW1, INPUT_PULLUP);
  pinMode(PIN_J1_SW2, INPUT_PULLUP);
  tilt_servo.attach(PIN_J2_SERVO);
  // Activate servo
  tilt_servo.write(90);
  delay(100);
  for (int a = 90; a <= 135; a++) {
    tilt_servo.write(a);
    delay(10);
  }
  for (int a = 135; a >= 0; a--) {
    tilt_servo.write(a);
    delay(10);
  }
  for (int a = 0; a <= 90; a++) {
    tilt_servo.write(a);
    delay(10);
  }
  // Enable stepper driver
  home(PIN_J1_DIR, PIN_J1_STEP, PIN_J1_SW1, HIGH);
  // Home reverse direction
  int range = home(PIN_J1_DIR, PIN_J1_STEP, PIN_J1_SW2, LOW);
  // Move to neutral
  move(PIN_J1_DIR, PIN_J1_STEP, HIGH, range / 2, 50);
  // Stop the stepper driver
  digitalWrite(PIN_STEPPER_ENABLE, HIGH);
  // Dead loop
  while (1);
}