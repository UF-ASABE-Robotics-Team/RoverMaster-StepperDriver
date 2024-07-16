// =============================================================================
// Components Onboard
// =============================================================================
// License: MIT
// Author: Yuxuan Zhang (zhangyuxuan@ufl.edu)
// =============================================================================
#include "esp32-hal-uart.h"
#include <TMC2209.h>
#include <components.h>
#include <global.h>
#include <pinout.h>
#include <stepper.h>

#define MICRO_STEP(N) (1.8 / (double)(N))
namespace Components {

#if defined(BOARD_A)

void init() {
  SERIAL_UPSTREAM.begin(115200);
  SERIAL_DOWNSTREAM.begin(115200, SERIAL_8N1, A1, A2);
  pinMode(PIN::STEPPER_ENABLE, OUTPUT);
  digitalWrite(PIN::STEPPER_ENABLE, HIGH);
  SERIAL_TMC2209.begin(115200, SERIAL_8N1, PIN::TMC_SERIAL_RX,
                       PIN::TMC_SERIAL_TX);
  // MS1/MS2 pins set to 0b00
  pinMode(D10, OUTPUT);
  pinMode(D11, OUTPUT);
  digitalWrite(D10, LOW);
  digitalWrite(D11, LOW);
}

Stepper::Motor *stepper_list[] = {
    new Stepper::Motor(
        "J1",
        Stepper::StepperInfo{
            .dir_pin = D5,
            .step_pin = D6,
            .sw1 = {.pin = A7, .trigger = HIGH},
            .sw2 = {.pin = A7, .trigger = HIGH},
            .ratio = MICRO_STEP(32) / 8.0,
            .backoff = 10.0,
            .margin = {.left = 10, .right = 10, .enforce = false},
            .origin = Stepper::HomingOrigin::MIDDLE},
        45.0,
        TMC2209::hook_init(
            new TMC2209::Config{.port = reinterpret_cast<Stream *>(&Serial2),
                                .addr = 0b00,
                                .R_SENSE = 0.11f,    // Ohm
                                .rms_current = 1200, // mA
                                .micro_steps = 32,
                                .stall_sensitivity = 0})),
};

#elif defined(BOARD_B)

void init() {
  SERIAL_UPSTREAM.begin(115200);
  pinMode(PIN::STEPPER_ENABLE, OUTPUT);
  digitalWrite(PIN::STEPPER_ENABLE, HIGH);
  SERIAL_TMC2209.begin(115200, SERIAL_8N1, PIN::TMC_SERIAL_RX,
                       PIN::TMC_SERIAL_TX);
}

Stepper::Motor *stepper_list[] = {
    new Stepper::Motor(
        "J2",
        Stepper::StepperInfo{
            .dir_pin = D12,
            .step_pin = D11,
            .sw1 = {.pin = D10, .trigger = HIGH, .sensorless = true},
            .sw2 = {.pin = D10, .trigger = HIGH, .sensorless = true},
            .ratio = MICRO_STEP(32) / 6.0,
            .backoff = 10.0,
            .margin = {.left = 10, .right = 10, .enforce = false},
            .origin = Stepper::HomingOrigin::MIDDLE},
        60.0,
        TMC2209::hook_init(
            new TMC2209::Config{.port = reinterpret_cast<Stream *>(&Serial2),
                                .addr = 0b00,
                                .R_SENSE = 0.11f,    // Ohm
                                .rms_current = 1000, // mA
                                .micro_steps = 32,
                                .stall_sensitivity = 64})),
    new Stepper::Motor(
        "Z",
        Stepper::StepperInfo{
            .dir_pin = D9,
            .step_pin = D8,
            .sw1 = {.pin = D7, .trigger = HIGH, .sensorless = true},
            .sw2 = {.pin = -1, .trigger = HIGH, .sensorless = true},
            .ratio = 8.0 * (MICRO_STEP(32) / 360.0) / 2.0,
            .backoff = 10.0,
            .margin = {.left = 5, .right = 5, .enforce = false},
            .origin = Stepper::HomingOrigin::START},
        10.0,
        TMC2209::hook_init(
            new TMC2209::Config{.port = reinterpret_cast<Stream *>(&Serial2),
                                .addr = 0b01,
                                .R_SENSE = 0.11f,    // Ohm
                                .rms_current = 1000, // mA
                                .micro_steps = 32,
                                .stall_sensitivity = 100})),
    new Stepper::Motor(
        "J3",
        Stepper::StepperInfo{
            .dir_pin = D5,
            .step_pin = D4,
            .sw1 = {.pin = D2, .trigger = HIGH, .sensorless = false},
            .sw2 = {.pin = D2, .trigger = HIGH, .sensorless = false},
            .ratio = MICRO_STEP(64) / 3.0,
            .backoff = 15.0,
            .margin = {.left = 10, .right = 10, .enforce = false},
            .origin = Stepper::HomingOrigin::MIDDLE},
        90.0,
        TMC2209::hook_init(
            new TMC2209::Config{.port = reinterpret_cast<Stream *>(&Serial2),
                                .addr = 0b10,
                                .R_SENSE = 0.11f,   // Ohm
                                .rms_current = 800, // mA
                                .micro_steps = 64,
                                .stall_sensitivity = 0})),
};

#else

#error "Board not defined"

#endif

const Vector<Stepper::Motor *> steppers(stepper_list,
                                        sizeof(stepper_list) /
                                            sizeof(*stepper_list)); // NOLINT

} // namespace Components
