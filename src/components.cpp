// =============================================================================
// Components Onboard
// =============================================================================
// License: MIT
// Author: Yuxuan Zhang (zhangyuxuan@ufl.edu)
// =============================================================================
#include "stepper.h"
#include <components.h>
#include <global.h>

namespace Components {

Stepper::Motor s1("J3",
                  Stepper::StepperInfo{
                      .dir_pin = D2,
                      .step_pin = D5,
                      .sw1 = {.pin = A7, .trigger = HIGH},
                      .sw2 = {.pin = A7, .trigger = HIGH},
                      .ratio = 1.8 / 16.0 / 3.0,
                      .backoff = 15.0,
                      .margin = {.left = 10, .right = 10, .enforce = false},
                      .origin = Stepper::HomingOrigin::MIDDLE},
                  120.0);

Stepper::Motor s2("J2",
                  Stepper::StepperInfo{
                      .dir_pin = D3,
                      .step_pin = D6,
                      .sw1 = {.pin = A6, .trigger = HIGH},
                      .sw2 = {.pin = A6, .trigger = HIGH},
                      .ratio = 1.8 / 8.0 / 8.0,
                      .backoff = 10.0,
                      .margin = {.left = 10, .right = 10, .enforce = false},
                      .origin = Stepper::HomingOrigin::MIDDLE},
                  40.0);

Stepper::Motor s3("Z",
                  Stepper::StepperInfo{
                      .dir_pin = D4,
                      .step_pin = D7,
                      .sw1 = {.pin = D13, .trigger = HIGH},
                      .sw2 = {.pin = -1, .trigger = HIGH},
                      .ratio = 8 * 1.8 / 8.0 / 2.0 / 360.0,
                      .backoff = 10.0,
                      .margin = {.left = 10, .right = 10, .enforce = false},
                      .origin = Stepper::HomingOrigin::MIDDLE},
                  20.0);

Stepper::Motor *stepper_list[] = {&s1, &s2, &s3};

const Vector<Stepper::Motor *> steppers(stepper_list,
                                        sizeof(stepper_list) /
                                            sizeof(*stepper_list)); // NOLINT

} // namespace Components
