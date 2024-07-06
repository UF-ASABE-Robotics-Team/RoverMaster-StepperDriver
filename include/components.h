// =============================================================================
// Components Onboard
// =============================================================================
// License: MIT
// Author: Yuxuan Zhang (zhangyuxuan@ufl.edu)
// =============================================================================
#pragma once
#include <global.h>
#include <stepper.h>
#include <Vector.h>

namespace Components {

extern const Vector<Stepper::Motor *> steppers;

void init();

} // namespace components
