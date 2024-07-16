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

void init();

extern const Vector<Stepper::Motor *> steppers;

} // namespace components
