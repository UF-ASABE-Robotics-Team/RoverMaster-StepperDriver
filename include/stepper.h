// =============================================================================
// Stepper Motor Control API
// =============================================================================
// License: MIT
// Author: Yuxuan Zhang (zhangyuxuan@ufl.edu)
// =============================================================================
#pragma once
#include <global.h>
#include <hook.h>
#include <sys/types.h>

namespace Stepper {

typedef struct EndSwitch_s {
  const int pin; // Switch disabled if pin < 0
  const int trigger;
  // Sensorless homing mode
  // Imposes untangle before homing, and skips slow-feed phase.
  const bool sensorless;
  bool valid() const { return pin >= 0; }
  bool triggered() const { return valid() && (digitalRead(pin) == trigger); }
} EndSwitch;

enum HomingOrigin { START, MIDDLE, END };

typedef struct StepperInfo_s {
  int dir_pin, step_pin;
  const EndSwitch sw1, sw2;
  int sw_dir(const EndSwitch *sw) const { return sw == &(sw1) ? LOW : HIGH; };
  // Stepper configuration
  double ratio;   // Steps per unit distance.
                  // For a rotational joint with 1.8 degree per step,
                  // and distance defined in degrees,
                  // ratio = 1 / 1.8 (steps per degree)
  double backoff; // Backoff distance,
                  // in relative distance, will be divided by ratio
  struct {
    // Margin for safe operation, in relative distance.
    // Helps to clear limit switch / mechanical end stop.
    // Positive value narrows operation range,
    // Negative value expands operation range.
    double left, right;
    // Turn on enforcement to hard limit operation inside margin.
    bool enforce;
  } margin;
  // Origin of homing. Coordinates will be zeroed at origin.
  // In case only one end switch is available, origin will be set to that end.
  enum HomingOrigin origin;
} StepperInfo;

enum HomingStage { FAST_FEED, BACKOFF, SLOW_FEED, DONE, UNTANGLE };

class Motor {
public:
  unsigned long interval;
  double base_speed;
  // Data race protection
  bool locked = false;
  void wait_lock(); // Do not use this function in timer callback
  // Stepper States
  struct {
    bool falling_edge;
    long position; // Current position, in steps
    long target;   // Destination position, in steps
    struct {
      long min, max;
    } range;
    struct {
      bool sw1, sw2;
    } homed;
  } state;
  // Internal Parameters
  const StepperInfo info;
  // Name of the joint driven by this motor
  const char *name;
  // Next scheduler call due timestamp
  // Determined at runtime
  struct {
    const EndSwitch *sw = NULL, *next_sw = NULL;
    HomingStage stage;
    int dir = LOW;
    long counter = 0;
  } home_state;
  // Execute step
  void step();
  void step(int direction);
  // Set speed by changing timer period
  void set_speed(double speed, bool change_base = true);
  // Initialization
  Motor(const char *name,
        // Stepper configuration
        const StepperInfo info,
        // Initial speed of the joint, used for initialize timer
        double speed = 60.0,
        // Init hook
        Hook after_init = Hook());
  void init();
  // Returns true if the motor can be operated.
  bool ready();
  // Exposed Methods
  void home(int direction = 0); // direction = 0 for both ends
                                // direction < 0 for sw1 (left end)
                                // direction > 0 for sw2 (right end)
                                // Homing sw2 should happen after sw1 homed.
  void move(double pos);
  void reset();
  // Micro task called by uTask
  void (*executor)(Motor *) = NULL;
  // Hooks for additional configuration
  Hook after_init;
};

// Micro Task, called by timer callback at twice the stepping frequency
void uTask(void *p);

} // namespace Stepper
