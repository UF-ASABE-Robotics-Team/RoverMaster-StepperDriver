#include <Arduino.h>

namespace stepper {

typedef struct {
  int dir, step, sw1, sw2;
} Pinout;

enum HomingStage { INIT, FASTFEED, BACKOFF, SLOWFEED, DONE };

class Motor {
  double speed;
  unsigned long step_interval;

public:
  // Name of the joint driven by this motor
  const char *name;
  // Internal Parameters
  const Pinout pin;
  const double delta, epsilon;
  void (*executor)(Motor *);
  const double backoff = -10.0;
  // Determined at runtime
  struct {
    int direction;
    const int *limit_switch;
    HomingStage stage;
  } home_state;
  struct {
    double min, max, cur, dst;
  } pos;
  void set_speed(double speed);
  double get_speed();
  // Last stepper movement time
  unsigned long u_stamp;
  // Initialization
  Motor(const char *name, const Pinout pin,
        double delta = 360.0 / (200 * 8 * 8));
  bool step();
  // Returns true if the motor can be operated.
  bool ready();
  // Exposed Methods
  void home(double speed = 100);
  void move(double pos, double speed = 100);
  void reset(void (*state)(Motor *) = NULL);
  // execute micro task
  void uTask();
};

} // namespace stepper
