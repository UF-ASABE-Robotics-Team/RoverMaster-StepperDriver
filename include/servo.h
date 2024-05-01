#include <Arduino.h>
#include <Servo.h>

namespace servo {

enum HomingStage { LOWEST, HIGHEST, NEUTRAL };

typedef struct {
  int min, neutral, max;
  double angle;
} servo_range;

class Motor {
private:
  Servo servo;
  const servo_range range;

public:
  // Underlying Implementation
  // Name of the joint driven by this motor
  const char *name;
  // Internal Parameters
  const int pin;
  const double delta;
  // Determined at runtime
  struct {
    HomingStage stage;
  } home_state;
  struct {
    double cur, dst;
  } pos;
  double speed;
  // Last stepper movement time
  unsigned long u_stamp;
  // Initialization
  Motor(const char *name, const int pin,
        const servo_range range = {1000, 1500, 2000, 180.0});
  // Exposed Methods
  bool step();
  void move(double angle, double speed);
  void move(double angle);
  void enable();
  void disable();
  // execute micro task
  void uTask();
};

} // namespace servo
