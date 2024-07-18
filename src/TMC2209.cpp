#include "pinout.h"
#include <TMC2209.h>
#include <TMCStepper.h>

void config(void *context) {
  if (!context) {
    SERIAL_UPSTREAM.println("ERROR TMC2209 config() called with nullptr");
    return;
  }
  const auto &config = *reinterpret_cast<TMC2209::Config *>(context);
  TMC2209Stepper driver(config.port, config.R_SENSE, config.addr);
  driver.begin(); // SPI: Init CS pins and possible SW SPI pins
  driver.toff(4); // Enables driver in software, changed from 5
  // Blank time controls step timing, 0 = no blanking, 1-3 = 1us, 4-7 = 2us,
  // etc.
  driver.blank_time(16);
  driver.rms_current(config.rms_current);
  driver.microsteps(config.micro_steps);
  // driver.en_pwm_mode(true); // Toggle stealthChop on TMC2130/2160/5130/5160
  driver.en_spreadCycle(false); // Toggle spreadCycle on TMC2208/2209/2224
  driver.pwm_autoscale(true);   // Needed for stealthChop
  // driver.semin(5);
  // driver.semax(2);
  // driver.sedn(0b01);
  // DIAG is pulsed by StallGuard when SG_RESULT falls below SGTHRS. It is only
  // enabled in StealthChop mode, and when TCOOLTHRS ≥ TSTEP > TPWMTHRS
  driver.TCOOLTHRS(0xFFFFF); // 20bit max
  driver.TPWMTHRS(0x00000F); // 20bit max
  driver.SGTHRS(config.stall_sensitivity);
}

namespace TMC2209 {

Hook hook_init(Config *context) { return Hook(config, context); }

} // namespace TMC2209