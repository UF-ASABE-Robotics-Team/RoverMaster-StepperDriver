// Author: Yuxuan Zhang

#include <Arduino.h>

#include "board_pinout.h"
#include "global.h"
#include "parser.h"
#include "servo.h"
#include "stepper.h"

bool enabled;

void enable() {
  digitalWrite(PIN_STEPPER_ENABLE, LOW);
  enabled = true;
}

void disable() {
  digitalWrite(PIN_STEPPER_ENABLE, HIGH);
  enabled = false;
}

double speed = 120.0;
struct {
  double j1, j2;
} move_dst = {0, 0};

const unsigned long sync_interval = 10000; // 10ms

int main() {
  init();
  Serial.begin(115200);
  printf("INFO  SYSTEM INIT\n");
  pinMode(PIN_STEPPER_ENABLE, OUTPUT);
  stepper::Motor j1("J1", {PIN_J1_DIR, PIN_J1_STEP, PIN_J1_SW1, PIN_J1_SW2});
  servo::Motor j2("J2", PIN_J2_SERVO, {600, 1655, 2600, 180.0});
  disable();
  int parser_ret;
  unsigned long next_sync = micros();
  while (1) {
    j1.uTask();
    j2.uTask();
    if ((parser_ret = parser::uTask())) {
      // Commands that can be executed when disabled
      if (strcmp(parser::command.cmd, "ENABLE") == 0) {
        enable();
        j2.enable();
        printf("INFO  SYSTEM ENABLED\n");
      } else if (strcmp(parser::command.cmd, "SPEED") == 0) {
        speed = atof(parser::command.arg);
      }
      // BARRIER: Commands that can only be executed when enabled
      else if (!enabled) {
        printf("ERROR SYSTEM DISABLED\n");
        continue;
      } else if (strcmp(parser::command.cmd, "DISABLE") == 0) {
        disable();
        j2.disable();
        printf("INFO  SYSTEM DISABLED\n");
      } else if (strcmp(parser::command.cmd, "HOME") == 0) {
        if (strcmp(parser::command.arg, j1.name) == 0) {
          j1.home(speed);
        } else {
          printf("ERROR Parsing Command HOME: Unknown Joint");
        }
      } else if (strcmp(parser::command.cmd, "MOVE") == 0) {
        if (parser_ret & parser::state::UPDATE && parser::command.val[0] != 0) {
          double val = strtod(parser::command.val, NULL);
          if (strcmp(parser::command.arg, j1.name) == 0) {
            move_dst.j1 = val;
          } else if (strcmp(parser::command.arg, j2.name) == 0) {
            move_dst.j2 = val;
          } else {
            printf("ERROR Parsing Command MOVE: Unknown Joint");
          }
        }
        if (parser_ret & parser::state::COMMIT) {
          const double m1 = abs(move_dst.j1 - j1.pos.cur),
                       m2 = abs(move_dst.j2 - j2.pos.cur),
                       max_move = max(m1, m2);
          if (max_move > 0) {
            double s;
            s = speed * m1 / max_move;
            j1.move(move_dst.j1, s);
            s = speed * m2 / max_move;
            j2.move(move_dst.j2, s);
          }
        }
      } else {
        printf("ERROR Unknown Command");
      }
    }
    // Check if it is time to sync
    const auto us = micros();
    if (enabled && global::sig_motion && us >= next_sync) {
      global::sig_motion = false;
      next_sync = us + sync_interval;
      printf("SYNC  J1=%f J2=%f\n", j1.pos.cur, j2.pos.cur);
    }
  }
  return 0;
}
