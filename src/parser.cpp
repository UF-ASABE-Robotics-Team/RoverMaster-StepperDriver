#include <Arduino.h>

#include "parser.h"

namespace parser {

struct command_s command = {"", "", ""};

int uTask() {
  static enum parser_state { CMD, ARG, VAL, PANIC } parser_state = CMD;
  static char *p = command.cmd;
  // Scan next char from Serial port
  if (Serial.available() > 0) {
    char c = Serial.read();
    if (c >= 'a' && c <= 'z')
      c += 'A' - 'a'; // Convert to upper case
    // Echo back to Serial, useful if you want to directly type commands
    // Serial.write(c);
    const bool is_separator = c == ' ' || c == '\t',
               is_assignment = c == '=' || c == ':',
               is_line_feed = c == '\n' || c == '\r';
    // Parse the command
    switch (parser_state) {
    case PANIC:
      if (is_line_feed) {
        p = command.cmd;
        parser_state = CMD;
      }
      break;
    case CMD:
      if (p == command.cmd && (is_separator || is_line_feed))
        ; // Skip leading spaces
      else if (is_separator) {
        *p = '\0';
        p = command.arg;
        parser_state = ARG;
      } else if (is_line_feed) {
        *p = '\0';
        command.arg[0] = '\0';
        command.val[0] = '\0';
        p = command.cmd;
        parser_state = CMD;
        return state::UPDATE | state::COMMIT;
      } else if (p == command.cmd + sizeof(command.cmd)) {
        parser_state = PANIC;
      } else {
        *p++ = c;
      }
      break;
    case ARG:
      if (p == command.arg && is_separator)
        ; // Skip leading spaces
      else if (is_assignment) {
        *p = '\0';
        p = command.val;
        parser_state = VAL;
      } else if (is_separator) {
        *p = '\0';
        p = command.arg;
        parser_state = ARG;
        return state::UPDATE;
      } else if (is_line_feed) {
        *p = '\0';
        command.val[0] = '\0';
        p = command.cmd;
        parser_state = CMD;
        return state::UPDATE | state::COMMIT;
      } else if (p == command.arg + sizeof(command.arg)) {
        parser_state = PANIC;
      } else {
        *p++ = c;
      }
      break;
    case VAL:
      if (is_separator) {
        *p = '\0';
        p = command.arg;
        parser_state = ARG;
        return state::UPDATE;
      } else if (is_line_feed) {
        *p = '\0';
        p = command.cmd;
        parser_state = CMD;
        return state::UPDATE | state::COMMIT;
      } else if (p == command.val + sizeof(command.val)) {
        parser_state = PANIC;
      } else {
        *p++ = c;
      }
      break;
    default:
      parser_state = PANIC;
    }
  }
  return state::NONE;
}

} // namespace parser