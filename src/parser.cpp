// =============================================================================
// Serial Command Parser
// =============================================================================
// License: MIT
// Author: Yuxuan Zhang (zhangyuxuan@ufl.edu)
// =============================================================================
#include <global.h>
#include <parser.h>

namespace Parser {

Context::Context(Stream *port, void *handler) : handler(handler), port(port) {
  this->state = ParserState::CMD;
  memset(cmd, 0, sizeof(cmd) + sizeof(arg) + sizeof(val));
};

void uTask(void *c) {
  auto &context = *reinterpret_cast<Context *>(c);
  auto handle = reinterpret_cast<CommandHandler>(context.handler);
  static char *p = context.cmd;
  // Scan next char from Serial port
  if (Serial.available() > 0) {
    char c = Serial.read();
    if (c >= 'a' && c <= 'z')
      c += 'A' - 'a'; // Convert to upper case
    // Echo back to Serial, useful if you want to directly type commands
    Serial.write(c);
    const bool is_separator = c == ' ' || c == '\t',
               is_assignment = c == '=' || c == ':',
               is_line_feed = c == '\n' || c == '\r';
    // Parse the command
    switch (context.state) {
    case ParserState::PANIC:
      if (is_line_feed) {
        p = context.cmd;
        context.state = ParserState::CMD;
      }
      break;
    case ParserState::CMD:
      if (p == context.cmd && (is_separator || is_line_feed))
        ; // Skip leading spaces
      else if (is_separator) {
        *p = '\0';
        p = context.arg;
        context.state = ParserState::ARG;
      } else if (is_line_feed) {
        *p = '\0';
        context.arg[0] = '\0';
        context.val[0] = '\0';
        p = context.cmd;
        context.state = ParserState::CMD;
        return handle(&context);
      } else if (p == context.cmd + sizeof(context.cmd)) {
        context.state = ParserState::PANIC;
      } else {
        *p++ = c;
      }
      break;
    case ParserState::ARG:
      if (p == context.arg && is_separator)
        ; // Skip leading spaces
      else if (is_assignment) {
        *p = '\0';
        p = context.val;
        context.state = ParserState::VAL;
      } else if (is_separator) {
        *p = '\0';
        p = context.arg;
        context.state = ParserState::ARG;
        return handle(&context);
      } else if (is_line_feed) {
        *p = '\0';
        context.val[0] = '\0';
        p = context.cmd;
        context.state = ParserState::CMD;
        return handle(&context);
      } else if (p == context.arg + sizeof(context.arg)) {
        context.state = ParserState::PANIC;
      } else {
        *p++ = c;
      }
      break;
    case ParserState::VAL:
      if (is_separator) {
        *p = '\0';
        p = context.arg;
        context.state = ParserState::ARG;
        return handle(&context);
      } else if (is_line_feed) {
        *p = '\0';
        p = context.cmd;
        context.state = ParserState::CMD;
        return handle(&context);
      } else if (p == context.val + sizeof(context.val)) {
        context.state = ParserState::PANIC;
      } else {
        *p++ = c;
      }
      break;
    default:
      context.state = ParserState::PANIC;
    }
  }
}

} // namespace Parser