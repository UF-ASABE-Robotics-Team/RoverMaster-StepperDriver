// =============================================================================
// Serial Command Parser
// =============================================================================
// License: MIT
// Author: Yuxuan Zhang (zhangyuxuan@ufl.edu)
// =============================================================================
#pragma once
#include <global.h>

namespace Parser {

void uTask(void *callback);
enum ParserState { CMD, ARG, VAL, PANIC };
class Context {
protected:
  enum ParserState state = ParserState::CMD;
  void *handler;

public:
  Stream *port;
  char cmd[16], arg[64], val[64];
  Context(Stream *port, void *handler);
  friend void uTask(void *context);
};

typedef void (*CommandHandler)(Context *);

} // namespace Parser
