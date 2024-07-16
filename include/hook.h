// =============================================================================
// Stepper Motor Control API
// =============================================================================
// License: MIT
// Author: Yuxuan Zhang (zhangyuxuan@ufl.edu)
// =============================================================================
#pragma once

class Hook {
public:
  void (*fn)(void *) = nullptr;
  void *context = nullptr;
  Hook() : fn(nullptr), context(nullptr) {}
  Hook(void (*fn)(void *)) : fn(fn), context(nullptr) {}
  Hook(void (*fn)(void *), void *context) : fn(fn), context(context) {}
  void call() {
    if (fn != nullptr)
      fn(context);
  };
  void call_with_context(void *context) {
    if (fn != nullptr)
      fn(context);
  };
};
