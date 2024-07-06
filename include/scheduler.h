// =============================================================================
// Task Scheduler
// =============================================================================
// License: MIT
// Author: Yuxuan Zhang (zhangyuxuan@ufl.edu)
// =============================================================================
#pragma once

#include <Vector.h>
#include <global.h>

namespace Scheduler {

typedef void (*Handler)(void *);

typedef struct task_s {
  Handler handle;
  void *context;
  const unsigned long *period;
  unsigned long wait_time;
  void run();
  task_s(Handler handle, void *context, const unsigned long *period);
  task_s() = default;
  task_s(task_s &t);
} Task;

extern Vector<Task *> tasks;

extern void tick();

} // namespace Scheduler
