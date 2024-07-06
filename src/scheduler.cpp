// =============================================================================
// Task Scheduler
// =============================================================================
// License: MIT
// Author: Yuxuan Zhang (zhangyuxuan@ufl.edu)
// =============================================================================
#include <scheduler.h>

#define LIST_SIZE 32

namespace Scheduler {

Task *task_list[LIST_SIZE];

Vector<Task *> tasks(task_list, 0);

void Task::run() { handle(context); }

Task::task_s(Handler handle, void *context, const unsigned long *period)
    : handle(handle), context(context), period(period), wait_time(0) {}

Task::task_s(task_s &t)
    : handle(t.handle), context(t.context), period(t.period), wait_time(0) {}

void tick() {
  // Last scheduler tick time
  static unsigned long last = micros();
  // Initialize the context
  Task *most_urgent_task = nullptr;
  unsigned long overdue_time = 0;
  unsigned long now = micros();
  // Compute delta t, handle counter overflow
  unsigned long delta_t = now > last ? now - last : now + (ULONG_MAX - last);
  // Update "last time"
  last = now;
  // Update schedule timetable & Find the most urgent overdue task
  for (auto &task : tasks) {
    task->wait_time += delta_t;
    if (task->wait_time >= *task->period) {
      const auto task_overdue_time = task->wait_time - *task->period;
      if (most_urgent_task == nullptr || task_overdue_time > overdue_time) {
        most_urgent_task = task;
        overdue_time = task_overdue_time;
      }
    }
  }
  // Run most urgent task, if any
  if (most_urgent_task != nullptr) {
    most_urgent_task->run();
    // Update wait time to be the amount of overdue time
    most_urgent_task->wait_time = overdue_time;
  }
}

} // namespace Scheduler
