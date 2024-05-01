namespace parser {

extern struct command_s {
  char cmd[8], arg[64], val[64];
} command;

enum state { NONE = 0, UPDATE = 0b01, COMMIT = 0b10 };

int uTask();

} // namespace parser
