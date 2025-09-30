#ifndef INC_DEBUG_H_
#define INC_DEBUG_H_

#ifdef DEBUG
#define DEBUG_TEST 1
#else
#define DEBUG_TEST 0
#endif
/*
Prints message with "[DEBUG]: " tag when DEBUG flag is set. Newline character
included.

Needs ##__VA_ARGS__ support (gcc)
*/
#define debug_print(fmt, ...)                                           \
  do {                                                                  \
    if (DEBUG_TEST) fprintf(stderr, "[DEBUG]: " fmt "\n", __VA_ARGS__); \
  } while (0)

#endif /* INC_DEBUG_H_ */