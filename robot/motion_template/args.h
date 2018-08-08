#if !defined GLOBAL_ARGS_H
#define GLOBAL_ARGS_H

#include "compiler.h"

#if !defined RUN_FILENAME_LENGTH
#define RUN_FILENAME_LENGTH        (128)
#endif

extern
int run__check_args(int argc, char **argv);

extern
char *run__getarg_tcphost(char *host);
extern
uint16_t run__getarg_tcpport();
extern
uint16_t run__getarg_udpport();
extern
posix__boolean_t run__getarg_simflag();
extern
void run__getarg_simodo(double *x, double *y, double *w);
extern
char *run__getarg_canio_driver(char *file);
extern
posix__boolean_t run__if_must_login();
extern
posix__boolean_t run__if_loadonexec();

#endif