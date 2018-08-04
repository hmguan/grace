#if !defined PERIOD_STORAGE_H
#define PERIOD_STORAGE_H

#include "compiler.h"

extern
int mm__load_mapping();
extern
void mm__release_mapping();

extern
int mm__getupl(void *upl);
extern
int mm__setupl(const void *upl);
extern
int mm__getloc(void *loc, int cb);
extern
int mm__setloc(const void *loc, int cb);
extern
int mm__set_calibration(int varid, int cb, const void *data);
extern
int mm__get_calibration(int varid, void *data);

#endif