#ifndef XLGYRO_OPTIONS_H
#define XLGYRO_OPTIONS_H

#include <stdint.h>

typedef struct XLGYRO_READER_THREAD_PARAMS_STRUCT
{
    char *ttyname;
    char *ttytest;
    double deviationHi;
    double deviationLo;
    double zAxisThdHi;
    double zAxisThdLo;
    uint16_t port;
    bool daemon;
} XLGYRO_READER_THREAD_PARAMS_S;

void XlGyroReadOptions(int argc, char *argv[], XLGYRO_READER_THREAD_PARAMS_S *pParams);

#endif /* XLGYRO_OPTIONS_H */
