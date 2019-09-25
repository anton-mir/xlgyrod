#ifndef XLGYRO_READRE_H
#define XLGYRO_READRE_H

typedef struct XLGYRO_READER_THREAD_PARAMS_STRUCT
{
    char *portname;
} XLGYRO_READER_THREAD_PARAMS_S;

int XlGyroReaderCreate(void *arg);

#endif /* XLGYRO_READRE_H */
