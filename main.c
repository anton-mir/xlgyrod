#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <stdbool.h>
#include <errno.h>
#include <fcntl.h>
#include <string.h>
#include <unistd.h>
#include <time.h>
#include "crc16.h"
#include "xlgyro_server.h"
#include "xlgyro_data_processor.h"
#include "xlgyro_reader.h"

#define DEFAULT_PORTNAME    "/dev/ttyUSB0"

int main(int argc, char *argv[])
{
    uint32_t appendIdx = 0;
    uint32_t processIdx = 0;
    uint32_t unprocessedBytes = 0;
    char *portname = NULL;
    XLGYRO_READER_THREAD_PARAMS_S params;

    if (argc > 1)
    {
        params.portname = argv[1];
    }
    else
    {
        params.portname = DEFAULT_PORTNAME;
    }

    int status = XlGyroDataProcessorCreate(NULL);
    if (status < 0)
    {
        int err = errno;
        printf("[XLGYROD]: XlGyroDataProcessorCreate() failed; error: %s\n", strerror(err));
        return status;
    }

    status = XlGyroServerCreate(NULL);
    if (status < 0)
    {
        int err = errno;
        printf("[XLGYROD]: XlGyroServerCreate() failed; error: %s\n", strerror(err));
        return status;
    }

    status = XlGyroReaderCreate((void*)&params);
    if (status < 0)
    {
        int err = errno;
        printf("[XLGYROD]: XlGyroServerCreate() failed; error: %s\n", strerror(err));
        return status;
    }

    while (true)
    {
        sleep(1000);
    }

    return 0;
}