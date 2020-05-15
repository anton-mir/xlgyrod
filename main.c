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
#include "xlgyro_options.h"

static XLGYRO_READER_THREAD_PARAMS_S params;

int main(int argc, char *argv[])
{
    uint32_t appendIdx = 0;
    uint32_t processIdx = 0;
    uint32_t unprocessedBytes = 0;
    int status = 0;

    XlGyroReadOptions(argc, argv, &params);

    if (params.daemon == true)
    {
        status = daemon(1, 0);
        if (status < 0)
        {
            int err = errno;
            printf("[XLGYROD]: daemon() failed; error: %s\n", strerror(err));
            return status;
        }
    }

    status = XlGyroDataProcessorCreate((void*)&params);
    if (status < 0)
    {

        int err = errno;
        printf("[XLGYROD]: XlGyroDataProcessorCreate() failed; error: %s\n", strerror(err));
        return status;
    }

    status = XlGyroReaderCreate((void*)&params);
    if (status < 0)
    {
        int err = errno;
        printf("[XLGYROD]: XlGyroServerCreate() failed; error: %s\n", strerror(err));
        return status;
    }

    status = XlGyroServerCreate((void*)&params);
    if (status < 0)
    {
        int err = errno;
        printf("[XLGYROD]: XlGyroServerCreate() failed; error: %s\n", strerror(err));
        return status;
    }

    if(strncmp("NONE",params.ttytest,strlen("NONE")) != 0)
    {
        status = XlGyroTestCreate((void*)&params);
        if (status < 0)
        {
            int err = errno;
            printf("[XLGYROD]: XlGyroTestCreate() failed; error: %s\n", strerror(err));
            return status;
        }
    }

    while (true)
    {
        sleep(1000);
    }

    return 0;
}