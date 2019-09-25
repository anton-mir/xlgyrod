#include <pthread.h>
#include <stdbool.h>
#include <stdio.h>
#include <unistd.h>
#include <mqueue.h>
#include <fcntl.h>    /* For O_* constants. */
#include <sys/stat.h> /* For mode constants. */
#include <sys/types.h>
#include <pthread.h>
#include <signal.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <errno.h>
#include "xlgyro_data_processor.h"
#include "xlgyro_server.h"
#include "xlgyro_options.h"

#define XLGYRO_DATA_PROCESSOR_MAXMSG        (32)    /* Maximum number of messages. */

typedef struct XLGYRO_DATA_QUEUE_STRUCT
{
    uint32_t head;
    uint32_t tail;
    XLGYRO_DATA_S queue[XLGYRO_DATA_PROCESSOR_MAXMSG];
    uint32_t count;
} XLGYRO_DATA_QUEUE_S;

static pthread_t xlgyroDataProcessorTh;
static XLGYRO_DATA_QUEUE_S xlGyroQueue = { 0 };
static pthread_mutex_t queueMtx = PTHREAD_MUTEX_INITIALIZER;
static pthread_cond_t queueCond = PTHREAD_COND_INITIALIZER;

static XLGYRO_READER_THREAD_PARAMS_S params = { 0 };

static FILE *fpLog;

static bool xlGyroQueueGetUnsafe(XLGYRO_DATA_S *pData);
static void logObstacle();

static bool isObstacle(XLGYRO_DATA_S *pData)
{
    bool ret = false;
    double deviation = 0;

    if (pData != NULL)
    {
        if ( pData->current.axisValue[E_Z_AXIS] > params.zAxisThdHi ||
             pData->current.axisValue[E_Z_AXIS] < params.zAxisThdLo )
        {
            ret = true;
        }

        deviation = pData->averaged.axisValue[E_Z_AXIS] - pData->current.axisValue[E_Z_AXIS];
        deviation /= pData->averaged.axisValue[E_Z_AXIS];
        if ( deviation > params.deviationHi ||
             deviation < params.deviationLo )
        {
            ret = true;
        }
    }

    return ret;
}

static void logObstacle()
{
    if (fpLog != NULL)
    {
        fprintf(fpLog, "[XLGYRODDATA] ++++++++++++ OBSTACLE ++++++++++++\n");
    }

    if (params.daemon == 0)
    {
        printf( "[XLGYRODDATA]: ++++++++++++ OBSTACLE ++++++++++++\n");
    }
}

static void printReceivedData(XLGYRO_DATA_S *pData)
{
    if (pData != NULL)
    {
        if (fpLog != NULL)
        {
            fprintf(fpLog, "=========================================================================\n");
            fprintf(fpLog, "[XLGYRODDATA]: Min: [%+.6f]; [%+.6f]; [%+.6f]; \n",
                        pData->min.axisValue[E_X_AXIS],
                        pData->min.axisValue[E_Y_AXIS],
                        pData->min.axisValue[E_Z_AXIS]);

            fprintf(fpLog, "[XLGYRODDATA]: Max: [%+.6f]; [%+.6f]; [%+.6f]; \n",
                        pData->max.axisValue[E_X_AXIS],
                        pData->max.axisValue[E_Y_AXIS],
                        pData->max.axisValue[E_Z_AXIS]);

            fprintf(fpLog, "[XLGYRODDATA]: Avr: [%+.6f]; [%+.6f]; [%+.6f]; \n",
                        pData->averaged.axisValue[E_X_AXIS],
                        pData->averaged.axisValue[E_Y_AXIS],
                        pData->averaged.axisValue[E_Z_AXIS]);

            fprintf(fpLog, "[XLGYRODDATA]: Cur: [%+.6f]; [%+.6f]; [%+.6f]; \n",
                        pData->current.axisValue[E_X_AXIS],
                        pData->current.axisValue[E_Y_AXIS],
                        pData->current.axisValue[E_Z_AXIS]);
        }

        if (params.daemon == 0)
        {
            printf("=========================================================================\n");
            printf("[XLGYRODDATA]: Min: [%+.6f]; [%+.6f]; [%+.6f]; \n",
                        pData->min.axisValue[E_X_AXIS],
                        pData->min.axisValue[E_Y_AXIS],
                        pData->min.axisValue[E_Z_AXIS]);

            printf("[XLGYRODDATA]: Max: [%+.6f]; [%+.6f]; [%+.6f]; \n",
                        pData->max.axisValue[E_X_AXIS],
                        pData->max.axisValue[E_Y_AXIS],
                        pData->max.axisValue[E_Z_AXIS]);

            printf("[XLGYRODDATA]: Avr: [%+.6f]; [%+.6f]; [%+.6f]; \n",
                        pData->averaged.axisValue[E_X_AXIS],
                        pData->averaged.axisValue[E_Y_AXIS],
                        pData->averaged.axisValue[E_Z_AXIS]);

            printf("[XLGYRODDATA]: Cur: [%+.6f]; [%+.6f]; [%+.6f]; \n",
                        pData->current.axisValue[E_X_AXIS],
                        pData->current.axisValue[E_Y_AXIS],
                        pData->current.axisValue[E_Z_AXIS]);
        }
    }
}

static void xlGyroSendData(XLGYRO_DATA_S *pData, bool isObstacle)
{
    uint8_t idx = 0;
    XLGYRO_PACKET_S packet = { 0 };

    packet.preambule1 = XLGYRO_PREAMBULE_VALUE;
    packet.preambule2 = XLGYRO_PREAMBULE_VALUE;
    memcpy(&packet.data, pData, sizeof(XLGYRO_DATA_S));
    packet.isObstacle = isObstacle;

    for (idx = 0; idx < XLGYRO_TRAILER_SIZE; ++idx)
    {
        packet.trailer[idx] = XLGYRO_TRAILER_VALUE;
    }

    XlGyroServerSendToClients((uint8_t*)&packet, sizeof(XLGYRO_PACKET_S));
}

static void *xlgyroDataProcessorThread(void *arg)
{
    XLGYRO_DATA_S xlGyroData = { 0 };
    bool obstacle = false;
    bool queueStatus = false;
    uint32_t items = 0;
    int error = 0;

    memset(&xlGyroData, 0, sizeof(XLGYRO_DATA_S));

    if (arg != NULL)
    {
        memcpy(&params, arg, sizeof(XLGYRO_READER_THREAD_PARAMS_S));
    }

    fpLog = fopen("xlgyrod.log", "w");

    while (true)
    {
        do
        {
            error = pthread_mutex_lock(&queueMtx);
            if (error != 0)
            {
                printf("[XLGYRODDATA]: pthread_mutex_lock() failed; error: %d\n", error);
                continue;
            }

            while (xlGyroQueue.count == 0)
            {
                error = pthread_cond_wait(&queueCond, &queueMtx);
                if (error != 0)
                {
                    printf("[XLGYRODDATA]: pthread_cond_wait() failed; error: %d\n", error);
                    break;
                }
            }

            queueStatus = xlGyroQueueGetUnsafe(&xlGyroData);

            error = pthread_mutex_unlock(&queueMtx);
            if (error != 0)
            {
                printf("[XLGYRODDATA]: pthread_mutex_unlock() failed; error: %d\n", error);
                continue;
            }

            if (queueStatus == true)
            {
                obstacle = isObstacle(&xlGyroData);
                if (obstacle)
                {
                    logObstacle();
                }
                xlGyroSendData(&xlGyroData, obstacle);
                printReceivedData(&xlGyroData);
                memset(&xlGyroData, 0, sizeof(XLGYRO_DATA_S));
            }

        } while (queueStatus);
    }
}

int XlGyroDataProcessorCreate(void *args)
{
    return pthread_create(&xlgyroDataProcessorTh, NULL, xlgyroDataProcessorThread, args);
}

static bool xlGyroQueueGetUnsafe(XLGYRO_DATA_S *pData)
{
    bool ret = false;

    if (xlGyroQueue.head != xlGyroQueue.tail)
    {
        memcpy(pData, &xlGyroQueue.queue[xlGyroQueue.head], sizeof(XLGYRO_DATA_S));
        xlGyroQueue.head++;
        if (xlGyroQueue.head >= XLGYRO_DATA_PROCESSOR_MAXMSG)
        {
            xlGyroQueue.head = 0;
        }
        if (xlGyroQueue.count > 0)
        {
            xlGyroQueue.count--;
        }

        ret = true;
    }

    return ret;
}

bool XlGyroQueueGet(XLGYRO_DATA_S *pData)
{
    bool ret = false;
    int error = 0;

    if (pData != NULL)
    {
        error = pthread_mutex_lock(&queueMtx);
        if (error != 0)
        {
            printf("[XLGYRODDATA]: pthread_mutex_lock() failed; error: %d\n", error);
            return false;
        }

        ret = xlGyroQueueGetUnsafe(pData);

        error = pthread_mutex_unlock(&queueMtx);
        if (error != 0)
        {
            printf("[XLGYRODDATA]: pthread_mutex_unlock() failed; error: %d\n", error);
            return false;
        }
    }

    return ret;
}

void XlGyroQueuePush(XLGYRO_DATA_S *pData)
{
    int error = 0;

    if (pData != NULL)
    {
        error = pthread_mutex_lock(&queueMtx);
        if (error != 0)
        {
            printf("[XLGYRODDATA]: pthread_mutex_lock() failed; error: %d\n", error);
            return;
        }

        memcpy(&xlGyroQueue.queue[xlGyroQueue.tail], pData, sizeof(XLGYRO_DATA_S));
        xlGyroQueue.tail++;
        if (xlGyroQueue.tail >= XLGYRO_DATA_PROCESSOR_MAXMSG)
        {
            xlGyroQueue.tail = 0;
        }

        if (xlGyroQueue.tail != xlGyroQueue.head)
        {
            xlGyroQueue.count++;
        }
        else
        {
            /* queue overlapped */
            /* Old value was overwritten with new one. Move head */
            xlGyroQueue.head++;
            if (xlGyroQueue.head >= XLGYRO_DATA_PROCESSOR_MAXMSG)
            {
                xlGyroQueue.head = 0;
            }
        }
        error = pthread_mutex_unlock(&queueMtx);
        if (error != 0)
        {
            printf("[XLGYRODDATA]: pthread_mutex_unlock() failed; error: %d\n", error);
            return;
        }

        error = pthread_cond_signal(&queueCond);
        if (error != 0)
        {
            printf("[XLGYRODDATA]: pthread_cond_signal() failed; error: %d\n", error);
            return;
        }
    }
}

uint32_t GetXlGyroQueueItemsCount()
{
    uint32_t ret = 0;
    int error = 0;

    error = pthread_mutex_lock(&queueMtx);
    if (error != 0)
    {
        printf("[XLGYRODDATA]: pthread_mutex_lock() failed; error: %d\n", error);
        return 0;
    }

    ret = xlGyroQueue.count;

    error = pthread_mutex_unlock(&queueMtx);
    if (error != 0)
    {
        int err = errno;
        printf("[XLGYRODDATA]: pthread_mutex_unlock() failed; error: %d\n", error);
        return 0;
    }

    return ret;
}