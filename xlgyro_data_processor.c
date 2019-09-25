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

#define SLEEP_ON_QUEUE_EMPTY_TIMEOUT        (200000)    // 200 ms

static pthread_t xlgyroDataProcessorTh;
static XLGYRO_DATA_QUEUE_S xlGyroQueue = { 0 };
static pthread_mutex_t queueMut = PTHREAD_MUTEX_INITIALIZER;

static bool isObstacle(XLGYRO_DATA_S *pData)
{
    bool ret = false;
    double deviation = 0;

    if (pData != NULL)
    {
        if (pData->current.axisValue[E_Z_AXIS] > 0.4 || pData->current.axisValue[E_Z_AXIS] < -0.4)
        {
            ret = true;
        }

        deviation = pData->averaged.axisValue[E_Z_AXIS] - pData->current.axisValue[E_Z_AXIS];
        deviation /= pData->averaged.axisValue[E_Z_AXIS];
        if (deviation > 300 || deviation < -300)
        {
            ret = true;
        }
    }

    return ret;
}

static void printReceivedData(XLGYRO_DATA_S *pData)
{
    if (pData != NULL)
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
    (void)arg;
    XLGYRO_DATA_S xlGyroData = { 0 };
    bool obstacle = false;
    bool queueStatus = false;
    uint32_t items = 0;

    memset(&xlGyroData, 0, sizeof(XLGYRO_DATA_S));

    while (true)
    {
        do
        {
            queueStatus = XlGyroQueueGet(&xlGyroData);
            if (queueStatus == true)
            {
                obstacle = isObstacle(&xlGyroData);
                if (obstacle)
                {
                    printf("OOOOBBBSSSSTAAACCCLEEEEE !!!\n");
                }
                xlGyroSendData(&xlGyroData, obstacle);
                printReceivedData(&xlGyroData);
                memset(&xlGyroData, 0, sizeof(XLGYRO_DATA_S));
            }

            items = IsXlGyroQueueItemsCount();
            if (items == 0)
            {
                usleep(SLEEP_ON_QUEUE_EMPTY_TIMEOUT);
            }

        } while (queueStatus);
    }
}

int XlGyroDataProcessorCreate(void *args)
{
    return pthread_create(&xlgyroDataProcessorTh, NULL, xlgyroDataProcessorThread, args);
}

bool XlGyroQueueGet(XLGYRO_DATA_S *pData)
{
    bool ret = false;

    if (pData != NULL)
    {
        pthread_mutex_lock(&queueMut);
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
        pthread_mutex_unlock(&queueMut);
    }

    return ret;
}

void XlGyroQueuePush(XLGYRO_DATA_S *pData)
{
    if (pData != NULL)
    {
        pthread_mutex_lock(&queueMut);
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
        pthread_mutex_unlock(&queueMut);
    }
}

uint32_t IsXlGyroQueueItemsCount()
{
    uint32_t ret = 0;

    pthread_mutex_lock(&queueMut);
    ret = xlGyroQueue.count;
    pthread_mutex_unlock(&queueMut);

    return ret;
}