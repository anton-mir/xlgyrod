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

static pthread_t xlgyroServerThread;
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
        printf("[XLGYROSERVER]: Min: [%+.6f]; [%+.6f]; [%+.6f]; \n",
                    pData->min.axisValue[E_X_AXIS],
                    pData->min.axisValue[E_Y_AXIS],
                    pData->min.axisValue[E_Z_AXIS]);

        printf("[XLGYROSERVER]: Cur: [%+.6f]; [%+.6f]; [%+.6f]; \n",
                    pData->current.axisValue[E_X_AXIS],
                    pData->current.axisValue[E_Y_AXIS],
                    pData->current.axisValue[E_Z_AXIS]);

        printf("[XLGYROSERVER]: Max: [%+.6f]; [%+.6f]; [%+.6f]; \n",
                    pData->max.axisValue[E_X_AXIS],
                    pData->max.axisValue[E_Y_AXIS],
                    pData->max.axisValue[E_Z_AXIS]);

        printf("[XLGYROSERVER]: Avr: [%+.6f]; [%+.6f]; [%+.6f]; \n",
                    pData->averaged.axisValue[E_X_AXIS],
                    pData->averaged.axisValue[E_Y_AXIS],
                    pData->averaged.axisValue[E_Z_AXIS]);
    }
}

static void *xlgyroDataProcessorThread(void *arg)
{
    XLGYRO_DATA_S xlGyroData = { 0 };
    bool obstacle = false;

    bool queueStatus = false;

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
                    printReceivedData(&xlGyroData);
                    printf("OOOOBBBSSSSTAAACCCLEEEEE !!!\n");
                }
                printReceivedData(&xlGyroData);
                memset(&xlGyroData, 0, sizeof(XLGYRO_DATA_S));
            }
        } while (queueStatus);
    }
}

int CreateXlGyroDataProcessor()
{
    return pthread_create(&xlgyroServerThread, NULL, xlgyroDataProcessorThread, NULL);
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
            if (xlGyroQueue.head >= XLGYRO_SERVER_QUEUE_MAXMSG)
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
        if (xlGyroQueue.tail >= XLGYRO_SERVER_QUEUE_MAXMSG)
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
            if (xlGyroQueue.head >= XLGYRO_SERVER_QUEUE_MAXMSG)
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