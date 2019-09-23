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
#include "xlgyroserver.h"

#define QUEUE_PERMS     ((int)(0644))
#define QUEUE_ATTR_INITIALIZER ((struct mq_attr) {0, XLGYRO_SERVER_QUEUE_MSGSIZE, XLGYRO_SERVER_QUEUE_MSGSIZE, 0, {0}})

static pthread_t xlgyroClientThread;
static XLGYRO_DATA_QUEUE_S xlGyroQueue = { 0 };
static pthread_mutex_t queueMut = PTHREAD_MUTEX_INITIALIZER;

/*
pthread_mutex_t mutex1 = PTHREAD_MUTEX_INITIALIZER;
int counter=0;
void functionC()
{
   pthread_mutex_lock( &mutex1 );
   counter++
   pthread_mutex_unlock( &mutex1 );
}

*/

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

static void *xlgyroClient(void *arg)
{
    XLGYRO_DATA_S xlGyroData = { 0 };
    printf("[XLGYROSERVER]: thread created\n");

    bool queueStatus = false;

    while (true)
    {
        memset(&xlGyroData, 0, sizeof(XLGYRO_DATA_S));
        queueStatus = XlGyroQueueGet(&xlGyroData);
        if (queueStatus == true)
        {
            printReceivedData(&xlGyroData);
        }
    }
}

int CreateXlGyroServer()
{
    return pthread_create(&xlgyroClientThread, NULL, xlgyroClient, NULL);
}

bool XlGyroQueueGet(XLGYRO_DATA_S *pData)
{
    bool ret = false;
    bool isEmpty = false;

    if (pData != NULL)
    {
        pthread_mutex_lock(&queueMut);
        isEmpty = IsXlGyroQueueEmpty();
        if (!isEmpty)
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
            xlGyroQueue.head++;
            if (xlGyroQueue.head >= XLGYRO_SERVER_QUEUE_MAXMSG)
            {
                xlGyroQueue.head = 0;
            }
        }
        pthread_mutex_unlock(&queueMut);
    }
}

bool IsXlGyroQueueEmpty()
{
    bool ret = false;

    pthread_mutex_lock(&queueMut);
    if (xlGyroQueue.count == 0)
    {
        ret = true;
    }
    pthread_mutex_unlock(&queueMut);

    return ret;
}

uint32_t IsXlGyroQueueItemsCount()
{
    uint32_t ret = 0;

    pthread_mutex_lock(&queueMut);
    ret = xlGyroQueue.count;
    pthread_mutex_unlock(&queueMut);

    return ret;
}