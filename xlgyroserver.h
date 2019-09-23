#ifndef XLGYROSERVER_H
#define XLGYROSERVER_H

#include <stdint.h>

#define XLGYRO_SERVER_QUEUE_NAME        "/xlgyroclientmq"   /* Queue name. */
#define XLGYRO_SERVER_QUEUE_MSGSIZE     (sizeof(XLGYRO_DATA_S))                /* Length of message. */
#define XLGYRO_SERVER_QUEUE_MAX_PRIO    ((int)(5))
#define XLGYRO_SERVER_QUEUE_MAXMSG      (32)                  /* Maximum number of messages. */

typedef enum
{
    E_X_AXIS = 0,
    E_Y_AXIS,
    E_Z_AXIS,
    E_AXIS_COUNT
} AXISES;

typedef struct AXIS_DOUBLE_VALUE_STRUCT
{
    double axisValue[E_AXIS_COUNT];
} AXIS_DOUBLE_VALUE_S;

typedef struct XLGYRO_DATA_STRUCT
{
    AXIS_DOUBLE_VALUE_S current;
    AXIS_DOUBLE_VALUE_S min;
    AXIS_DOUBLE_VALUE_S max;
    AXIS_DOUBLE_VALUE_S averaged;
} XLGYRO_DATA_S;

typedef struct XLGYRO_DATA_QUEUE_STRUCT
{
    uint32_t head;
    uint32_t tail;
    XLGYRO_DATA_S queue[XLGYRO_SERVER_QUEUE_MAXMSG];
    uint32_t count;
} XLGYRO_DATA_QUEUE_S;

int CreateXlGyroServer();
bool IsXlGyroQueueEmpty();
void XlGyroQueuePush(XLGYRO_DATA_S *pData);
bool XlGyroQueueGet(XLGYRO_DATA_S *pData);
uint32_t IsXlGyroQueueItemsCount();

#endif /* XLGYROSERVER_H */
