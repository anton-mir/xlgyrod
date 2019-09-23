#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <stdbool.h>
#include <errno.h>
#include <fcntl.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>
#include "crc16.h"

#define LINEAR_ACCELERATION_RANGE_2         (0.000061)
#define LINEAR_ACCELERATION_RANGE_4         (0.000122)
#define LINEAR_ACCELERATION_RANGE_8         (0.000244)
#define LINEAR_ACCELERATION_RANGE_16        (0.000732)
#define ANGULAR_RATE_RANGE_245              (0.00875)
#define ANGULAR_RATE_RANGE_500              (0.0175)
#define ANGULAR_RATE_RANGE_2000             (0.07)
#define SLIDING_WINDOW_SIZE                 (32)
#define AVERAGED_BUF_SIZE                   (300)       // ~ 30s

#define DEFAULT_PORTNAME     "/dev/ttyUSB0"

#define DATA_BUF_SIZE               (1024 * 8)
#define PACKET_PREAMBULE            (0xAA55)
#define PACKET_PREAMBULE_HI         (0xAA)
#define PACKET_PREAMBULE_LO         (0x55)
#define MAX_SAMPLES                 (1024)

#define FIELD_SIZEOF(t, f)          (sizeof(((t*)0)->f))

#define DATA_PACKET_LEN(itms)  ( \
                                            sizeof(PACKET_HEADER_S) + \
                                            sizeof(RAW_DATA_S) * itms * 2 )

#define PACKET_PAYLOAD_LEN(itms)            (sizeof(RAW_DATA_S) * itms * 2)

typedef enum
{
    E_X_AXIS = 0,
    E_Y_AXIS,
    E_Z_AXIS,
    E_AXIS_COUNT
} AXISES;

typedef struct __attribute__((packed, aligned(1))) RAW_DATA_STRUCT
{
    int16_t rawData[E_AXIS_COUNT];
} RAW_DATA_S;

typedef struct __attribute__((packed, aligned(1))) PACKET_HEADER_STRUCT
{
    uint16_t preambule1;
    uint16_t preambule2;
    uint16_t samples;
} PACKET_HEADER_S;

int aa = sizeof(PACKET_HEADER_S);

typedef struct __attribute__((packed, aligned(1))) DATA_PACKET_STRUCT
{
    PACKET_HEADER_S header;
    struct
    {
        RAW_DATA_S aValue;
        RAW_DATA_S gValue;
    } agBufs[DATA_BUF_SIZE];
} DATA_PACKET_S;

typedef struct AXIS_DOUBLE_VALUE_STRUCT
{
    double axisValue[E_AXIS_COUNT];
} AXIS_DOUBLE_VALUE_S;

typedef struct RAW_SLIDING_WINDOW_STRUCT
{
    RAW_DATA_S window[SLIDING_WINDOW_SIZE];
    int32_t windowSum[E_AXIS_COUNT];
    RAW_DATA_S min;
    RAW_DATA_S max;
    bool inited;
} RAW_SLIDING_WINDOW_S;

typedef struct AVERAGED_BUF_STRUCT
{
    AXIS_DOUBLE_VALUE_S values[AVERAGED_BUF_SIZE];
    uint32_t idx;
    bool init;
} AVERAGED_BUF_S;

/**
 * @brief Defines accelerometer resolution value
 * @var accelResolution
 * @note For now accelerometer of LSM9DS1 is configured as
 *       LINEAR_ACCELERATION_RANGE_2 and this value is not changed
 */
const double accelResolution = LINEAR_ACCELERATION_RANGE_2;

/**
 * @brief Defines gyroscope resolution value
 * @var gyroResolution
 * @note For now gyroscope of LSM9DS1 is configured as
 *       ANGULAR_RATE_RANGE_245 and this value is not changed
 */
const double gyroResolution = ANGULAR_RATE_RANGE_245;

static uint8_t rawDataBuf[DATA_BUF_SIZE];

static DATA_PACKET_S rxPacket = { 0 };

static RAW_SLIDING_WINDOW_S aSlidingWindow = { 0 };
static RAW_SLIDING_WINDOW_S gSlidingWindow = { 0 };
static AVERAGED_BUF_S aAveragedBuf = {0};

static void setMin(RAW_SLIDING_WINDOW_S *pWindow, RAW_DATA_S *pRawData)
{
    if (pRawData->rawData[E_X_AXIS] < pWindow->min.rawData[E_X_AXIS])
    {
        pWindow->min.rawData[E_X_AXIS] = pRawData->rawData[E_X_AXIS];
    }

    if (pRawData->rawData[E_Y_AXIS] < pWindow->min.rawData[E_Y_AXIS])
    {
        pWindow->min.rawData[E_Y_AXIS] = pRawData->rawData[E_Y_AXIS];
    }

    if (pRawData->rawData[E_Z_AXIS] < pWindow->min.rawData[E_Z_AXIS])
    {
        pWindow->min.rawData[E_Z_AXIS] = pRawData->rawData[E_Z_AXIS];
    }
}

static void setMax(RAW_SLIDING_WINDOW_S *pWindow, RAW_DATA_S *pRawData)
{
    if (pRawData->rawData[E_X_AXIS] > pWindow->max.rawData[E_X_AXIS])
    {
        pWindow->max.rawData[E_X_AXIS] = pRawData->rawData[E_X_AXIS];
    }

    if (pRawData->rawData[E_Y_AXIS] > pWindow->max.rawData[E_Y_AXIS])
    {
        pWindow->max.rawData[E_Y_AXIS] = pRawData->rawData[E_Y_AXIS];
    }

    if (pRawData->rawData[E_Z_AXIS] > pWindow->max.rawData[E_Z_AXIS])
    {
        pWindow->max.rawData[E_Z_AXIS] = pRawData->rawData[E_Z_AXIS];
    }
}

static void slidingWindowGetMin(RAW_SLIDING_WINDOW_S *pWindow, RAW_DATA_S *pMin)
{
    pMin->rawData[E_X_AXIS] = pWindow->min.rawData[E_X_AXIS];
    pMin->rawData[E_Y_AXIS] = pWindow->min.rawData[E_Y_AXIS];
    pMin->rawData[E_Z_AXIS] = pWindow->min.rawData[E_Z_AXIS];
}

static void slidingWindowGetMax(RAW_SLIDING_WINDOW_S *pWindow, RAW_DATA_S *pMin)
{
    pMin->rawData[E_X_AXIS] = pWindow->max.rawData[E_X_AXIS];
    pMin->rawData[E_Y_AXIS] = pWindow->max.rawData[E_Y_AXIS];
    pMin->rawData[E_Z_AXIS] = pWindow->max.rawData[E_Z_AXIS];
}

static void slidingWindowPush(RAW_SLIDING_WINDOW_S *pWindow, RAW_DATA_S *pValue)
{
    pWindow->windowSum[E_X_AXIS] = 0;
    pWindow->windowSum[E_Y_AXIS] = 0;
    pWindow->windowSum[E_Z_AXIS] = 0;

    pWindow->min.rawData[E_X_AXIS] = INT16_MAX;
    pWindow->min.rawData[E_Y_AXIS] = INT16_MAX;
    pWindow->min.rawData[E_Z_AXIS] = INT16_MAX;

    pWindow->max.rawData[E_X_AXIS] = INT16_MIN;
    pWindow->max.rawData[E_Y_AXIS] = INT16_MIN;
    pWindow->max.rawData[E_Z_AXIS] = INT16_MIN;

    if (pWindow->inited != true)
    {
        /* If sliding window is not inited - fill it with frist given value */
        for (uint32_t i = 0; i < SLIDING_WINDOW_SIZE; ++i)
        {
            memcpy(&pWindow->window[i], pValue, sizeof(RAW_DATA_S));

            setMin(pWindow, &pWindow->window[i]);
            setMax(pWindow, &pWindow->window[i]);

            pWindow->windowSum[E_X_AXIS] += pWindow->window[i].rawData[E_X_AXIS];
            pWindow->windowSum[E_Y_AXIS] += pWindow->window[i].rawData[E_Y_AXIS];
            pWindow->windowSum[E_Z_AXIS] += pWindow->window[i].rawData[E_Z_AXIS];
        }

        pWindow->inited = true;
    }
    else
    {
        for (uint32_t i = (SLIDING_WINDOW_SIZE - 1); i > 0; --i)
        {
            pWindow->window[i] = pWindow->window[i - 1];

            setMin(pWindow, &pWindow->window[i]);
            setMax(pWindow, &pWindow->window[i]);

            pWindow->windowSum[E_X_AXIS] += pWindow->window[i].rawData[E_X_AXIS];
            pWindow->windowSum[E_Y_AXIS] += pWindow->window[i].rawData[E_Y_AXIS];
            pWindow->windowSum[E_Z_AXIS] += pWindow->window[i].rawData[E_Z_AXIS];
        }

        memcpy(&pWindow->window[0], pValue, sizeof(RAW_DATA_S));

        pWindow->windowSum[E_X_AXIS] += pWindow->window[0].rawData[E_X_AXIS];
        pWindow->windowSum[E_Y_AXIS] += pWindow->window[0].rawData[E_Y_AXIS];
        pWindow->windowSum[E_Z_AXIS] += pWindow->window[0].rawData[E_Z_AXIS];
    }
}

static void accelRawDataAveraged(RAW_DATA_S *pAccelAveraged)
{
    if (pAccelAveraged != NULL)
    {
        pAccelAveraged->rawData[E_X_AXIS] = aSlidingWindow.windowSum[E_X_AXIS] / SLIDING_WINDOW_SIZE;
        pAccelAveraged->rawData[E_Y_AXIS] = aSlidingWindow.windowSum[E_Y_AXIS] / SLIDING_WINDOW_SIZE;
        pAccelAveraged->rawData[E_Z_AXIS] = aSlidingWindow.windowSum[E_Z_AXIS] / SLIDING_WINDOW_SIZE;
    }
}

static void gyroRawDataAveraged(RAW_DATA_S *pGyroAveraged)
{
    if (pGyroAveraged != NULL)
    {
        pGyroAveraged->rawData[E_X_AXIS] = gSlidingWindow.windowSum[E_X_AXIS] / SLIDING_WINDOW_SIZE;
        pGyroAveraged->rawData[E_Y_AXIS] = gSlidingWindow.windowSum[E_Y_AXIS] / SLIDING_WINDOW_SIZE;
        pGyroAveraged->rawData[E_Z_AXIS] = gSlidingWindow.windowSum[E_Z_AXIS] / SLIDING_WINDOW_SIZE;
    }
}

static void accelCalc(RAW_DATA_S *pRawData, AXIS_DOUBLE_VALUE_S *pValue)
{
    pValue->axisValue[E_X_AXIS] = pRawData->rawData[E_X_AXIS] * accelResolution;
    pValue->axisValue[E_Y_AXIS] = pRawData->rawData[E_Y_AXIS] * accelResolution;
    pValue->axisValue[E_Z_AXIS] = pRawData->rawData[E_Z_AXIS] * accelResolution;
}

static void gyroCalc(RAW_DATA_S *pRawData, AXIS_DOUBLE_VALUE_S *pValue)
{
    pValue->axisValue[E_X_AXIS] = pRawData->rawData[E_X_AXIS] * gyroResolution;
    pValue->axisValue[E_Y_AXIS] = pRawData->rawData[E_Y_AXIS] * gyroResolution;
    pValue->axisValue[E_Z_AXIS] = pRawData->rawData[E_Z_AXIS] * gyroResolution;
}

static void averagedBufPush(AVERAGED_BUF_S *pBuf, AXIS_DOUBLE_VALUE_S *pValue)
{
    if (pBuf->init == false)
    {
        for (uint32_t i = 0; i < AVERAGED_BUF_SIZE; ++i)
        {
            pBuf->values[i].axisValue[E_X_AXIS] = pValue->axisValue[E_X_AXIS];
            pBuf->values[i].axisValue[E_Y_AXIS] = pValue->axisValue[E_Y_AXIS];
            pBuf->values[i].axisValue[E_Z_AXIS] = pValue->axisValue[E_Z_AXIS];
        }
        pBuf->init = true;
    }

    pBuf->values[pBuf->idx].axisValue[E_X_AXIS] = pValue->axisValue[E_X_AXIS];
    pBuf->values[pBuf->idx].axisValue[E_Y_AXIS] = pValue->axisValue[E_Y_AXIS];
    pBuf->values[pBuf->idx].axisValue[E_Z_AXIS] = pValue->axisValue[E_Z_AXIS];

    pBuf->idx++;
    if (pBuf->idx >= AVERAGED_BUF_SIZE)
    {
        pBuf->idx = 0;
    }
}

static void getTotalAveraged(AVERAGED_BUF_S *pBuf, AXIS_DOUBLE_VALUE_S *pTotal)
{
    double xSum = 0;
    double ySum = 0;
    double zSum = 0;

    for (uint32_t i = 0; i < AVERAGED_BUF_SIZE; ++i)
    {
        xSum += pBuf->values[i].axisValue[E_X_AXIS];
        ySum += pBuf->values[i].axisValue[E_Y_AXIS];
        zSum += pBuf->values[i].axisValue[E_Z_AXIS];
    }

    pTotal->axisValue[E_X_AXIS] = xSum / AVERAGED_BUF_SIZE;
    pTotal->axisValue[E_Y_AXIS] = ySum / AVERAGED_BUF_SIZE;
    pTotal->axisValue[E_Z_AXIS] = zSum / AVERAGED_BUF_SIZE;
}

static bool isObstacle()
{
    bool ret = false;
    RAW_DATA_S aData = { 0 };
    RAW_DATA_S aMin = { 0 };
    RAW_DATA_S aMax = { 0 };
    AXIS_DOUBLE_VALUE_S accelValue = { 0 };
    AXIS_DOUBLE_VALUE_S aMinValue = { 0 };
    AXIS_DOUBLE_VALUE_S aMaxValue = { 0 };
    AXIS_DOUBLE_VALUE_S gyroValue = { 0 };
    AXIS_DOUBLE_VALUE_S aTotalAveraged= { 0 };
    double deviation = 0;

    accelRawDataAveraged(&aData);
    accelCalc(&aData, &accelValue);
    averagedBufPush(&aAveragedBuf, &accelValue);
    slidingWindowGetMin(&aSlidingWindow, &aMin);
    slidingWindowGetMax(&aSlidingWindow, &aMax);
    accelCalc(&aMin, &aMinValue);
    accelCalc(&aMax, &aMaxValue);
    getTotalAveraged(&aAveragedBuf, &aTotalAveraged);

    printf("Acc: [%+.6f]; [%+.6f]; [%+.6f]; \n",
                accelValue.axisValue[E_X_AXIS],
                accelValue.axisValue[E_Y_AXIS],
                accelValue.axisValue[E_Z_AXIS]);

    if (accelValue.axisValue[E_Z_AXIS] > 0.4 || accelValue.axisValue[E_Z_AXIS] < -0.4)
    {
        ret = true;
    }

    deviation = aTotalAveraged.axisValue[E_Z_AXIS] - accelValue.axisValue[E_Z_AXIS];
    deviation /= aTotalAveraged.axisValue[E_Z_AXIS];
    if (deviation > 150 || deviation < -150)
    {
        ret = true;
    }

    return ret;
}

static int setInterfaceAttribs(int fd, int speed, int parity)
{
    struct termios tty;
    memset (&tty, 0, sizeof tty);
    if (tcgetattr (fd, &tty) != 0)
    {
        int err = errno;
        printf("ERROR: tcgetattr: %s (%d)\n", strerror(err), err);
        return -1;
    }

    cfsetospeed(&tty, speed);
    cfsetispeed(&tty, speed);

    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;     // 8-bit chars
    // disable IGNBRK for mismatched speed tests; otherwise receive break
    // as \000 chars
    tty.c_iflag &= ~IGNBRK;         // disable break processing
    tty.c_lflag = 0;                // no signaling chars, no echo,
                                    // no canonical processing
    tty.c_oflag = 0;                // no remapping, no delays
    tty.c_cc[VMIN]  = 0;            // read doesn't block
    tty.c_cc[VTIME] = 200;           // 2.0 seconds read timeout // should be 20

    tty.c_iflag &= ~(IXON | IXOFF | IXANY); // shut off xon/xoff ctrl

    tty.c_cflag |= (CLOCAL | CREAD);// ignore modem controls,
                                    // enable reading
    tty.c_cflag &= ~(PARENB | PARODD);      // shut off parity
    tty.c_cflag |= parity;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CRTSCTS;

    if (tcsetattr(fd, TCSANOW, &tty) != 0)
    {
        int err = errno;
        printf("ERROR: tcsetattr: %s (%d)\n", strerror(err), err);
        return -1;
    }
    return 0;
}

static void setBlocking(int fd, int shouldBlock)
{
    struct termios tty;
    memset (&tty, 0, sizeof tty);
    if (tcgetattr (fd, &tty) != 0)
    {
        int err = errno;
        printf("ERROR: tggetattr: %s (%d)\n", strerror(err), err);
        return;
    }

    tty.c_cc[VMIN]  = shouldBlock ? 1 : 0;
    tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout

    if (tcsetattr(fd, TCSANOW, &tty) != 0)
    {
        int err = errno;
        printf("ERROR: setting term attributes: %s (%d)\n", strerror(err), err);
    }
}

static int32_t findPacketStartIdx(uint8_t *pData, uint32_t len)
{
    uint32_t ret = -1;
    if (pData == NULL || len == 0)
    {
        return 0;
    }

    for (uint32_t i = 0; i < (len - 3); ++i)
    {
        if ( pData[i] == PACKET_PREAMBULE_LO &&
             pData[i + 1] == PACKET_PREAMBULE_HI &&
             pData[i + 2] == PACKET_PREAMBULE_LO &&
             pData[i + 3] == PACKET_PREAMBULE_HI )
        {
            ret = i;
            break;
        }
    }

    return ret;
}

static void shiftBuf(uint8_t *pInBuf, uint32_t fromIdx, uint32_t toIdx)
{
    uint32_t i = 0;
    uint32_t j = 0;

    if (pInBuf != NULL)
    {
        for (i = fromIdx, j = toIdx; (i < DATA_BUF_SIZE && j < DATA_BUF_SIZE); ++i, ++j)
        {
            pInBuf[j] = pInBuf[i];
        }
    }
}

static void processReceivedPacket(DATA_PACKET_S *pPacket, uint32_t samples)
{
    RAW_DATA_S aData = { 0 };
    RAW_DATA_S gData = { 0 };

    for (int i = 0; i < samples; ++i)
    {
        aData.rawData[E_X_AXIS] = pPacket->agBufs[i].aValue.rawData[E_X_AXIS];
        aData.rawData[E_Y_AXIS] = pPacket->agBufs[i].aValue.rawData[E_Y_AXIS];
        aData.rawData[E_Z_AXIS] = pPacket->agBufs[i].aValue.rawData[E_Z_AXIS];
        slidingWindowPush(&aSlidingWindow, &aData);

        // gData.rawData[E_X_AXIS] = pPacket->agBufs[i].gValue.rawData[E_X_AXIS];
        // gData.rawData[E_Y_AXIS] = pPacket->agBufs[i].gValue.rawData[E_Y_AXIS];
        // gData.rawData[E_Z_AXIS] = pPacket->agBufs[i].gValue.rawData[E_Z_AXIS];
        // slidingWindowPush(&gSlidingWindow, &gData);
    }

    obstacle = isObstacle();
    if (obstacle)
    {
        printf("OOOOBBBSSSSTAAACCCLEEEEE\n");
    }

    // gyroRawDataAveraged(&gData);
    // gyroCalc(&gData, &gyroValue);

    // printf("=========================================================================\n");
    // printf("Min: [%+.6f]; [%+.6f]; [%+.6f]; \n",
    //             aMinValue.axisValue[E_X_AXIS],
    //             aMinValue.axisValue[E_Y_AXIS],
    //             aMinValue.axisValue[E_Z_AXIS]);

    // printf("Acc: [%+.6f]; [%+.6f]; [%+.6f]; \n",
    //             accelValue.axisValue[E_X_AXIS],
    //             accelValue.axisValue[E_Y_AXIS],
    //             accelValue.axisValue[E_Z_AXIS]);

    // printf("Max: [%+.6f]; [%+.6f]; [%+.6f]; \n",
    //             accelValue.axisValue[E_X_AXIS],
    //             accelValue.axisValue[E_Y_AXIS],
    //             accelValue.axisValue[E_Z_AXIS]);

    // printf("Avr: [%+.6f]; [%+.6f]; [%+.6f]; \n",
    //             aTotalAveraged.axisValue[E_X_AXIS],
    //             aTotalAveraged.axisValue[E_Y_AXIS],
    //             aTotalAveraged.axisValue[E_Z_AXIS]);


    // printf("G: [%+.6f]; [%+.6f]; [%+.6f]\n",
    //             gyroValue.axisValue[E_X_AXIS],
    //             gyroValue.axisValue[E_Y_AXIS],
    //             gyroValue.axisValue[E_Z_AXIS]);
}

int main(int argc, char *argv[])
{
    uint32_t appendIdx = 0;
    uint32_t processIdx = 0;
    uint32_t unprocessedBytes = 0;
    char *portname = NULL;

    if (argc > 1)
    {
        portname = argv[1];
    }
    else
    {
        portname = DEFAULT_PORTNAME;
    }

    int fd = open (portname, O_RDWR | O_NOCTTY | O_SYNC);
    if (fd < 0)
    {
        int err = errno;
        printf("ERROR: open() failed; error: %s\n", strerror(err));
        return fd;
    }
    else
    {
        printf("Opened: %s\n", portname);
    }

    setInterfaceAttribs (fd, B115200, 0);
    // setBlocking (fd, 0);

    bool done = false;
    do
    {
        ssize_t size = read(fd, &rawDataBuf[appendIdx], (DATA_BUF_SIZE - unprocessedBytes));
        if (size > 0)
        {
            unprocessedBytes += size;
            appendIdx += size;

            do
            {
                int32_t packetStart = findPacketStartIdx(&rawDataBuf[processIdx], unprocessedBytes);
                if (packetStart < 0)
                {
                    appendIdx = 0;
                    processIdx = 0;
                    unprocessedBytes = 0;
                    break;
                }

                processIdx = packetStart;
                memcpy(&rxPacket.header, &rawDataBuf[processIdx], sizeof(PACKET_HEADER_S));
                if (rxPacket.header.samples >= MAX_SAMPLES)
                {
                    processIdx += sizeof(PACKET_HEADER_S);
                    shiftBuf(rawDataBuf, processIdx, 0);

                    if (appendIdx > processIdx)
                    {
                        appendIdx -= processIdx;
                    }

                    if (unprocessedBytes > processIdx)
                    {
                        unprocessedBytes -= processIdx;
                    }

                    processIdx = 0;

                    break;
                }
                uint32_t packetLen = DATA_PACKET_LEN(rxPacket.header.samples);

                if ( processIdx + packetLen + 4 > unprocessedBytes)
                {
                    /* We do not received completed package yet.
                     * Wait for next chunk */
                    break;
                }

                // uint16_t crc16 = CalcCrc16(&rawDataBuf[processIdx], packetLen);
                if ((processIdx + packetLen + 4) >= DATA_BUF_SIZE)
                {
                    break;
                }

                if ( rawDataBuf[processIdx + packetLen] != 0xA5 ||
                     rawDataBuf[processIdx + packetLen + 1] != 0xA5 ||
                     rawDataBuf[processIdx + packetLen + 2] != 0xA5 ||
                     rawDataBuf[processIdx + packetLen + 2] != 0xA5 )
                {
                    processIdx += sizeof(PACKET_HEADER_S);
                    shiftBuf(rawDataBuf, processIdx, 0);

                    if (appendIdx > processIdx)
                    {
                        appendIdx -= processIdx;
                    }

                    if (unprocessedBytes > processIdx)
                    {
                        unprocessedBytes -= processIdx;
                    }

                    processIdx = 0;

                    break;
                }

                memcpy(
                    &rxPacket.agBufs,
                    &rawDataBuf[processIdx + sizeof(PACKET_HEADER_S)],
                    packetLen - sizeof(PACKET_HEADER_S));

                processReceivedPacket(&rxPacket, rxPacket.header.samples);

                processIdx += packetLen + 2;
                shiftBuf(rawDataBuf, processIdx, 0);

                if (appendIdx >= processIdx)
                {
                    appendIdx -= processIdx;
                }
                else
                {
                    appendIdx = 0;
                }

                if (unprocessedBytes >= processIdx)
                {
                    unprocessedBytes -= processIdx;
                }
                else
                {
                    unprocessedBytes = 0;
                }

                processIdx = 0;

            } while (true);
        }

    } while (done != true);

    return 0;
}