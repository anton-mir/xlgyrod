#include <errno.h>
#include <unistd.h>
#include <sys/time.h>
#include <stdbool.h>
#include <pthread.h>
#include <string.h>
#include<stdlib.h>
#include <stdio.h>
#include <termios.h>
#include <fcntl.h>
#include "xlgyro_data_processor.h"
#include "xlgyro_server.h"
#include "xlgyro_reader.h"
#include "xlgyro_options.h"

#define LINEAR_ACCELERATION_RANGE_2         (0.000061)
#define LINEAR_ACCELERATION_RANGE_4         (0.000122)
#define LINEAR_ACCELERATION_RANGE_8         (0.000244)
#define LINEAR_ACCELERATION_RANGE_16        (0.000732)
#define ANGULAR_RATE_RANGE_245              (0.00875)
#define ANGULAR_RATE_RANGE_500              (0.0175)
#define ANGULAR_RATE_RANGE_2000             (0.07)
#define SLIDING_WINDOW_SIZE                 (32)
#define AVERAGED_BUF_SIZE                   (300)       // ~ 30s

#define DATA_BUF_SIZE               (1024 * 8)
#define PACKET_PREAMBULE            (0xAA55)
#define PACKET_PREAMBULE_HI         (0xAA)
#define PACKET_PREAMBULE_LO         (0x55)
#define MAX_SAMPLES                 (1024)
#define TRAILER_LEN                 (4)
#define TRAILER_VALUE               (0xA5)

#define DATA_PACKET_LEN(itms)  ( \
                                            sizeof(PACKET_HEADER_S) + \
                                            sizeof(RAW_DATA_S) * itms * 2 )

#define FIELD_SIZEOF(t, f)          (sizeof(((t*)0)->f))
#define PREAMBULE_LEN()             ( \
                                        FIELD_SIZEOF(PACKET_HEADER_S, preambule1) + \
                                        FIELD_SIZEOF(PACKET_HEADER_S, preambule2) )

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

typedef struct __attribute__((packed, aligned(1))) DATA_PACKET_STRUCT
{
    PACKET_HEADER_S header;
    struct
    {
        RAW_DATA_S aValue;
        RAW_DATA_S gValue;
    } agBufs[DATA_BUF_SIZE];
} DATA_PACKET_S;

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

typedef struct READ_BUF_CTRL_STRUCT
{
    uint32_t appendIdx;
    uint32_t processIdx;
    uint32_t packetLen;
    uint8_t *pReadBuf;
    uint32_t readBufLen;
} READ_BUF_CTRL_S;

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

static AVERAGED_BUF_S aAveragedBuf = { 0 };

static pthread_t xlgyroReaderTh;

static pthread_t xlgyroTestTh;

static READ_BUF_CTRL_S bufCtrl = { 0 };

static XLGYRO_READER_THREAD_PARAMS_S params = { 0 };

static int serialInterfaceInit(int fd, int speed, int parity);
static int32_t findPacketStartIdx(uint8_t *pData, uint32_t idxOffset, uint32_t len);
static void shiftBuf(uint8_t *pInBuf, uint32_t fromIdx, uint32_t toIdx);
static bool isPacketStaffed(READ_BUF_CTRL_S *pBufCtrl);
static bool isPacketLenValid(READ_BUF_CTRL_S *pBufCtrl);
static bool isTrailerValid(uint8_t *pData, READ_BUF_CTRL_S *pBufCtrl);
static void processReceivedPacket(DATA_PACKET_S *pPacket, uint32_t samples);
static void slidingWindowPush(RAW_SLIDING_WINDOW_S *pWindow, RAW_DATA_S *pValue);
static void setMin(RAW_SLIDING_WINDOW_S *pWindow, RAW_DATA_S *pRawData);
static void setMax(RAW_SLIDING_WINDOW_S *pWindow, RAW_DATA_S *pRawData);
static void accelRawDataAveraged(RAW_DATA_S *pAccelAveraged);
static void accelRawDataAveraged(RAW_DATA_S *pAccelAveraged);
static void gyroRawDataAveraged(RAW_DATA_S *pGyroAveraged);
static void accelCalc(RAW_DATA_S *pRawData, AXIS_DOUBLE_VALUE_S *pValue);
static void gyroCalc(RAW_DATA_S *pRawData, AXIS_DOUBLE_VALUE_S *pValue);
static void averagedBufPush(AVERAGED_BUF_S *pBuf, AXIS_DOUBLE_VALUE_S *pValue);
static void getTotalAveraged(AVERAGED_BUF_S *pBuf, AXIS_DOUBLE_VALUE_S *pTotal);
static void getXlGyroData(XLGYRO_DATA_S *pData);
static void slidingWindowGetMin(RAW_SLIDING_WINDOW_S *pWindow, RAW_DATA_S *pMin);
static void slidingWindowGetMax(RAW_SLIDING_WINDOW_S *pWindow, RAW_DATA_S *pMin);
static int sendToXlGyroServer(XLGYRO_DATA_S *pData);


static int serialInterfaceInit(int fd, int speed, int parity)
{
    struct termios config;
    int status = 0;

    memset(&config, 0, sizeof config);

    /* Check if the file descriptor is pointing to a TTY device or not. */
    status = isatty(fd);
    if (status == 0)
    {
        int err = errno;
        printf(
            "[XLGYRODREADER]: file descriptor is not tty; fd: %d; "
            "error: %s (%d)\n",
            fd,
            strerror(err),
            err);
        return -1;
    }

    status = tcgetattr(fd, &config);
    if (status < 0)
    {
        int err = errno;
        printf("[XLGYRODREADER]: tcgetattr() failed; error: %s (%d)\n", strerror(err), err);
        return -1;
    }

    /* Input flags - Turn off input processing
     * convert break to null byte, no CR to NL translation,
     * no NL to CR translation, don't mark parity errors or breaks
     * no input parity check, don't strip high bit off */
    config.c_iflag &= ~(IGNBRK | BRKINT | ICRNL | INLCR | PARMRK | INPCK | ISTRIP);
    /* Disable xon/xoff ctrl */
    config.c_iflag &= ~(IXON | IXOFF | IXANY);

    /* Output flags - Turn off output processing
     * no CR to NL translation, no NL to CR-NL translation,
     * no NL to CR translation, no column 0 CR suppression,
     * no Ctrl-D suppression, no fill characters, no case mapping,
     * no local output processing */
    config.c_oflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON);

    /* no signaling chars, no echo,
     * no canonical processing
     * no remapping, no delays */
    config.c_lflag = 0;

    /* Turn off character processing
     * clear current char size mask, no parity checking,
     * no output processing, force 8 bit input */
    config.c_cflag &= ~(CSIZE);
    config.c_cflag |= CS8;
    /* gnore modem controls,
     * nable reading */
    config.c_cflag |= (CLOCAL | CREAD);
    /* Disable parity */
    config.c_cflag &= ~(PARENB | PARODD);
    config.c_cflag |= parity;
    config.c_cflag &= ~(CSTOPB | CRTSCTS);

    config.c_cc[VMIN]  = 0xFF;
    config.c_cc[VTIME] = 15;    // 15 * 10ms = 150ms

    status = cfsetispeed(&config, speed);
    if (status < 0)
    {
        int err = errno;
        printf("[XLGYRODREADER]: cfsetispeed() failed; error: %s (%d)\n", strerror(err), err);
        return -1;
    }

    status = cfsetospeed(&config, speed);
    if (status < 0)
    {
        int err = errno;
        printf("[XLGYRODREADER]: cfsetospeed() failed; error: %s (%d)\n", strerror(err), err);
        return -1;
    }

    status = tcsetattr(fd, TCSANOW, &config);
    if (status < 0)
    {
        int err = errno;
        printf("[XLGYRODREADER]: tcsetattr() failed; error: %s (%d)\n", strerror(err), err);
        return -1;
    }

    return 0;
}

static int32_t findPacketStartIdx(uint8_t *pData, uint32_t idxOffset, uint32_t len)
{
    int32_t ret = -1;
    int32_t idx = 0;

    do
    {

        if (pData == NULL)
        {
            break;
        }

        if (len >= DATA_BUF_SIZE || len < 3)
        {
            break;
        }

        if (idxOffset >= DATA_BUF_SIZE)
        {
            break;
        }

        for (idx = idxOffset; idx < (len - 3); ++idx)
        {
            if ( pData[idx] == PACKET_PREAMBULE_LO &&
                 pData[idx + 1] == PACKET_PREAMBULE_HI &&
                 pData[idx + 2] == PACKET_PREAMBULE_LO &&
                 pData[idx + 3] == PACKET_PREAMBULE_HI )
            {
                ret = idx;
                break;
            }
        }
    } while (0);

    return ret;
}

static void shiftBuf(uint8_t *pInBuf, uint32_t fromIdx, uint32_t toIdx)
{
    uint32_t from = 0;
    uint32_t to = 0;

    if (pInBuf != NULL)
    {
        for (from = fromIdx, to = toIdx; (from < DATA_BUF_SIZE && to < DATA_BUF_SIZE); ++from, ++to)
        {
            pInBuf[to] = pInBuf[from];
        }
    }
}

static bool isPacketStaffed(READ_BUF_CTRL_S *pBufCtrl)
{
    bool ret = false;
    const uint32_t expectedLen = pBufCtrl->processIdx + pBufCtrl->packetLen + TRAILER_LEN;

    if (expectedLen <= pBufCtrl->appendIdx)
    {
        ret = true;
    }

    return ret;
}

static bool isPacketLenValid(READ_BUF_CTRL_S *pBufCtrl)
{
    bool ret = false;
    const uint32_t expectedLen = pBufCtrl->processIdx + pBufCtrl->packetLen + TRAILER_LEN;

    if (expectedLen <= DATA_BUF_SIZE)
    {
        ret = true;
    }

    return ret;
}

static bool isTrailerValid(uint8_t *pData, READ_BUF_CTRL_S *pBufCtrl)
{
    bool ret = false;
    uint32_t idx = 0;
    uint32_t count = 0;
    uint32_t offset = pBufCtrl->processIdx + pBufCtrl->packetLen;

    for (idx = 0; idx < TRAILER_LEN && idx < (DATA_BUF_SIZE - TRAILER_LEN); ++idx)
    {
        if (pData[offset + idx] == TRAILER_VALUE)
        {
            count++;
        }
    }

    if (count == TRAILER_LEN)
    {
        ret = true;
    }

    return ret;
}

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


static void slidingWindowPush(RAW_SLIDING_WINDOW_S *pWindow, RAW_DATA_S *pValue)
{
    uint32_t i = 0;

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
        for (i = 0; i < SLIDING_WINDOW_SIZE; ++i)
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

static void getXlGyroData(XLGYRO_DATA_S *pData)
{
    RAW_DATA_S aData = { 0 };
    RAW_DATA_S aMin = { 0 };
    RAW_DATA_S aMax = { 0 };

    if (pData != NULL)
    {
        accelRawDataAveraged(&aData);
        accelCalc(&aData, &pData->current);
        averagedBufPush(&aAveragedBuf, &pData->current);
        slidingWindowGetMin(&aSlidingWindow, &aMin);
        slidingWindowGetMax(&aSlidingWindow, &aMax);
        accelCalc(&aMin, &pData->min);
        accelCalc(&aMax, &pData->max);
        getTotalAveraged(&aAveragedBuf, &pData->averaged);
    }
}

static int sendToXlGyroServer(XLGYRO_DATA_S *pData)
{
    int ret = 0;

    XlGyroQueuePush(pData);

    return ret;
}

static void processReceivedPacket(DATA_PACKET_S *pPacket, uint32_t samples)
{
    RAW_DATA_S aData = { 0 };
    RAW_DATA_S gData = { 0 };
    XLGYRO_DATA_S xlGyroData = { 0 };
    int status = 0;

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

    getXlGyroData(&xlGyroData);
    status = sendToXlGyroServer(&xlGyroData);
    {
        int err = errno;
        if (status < 0)
        {
            printf("[XLGYRODREADER]: failed to send: %s (%d)\n", strerror(err), err);
        }
    }
}

static void *xlgyroReaderThread(void *arg)
{
    int portFd = 0;
    int status = 0;
    ssize_t size = 0;
    int32_t packetStartIdx = 0;

    if (arg != NULL)
    {
        memcpy(&params, arg, sizeof(XLGYRO_READER_THREAD_PARAMS_S));
    }

    portFd = open(params.ttyname, O_RDWR | O_NOCTTY);
    if (portFd < 0)
    {
        int err = errno;
        printf("[XLGYRODREADER]: open() failed; error: %s\n", strerror(err));
        printf("[XLGYRODREADER]: Connect device!\n");
        while ( (portFd = open(params.ttyname, O_RDWR | O_NOCTTY))<0) ;
    }

    printf("Opened: %s\n", params.ttyname);

    status = serialInterfaceInit(portFd, B115200, 0);
    if (status < 0)
    {
        return NULL;
    }

    while (true)
    {
        size = read(portFd, &rawDataBuf[bufCtrl.appendIdx], (DATA_BUF_SIZE - bufCtrl.appendIdx));
         if(size>0){
        if (size >= sizeof(PACKET_HEADER_S))
        {
            bufCtrl.appendIdx += size;

            do
            {
                packetStartIdx = findPacketStartIdx(
                                        rawDataBuf,
                                        bufCtrl.processIdx,
                                        bufCtrl.appendIdx);

                if (packetStartIdx < 0)
                {
                    shiftBuf(rawDataBuf, bufCtrl.appendIdx, 0);
                    bufCtrl.processIdx = 0;
                    bufCtrl.appendIdx = 0;
                    break;
                }

                bufCtrl.processIdx = packetStartIdx;

                memcpy(
                    &rxPacket.header,
                    &rawDataBuf[bufCtrl.processIdx],
                    sizeof(PACKET_HEADER_S));

                if (rxPacket.header.samples >= MAX_SAMPLES)
                {
                    /* Packet is corrupted. Skip it */
                    bufCtrl.processIdx += PREAMBULE_LEN();
                    shiftBuf(rawDataBuf, bufCtrl.processIdx, 0);

                    if (bufCtrl.appendIdx >= bufCtrl.processIdx)
                    {
                        bufCtrl.appendIdx -= bufCtrl.processIdx;
                    }

                    bufCtrl.processIdx = 0;
                    break;
                }

                bufCtrl.packetLen = DATA_PACKET_LEN(rxPacket.header.samples);
                if (isPacketLenValid(&bufCtrl) != true)
                {
                    /* Packet is  too long and it doesn't fit into buffer
                     * Just skip it */
                    bufCtrl.processIdx += PREAMBULE_LEN();
                    shiftBuf(rawDataBuf, bufCtrl.processIdx, 0);

                    if (bufCtrl.appendIdx >= bufCtrl.processIdx)
                    {
                        bufCtrl.appendIdx -= bufCtrl.processIdx;
                    }

                    bufCtrl.processIdx = 0;
                }

                if (isPacketStaffed(&bufCtrl) != true)
                {
                    /* We do not received whole package yet.
                     * Wait for next chunk */
                    break;
                }

                if (isTrailerValid(rawDataBuf, &bufCtrl) != true)
                {
                    /* Packet is corrupted. Skip it */
                    bufCtrl.processIdx += PREAMBULE_LEN();
                    shiftBuf(rawDataBuf, bufCtrl.processIdx, 0);

                    if (bufCtrl.appendIdx >= bufCtrl.processIdx)
                    {
                        bufCtrl.appendIdx -= bufCtrl.processIdx;
                    }

                    bufCtrl.processIdx = 0;
                }

                memcpy(
                    &rxPacket.agBufs,
                    &rawDataBuf[bufCtrl.processIdx + sizeof(PACKET_HEADER_S)],
                    bufCtrl.packetLen - sizeof(PACKET_HEADER_S));

                processReceivedPacket(&rxPacket, rxPacket.header.samples);

                bufCtrl.processIdx += bufCtrl.packetLen + TRAILER_LEN;
                shiftBuf(rawDataBuf, bufCtrl.processIdx, 0);
                if (bufCtrl.appendIdx >= bufCtrl.processIdx)
                {
                    bufCtrl.appendIdx -= bufCtrl.processIdx;
                }
                else
                {
                    bufCtrl.appendIdx = 0;
                }
                bufCtrl.processIdx = 0;
            } while (true);
        }
        }else{
            printf("[XLGYRODREADER]: Device disconnected!\n");
            printf("[XLGYRODREADER]: Connect device!\n");
            close(portFd);
            do {
                portFd = open(params.ttyname, O_RDWR | O_NOCTTY);
            } while (portFd<0);
            status = serialInterfaceInit(portFd, B115200, 0);
            printf("Opened: %s\n", params.ttyname);
        }

    }
}

int XlGyroReaderCreate(void *arg)
{
    return pthread_create(&xlgyroReaderTh, NULL, xlgyroReaderThread, arg);
}

static void *xlgyroTestThread(void *arg)
{
 
    int portFd = 0;
    int status = 0;
    ssize_t size = 0;
    int32_t packetStartIdx = 0;

    if (arg != NULL)
    {
        memcpy(&params, arg, sizeof(XLGYRO_READER_THREAD_PARAMS_S));
    }

    portFd = open(params.ttytest, O_RDWR | O_NOCTTY);
    if (portFd < 0)
    {
        int err = errno;
        printf("[XLGYRODTEST]: open() failed; error: %s\n", strerror(err));
        return NULL;
    }

    printf("Opened: %s\n", params.ttytest);
    
    status = serialInterfaceInit(portFd, B115200, 0);
    if (status < 0)
    {
        return NULL;
    }


    int  bytes_written  = 0;  	
    uint8_t txBuf[MAX_SAMPLES*6] = { 0 };
    DATA_PACKET_S payload[1] = {0};
    uint32_t payloadLen = 0;


    while (1)
    {
        //Generate paylode with random amount of aValue,gValue values
        generate_payload(payload);
        payloadLen = DATA_PACKET_LEN(payload[0].header.samples);
        encapsulateAsRaw(txBuf,payload,payloadLen);

        bytes_written = write(portFd, txBuf, sizeof(txBuf));

        //zeor out txBuf and payload before reusing
        memset(txBuf,0,payloadLen+4);
        memset(payload,0,payloadLen);

        printf("\n  Payload written to % s",params.ttytest);
        printf("\n  % d Bytes written to % s", bytes_written,params.ttytest);
        printf("\n +----------------------------------+\n\n");
        sleep(1);
    }
    close(portFd);/* Close the Serial port */
   
}


void generate_payload(DATA_PACKET_S *payload)
{
    payload[0].header.preambule1 = PACKET_PREAMBULE;
    payload[0].header.preambule2 = PACKET_PREAMBULE;
    int agBufLen = rand() % 100;
    
    for(uint32_t i = 0; i < agBufLen; i++)
    {
        payload[0].agBufs[i].aValue.rawData[E_X_AXIS] = (rand() << 8 | rand());
        payload[0].agBufs[i].aValue.rawData[E_Y_AXIS] = (rand() << 8 | rand());
        payload[0].agBufs[i].aValue.rawData[E_Z_AXIS] = (rand() << 8 | rand());
        payload[0].agBufs[i].gValue.rawData[E_X_AXIS] = (rand() << 8 | rand());
        payload[0].agBufs[i].gValue.rawData[E_Y_AXIS] = (rand() << 8 | rand());
        payload[0].agBufs[i].gValue.rawData[E_Z_AXIS] = (rand() << 8 | rand());
        payload[0].header.samples++;
    }
}

void encapsulateAsRaw(uint8_t raw_data[],DATA_PACKET_S *payload, uint32_t payloadLen)
{
    if ((payloadLen + 4) <= MAX_SAMPLES*6)   
    {
        memcpy(raw_data, (uint8_t*)&payload[0], payloadLen);
    }
    raw_data[payloadLen] = 0xA5;
    raw_data[payloadLen + 1] = 0xA5;
    raw_data[payloadLen + 2] = 0xA5;
    raw_data[payloadLen + 3] = 0xA5;
}

int XlGyroTestCreate(void *arg)
{
    return pthread_create(&xlgyroTestTh, NULL, xlgyroTestThread, arg);
}

