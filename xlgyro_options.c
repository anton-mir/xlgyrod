#include <getopt.h>
#include <stddef.h>
#include <stdio.h>
#include <stdbool.h>
#include <stdlib.h>
#include "xlgyro_options.h"

#define DEFAULT_TTYNAME         "/dev/ttyUSB0"
#define DEFAULT_DEVIATION_HI    (double)(300.0)
#define DEFAULT_DEVIATION_LO    (double)(-300.0)
#define Z_AXIS_THD_HI           (double)(0.4)
#define Z_AXIS_THD_LO           (double)(-0.4)
#define DEFAULT_SERVER_PORT     (11333)
#define DEFAULT_DAEMON          (uint8_t)(0)
#define DEFAULT_TEST             "NONE"

static struct option long_options[] =
{
            {"dh",     required_argument, NULL,  0 },
            {"dl",     required_argument, NULL,  0 },
            {"zh",     required_argument, NULL,  0 },
            {"zl",     required_argument, NULL,  0 },
            {"tty",    required_argument, NULL,  0 },
            {"port",   required_argument, NULL,  0 },
            {"daemon", required_argument, NULL,  0 },
            {"test",   required_argument, NULL,  0 },
            {0,        0,                 0,     0 }
};

void XlGyroReadOptions(int argc, char *argv[], XLGYRO_READER_THREAD_PARAMS_S *pParams)
{
    int option = 0;
    int optionIdx = 0;

    if (pParams != NULL && argv != NULL)
    {
        pParams->ttyname = DEFAULT_TTYNAME;
        pParams->deviationHi = DEFAULT_DEVIATION_HI;
        pParams->deviationLo = DEFAULT_DEVIATION_LO;
        pParams->zAxisThdHi = Z_AXIS_THD_HI;
        pParams->zAxisThdLo = Z_AXIS_THD_LO;
        pParams->port = DEFAULT_SERVER_PORT;
        pParams->daemon = DEFAULT_DAEMON;
        pParams->ttytest = DEFAULT_TEST;

        while (true)
        {
            option = getopt_long(argc, argv, "", long_options, &optionIdx);

            if (option < 0)
            {
                break;
            }

            switch (optionIdx)
            {
                case 0:
                {
                    if (optarg != NULL)
                    {
                        pParams->deviationHi = atof(optarg);
                    }
                    break;
                }

                case 1:
                {
                    if (optarg != NULL)
                    {
                        pParams->deviationLo = atof(optarg);
                    }
                    break;
                }

                case 2:
                {
                    if (optarg != NULL)
                    {
                        pParams->zAxisThdHi = atof(optarg);
                    }
                    break;
                }

                case 3:
                {
                    if (optarg != NULL)
                    {
                        pParams->zAxisThdLo = atof(optarg);
                    }
                    break;
                }

                case 4:
                {
                    if (optarg != NULL)
                    {
                        pParams->ttyname = optarg;
                    }
                    break;
                }

                case 5:
                {
                    if (optarg != NULL)
                    {
                        pParams->port = atoi(optarg);
                    }
                    break;
                }

                case 6:
                {
                    if (optarg != NULL)
                    {
                        pParams->daemon = atoi(optarg);
                    }
                    break;
                }
                case 7:
                {
                    if (optarg != NULL)
                    {
                        pParams->ttytest = optarg;
                    }
                    break;
                }

                default:
                {
                    break;
                }
            }
        }

        printf("Current settings: \n");
        printf("ttyname     : %s\n",   pParams->ttyname);
        printf("deviationHi : %+lf\n", pParams->deviationHi);
        printf("deviationLo : %+lf\n", pParams->deviationLo);
        printf("zAxisThdHi  : %+lf\n", pParams->zAxisThdHi);
        printf("zAxisThdLo  : %+lf\n", pParams->zAxisThdLo);
        printf("port        : %d\n",   pParams->port);
        printf("daemon      : %d\n",   pParams->daemon);
        printf("test        : %s\n",   pParams->ttytest);
        printf("=========================================\n\n");
    }
}