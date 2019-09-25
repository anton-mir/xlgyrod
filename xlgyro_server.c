#include <errno.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <sys/time.h>
#include <stdbool.h>
#include <pthread.h>
#include <string.h>
#include <stdio.h>
#include "xlgyro_server.h"

#define CHECK_CONNECTION_BUF_SIZE       (32)

static int clientSocket[XLGYRO_ACTIVE_CLIENTS_NUM] = { 0 };
static pthread_t xlgyroServerTh;
static pthread_mutex_t socketMut = PTHREAD_MUTEX_INITIALIZER;
static uint8_t checkConnectionBuf[CHECK_CONNECTION_BUF_SIZE];

static void *xlgyroServerThread(void *arg)
{
    int maxSd = 0;
    int listenSocket = 0;
    int pendingSocket = 0;
    struct sockaddr_in srvAddr;
    struct sockaddr_in clientAddr;
    int socketOptions = 1;
    int status = 0;
    int selectStatus = 0;
    int client = 0;
    int addrlen = 0;
    int idx = 0;
    ssize_t readBytes = 0;

    //set of socket descriptors
    fd_set readfds;

    for (idx = 0; idx < XLGYRO_ACTIVE_CLIENTS_NUM; ++idx)
    {
        clientSocket[idx] = 0;
    }

    listenSocket = socket(AF_INET , SOCK_STREAM , 0);
    if (listenSocket < 0)
    {
        int err = errno;
        printf("[XLGYRODSERVER]: socket() failed; error: %s\n", strerror(err));
        return NULL;
    }

    status = setsockopt(listenSocket, SOL_SOCKET, SO_REUSEADDR, (char*)&socketOptions, sizeof(socketOptions));

    if (status < 0)
    {
        int err = errno;
        printf("[XLGYRODSERVER]: setsockopt() failed; error: %s\n", strerror(err));
        return NULL;
    }

    srvAddr.sin_family = AF_INET;
    srvAddr.sin_addr.s_addr = INADDR_ANY;
    srvAddr.sin_port = htons(XLGYRO_SERVER_PORT);

    status = bind(listenSocket, (struct sockaddr*)&srvAddr, sizeof(srvAddr));
    if (status < 0)
    {
        int err = errno;
        printf("[XLGYRODSERVER]: bind() failed; error: %s\n", strerror(err));
        return NULL;
    }

    printf("[XLGYRODSERVER]: listen on port: %d\n", XLGYRO_SERVER_PORT);

    status = listen(listenSocket, 2);
    if (status < 0)
    {
        int err = errno;
        printf("[XLGYRODSERVER]: listen() failed; error: %s\n", strerror(err));
        return NULL;
    }

    addrlen = sizeof(srvAddr);

    while (true)
    {
        FD_ZERO(&readfds);
        FD_SET(listenSocket, &readfds);
        maxSd = listenSocket;

        for (int i = 0; i < XLGYRO_ACTIVE_CLIENTS_NUM; ++i)
        {
            pendingSocket = clientSocket[i];

            /* If valid socket descriptor then add to read list */
            if (pendingSocket > 0)
            {
                FD_SET(pendingSocket , &readfds);
            }

            /* Highest file descriptor number, need it for the select function */
            if (pendingSocket > maxSd)
            {
                maxSd = pendingSocket;
            }
        }

        /* Wait for an activity on one of the sockets */
        selectStatus = select((maxSd + 1), &readfds , NULL , NULL , NULL);
        if ((selectStatus < 0) && (errno != EINTR))
        {
            int err = errno;
            printf("[XLGYRODSERVER]: select() failed; error: %s\n", strerror(err));
        }

        if (FD_ISSET(listenSocket, &readfds))
        {
            client = accept(listenSocket, (struct sockaddr *)&clientAddr, (socklen_t*)&addrlen);
            if (client < 0)
            {
                int err = errno;
                printf("[XLGYRODSERVER]: accept() failed; error: %s\n", strerror(err));
                continue;
            }

            printf(
                "[XLGYRODSERVER] incoming connection; "
                "ip: %s; "
                "port: %d\n",
                inet_ntoa(clientAddr.sin_addr),
                ntohs(clientAddr.sin_port));

            bool clientAdded = false;
            pthread_mutex_lock(&socketMut);
            for (idx = 0; idx < XLGYRO_ACTIVE_CLIENTS_NUM; ++idx)
            {
                if (clientSocket[idx] == 0)
                {
                    clientSocket[idx] = client;
                    clientAdded = true;
                    break;
                }
            }
            pthread_mutex_unlock(&socketMut);

            if (clientAdded != true)
            {
                printf("[XLGYRODSERVER]: client rejected; no free sockets\n");
                continue;
            }

            for (idx = 0; idx < XLGYRO_ACTIVE_CLIENTS_NUM; ++idx)
            {
                client = clientSocket[idx];

                if (FD_ISSET(client , &readfds))
                {
                    /* Check if it was for closing */
                    readBytes = read(client , checkConnectionBuf, CHECK_CONNECTION_BUF_SIZE);
                    if (readBytes == 0)
                    {
                        close(client);
                        pthread_mutex_lock(&socketMut);
                        clientSocket[idx] = 0;
                        pthread_mutex_unlock(&socketMut);
                    }
                }
            }
        }
    }
}

int XlGyroServerCreate(void *args)
{
    return pthread_create(&xlgyroServerTh, NULL, xlgyroServerThread, args);
}

void XlGyroServerSendToClients(uint8_t *pData, uint32_t len)
{
    uint32_t idx = 0;

    if (pData != NULL)
    {
        pthread_mutex_lock(&socketMut);
        for (idx = 0; idx < XLGYRO_ACTIVE_CLIENTS_NUM; ++idx)
        {
            /* If it's active client */
            if (clientSocket[idx] != 0)
            {
                send(clientSocket[idx], pData, len, 0);
            }
        }
        pthread_mutex_unlock(&socketMut);
    }
}

