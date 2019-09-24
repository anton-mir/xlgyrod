#ifndef XLGYRO_SERVER_H
#define XLGYRO_SERVER_H

#include <stdint.h>

#define XLGYRO_SERVER_PORT                  (11333)
#define XLGYRO_ACTIVE_CLIENTS_NUM           (32)

int CreateXlGyroServer();
void XlGyroServerSendToClients(uint8_t *pData, uint32_t len);

#endif /* XLGYRO_SERVER_H */
