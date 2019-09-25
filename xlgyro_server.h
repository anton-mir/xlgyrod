#ifndef XLGYRO_SERVER_H
#define XLGYRO_SERVER_H

#include <stdint.h>

int XlGyroServerCreate(void *args);
void XlGyroServerSendToClients(uint8_t *pData, uint32_t len);

#endif /* XLGYRO_SERVER_H */
