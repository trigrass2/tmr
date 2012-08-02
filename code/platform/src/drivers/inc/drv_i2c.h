#ifndef __DRV_I2C_H__
#define __DRV_I2C_H__
#include "device.h"

DWORD drv_i2c_init(LPCSTR dwContext);
DWORD drv_i2c_open(DWORD dwData, DWORD dwAccess);
BOOL drv_i2c_close(DWORD dwData);
DWORD drv_i2c_read(DWORD dwData, LPVOID pBuf, DWORD Len);
DWORD drv_i2c_write(DWORD dwData, LPCVOID pBuf, DWORD Len);
BOOL drv_i2c_ioctl(DWORD dwContext, DWORD dwCode, LPVOID pInBuffer, DWORD inSize, LPVOID pOutBuffer, DWORD outSize, PDWORD pOutSize);
void drv_i2c_suspend(DWORD dwData);
void drv_i2c_resume(DWORD dwData);

#endif
