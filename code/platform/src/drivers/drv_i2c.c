#include "board.h"
#include "device.h"
#include "drv_i2c.h"

DWORD drv_i2c_init(LPCSTR dwContext)
{
    DWORD handle = 0;
    
    RETAILMSG(0, ("[DRV I2C]:drv_i2c_init() \r\n"));
    
    return handle;
}

DWORD drv_i2c_open(DWORD dwData, DWORD dwAccess)
{
    RETAILMSG(0, ("[DRV I2C]:drv_i2c_open() \r\n"));
    return dwData;
}

BOOL drv_i2c_close(DWORD dwData)
{
    BOOL rc = FALSE;
    
    RETAILMSG(0, ("[DRV I2C]:drv_i2c_close() \r\n"));
    
    return rc;
}

DWORD drv_i2c_read(DWORD dwData, LPVOID pBuf, DWORD Len)
{
    RETAILMSG(0, ("[DRV I2C]:drv_i2c_read() \r\n"));
    return dwData;
}

DWORD drv_i2c_write(DWORD dwData, LPCVOID pBuf, DWORD Len)
{
    RETAILMSG(0, ("[DRV I2C]:drv_i2c_write() \r\n"));
    return dwData;
}

BOOL drv_i2c_ioctl(DWORD dwContext, DWORD dwCode, LPVOID pInBuffer, DWORD inSize, LPVOID pOutBuffer, DWORD outSize, PDWORD pOutSize)
{
    BOOL rc = FALSE;
    
    RETAILMSG(0, ("[DRV I2C]:drv_i2c_ioctl() \r\n"));
    
    return rc;
}

void drv_i2c_suspend(DWORD dwData)
{
    RETAILMSG(0, ("[DRV I2C]:drv_i2c_suspend() \r\n"));
}

void drv_i2c_resume(DWORD dwData)
{
    RETAILMSG(0, ("[DRV I2C]:drv_i2c_resume() \r\n"));
}
