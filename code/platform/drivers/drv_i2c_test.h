#ifndef __DRV_I2C_H__
#define __DRV_I2C_H__
#include "device.h"

struct i2c_gpio_platform_data {
	unsigned int	sda_pin;
	unsigned int	scl_pin;
	int		udelay;
	int		timeout;
	unsigned int	sda_is_open_drain:1;
	unsigned int	scl_is_open_drain:1;
	unsigned int	scl_is_output_only:1;
};

#if 0
struct i2c_client {
	unsigned short flags;		/* div., see below		*/
	unsigned short addr;		/* chip address - NOTE: 7bit	*/
					/* addresses are stored in the	*/
					/* _LOWER_ 7 bits		*/
	char name[I2C_NAME_SIZE];
	struct i2c_adapter *adapter;	/* the adapter we sit on	*/
	struct i2c_driver *driver;	/* and our access routines	*/
	struct device dev;		/* the device structure		*/
	int irq;			/* irq issued by device		*/

};

struct i2c_driver {

	/* Standard driver model interfaces */
	int (*probe)(struct i2c_client *, const struct i2c_device_id *);
	int (*remove)(struct i2c_client *);

	/* driver model interfaces that don't relate to enumeration  */
	void (*shutdown)(struct i2c_client *);
	int (*suspend)(struct i2c_client *, pm_message_t mesg);
	int (*resume)(struct i2c_client *);

	/* Alert callback, for example for the SMBus alert protocol.
	 * The format and meaning of the data value depends on the protocol.
	 * For the SMBus alert protocol, there is a single bit of data passed
	 * as the alert response's low bit ("event flag").
	 */
	void (*alert)(struct i2c_client *, unsigned int data);

	/* a ioctl like command that can be used to perform specific functions
	 * with the device.
	 */
	int (*command)(struct i2c_client *client, unsigned int cmd, void *arg);

	struct device_driver driver;
	const struct i2c_device_id *id_table;

};
#endif

DWORD drv_i2c_init(LPCSTR dwContext);
DWORD drv_i2c_open(DWORD dwData, DWORD dwAccess);
BOOL drv_i2c_close(DWORD dwData);
DWORD drv_i2c_read(DWORD dwData, LPVOID pBuf, DWORD Len);
DWORD drv_i2c_write(DWORD dwData, LPCVOID pBuf, DWORD Len);
BOOL drv_i2c_ioctl(DWORD dwContext, DWORD dwCode, LPVOID pInBuffer, DWORD inSize, LPVOID pOutBuffer, DWORD outSize, PDWORD pOutSize);
void drv_i2c_suspend(DWORD dwData);
void drv_i2c_resume(DWORD dwData);

#endif
