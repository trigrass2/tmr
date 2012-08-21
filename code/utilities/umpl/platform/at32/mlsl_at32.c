/*
 $License:
    Copyright (C) 2010 InvenSense Corporation, All Rights Reserved.
 $
 */

/******************************************************************************
 * $Id: mlsl_linux.c 4639 2011-01-28 04:39:15Z yserlin $
 *****************************************************************************/

/** 
 *  @defgroup MLSL (Motion Library - Serial Layer)
 *  @brief  The Motion Library System Layer provides the Motion Library the 
 *          interface to the system functions.
 *
 *  @{
 *      @file   mlsl_at32.c
 *      @brief  The Motion Library System Layer.
 *
 */

/* ------------------ */
/* - Include Files. - */
/* ------------------ */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "log.h"
#undef MPL_LOG_TAG
#define MPL_LOG_TAG "mlsl-at32"

#include "mpu.h"

#if defined CONFIG_MPU_SENSORS_MPU6050A2
#    include "mpu6050a2.h"
#elif defined CONFIG_MPU_SENSORS_MPU6050B1
#    include "mpu6050b1.h"
#elif defined CONFIG_MPU_SENSORS_MPU3050
#  include "mpu3050.h"
#else
#error Invalid or undefined CONFIG_MPU_SENSORS_MPUxxxx
#endif

#include "mlsl.h"
#include "mlinclude.h"
#include "stdint_invensense.h"

//#include "twim.h"
//#include "umplCalClient.h"

#ifdef DEBUG
#define _SerialError(...) MPL_LOGE(__VA_ARGS__)
#define _SerialDebug(...) MPL_LOGD(__VA_ARGS__)
#else
#define _SerialError(...) do {} while(0)
#define _SerialDebug(...) do {} while(0)
#endif

/**
 * @brief stub on at32 platform. The Atmel ASF twim library,
 *        which provides twim_read and twim_write,
 *        should be configured before this is called.
 *
 * @param port  Unused, pass as NULL
 * @param sl_handle  Unused, pass as NULL
 */

inv_error_t inv_serial_open(char const * port, void **sl_handle)
{
    return INV_SUCCESS;
}

inv_error_t inv_serial_reset(void *sl_handle)
{
    return INV_SUCCESS;
}

inv_error_t inv_serial_close(void *sl_handle)
{
    return INV_SUCCESS;
}

/**
 *  @brief  used as generic serial write. Does not accept a
 *          register parameter like the rest of the MLSLSerial
 *          functions - if used to write to a register, the register
 *          address should be the first member of data
 *          This should be sent by I2C.
 *
 *  @param  sl_handle       Unused, pass as NULL
 *  @param  slaveAddr       I2C slave address of device.
 *  @param  length          Length of burst of data.
 *  @param  data            Pointer to block of data.
 *
 *  @return INV_SUCCESS if successful, a non-zero error code otherwise.
 */
inv_error_t inv_serial_write( void *sl_handle,
						  unsigned char slaveAddr,
                          unsigned short length,
                          unsigned char const *data )
{
    #if 0
    inv_error_t result;
	
    // twim_write does follow the slaveAddr / slaveRegister convention
    // so we transform the inputs to follow this convention.
	uint8_t regAddr = data[0];
	data = data + 1;
	length = length -1;
	result = twim_write(slaveAddr, regAddr, data, (size_t)length);
	
    if(result != STATUS_OK ){
		_SerialError("inv_serial_write: Error. Could not twim_write.\n");
        return INV_ERROR_SERIAL_WRITE; }

		#endif

    return INV_SUCCESS;
}



/**
 *  @brief  used to write a single byte to a single register.
 *
 *  @param  sl_handle       Unused, pass as NULL
 *  @param  slaveAddr       I2C slave address of device.
 *  @param  registerAddr    Register address to write.
 *  @param  data            Single byte of data to write.
 *
 *  @return INV_SUCCESS if successful, a non-zero error code otherwise.
 */
inv_error_t inv_serial_single_write(void *sl_handle,
                                    unsigned char slaveAddr, 
                                    unsigned char registerAddr, 
                                    unsigned char data)
{
    #if 0
    inv_error_t result;
    // Copy the argument to this call stack.
    // In the past there have been compiler issues which prohibit
    // dereferencing arguments.
	unsigned char tempData[1];
	tempData[0] = data;
	result = twim_write((uint16_t)slaveAddr, (uint8_t)registerAddr, tempData, 1);

    if(result != STATUS_OK ){
#ifdef M_HW
    /* Mantis rev A2 Silicon Errata: setting reset bit will cause i2c nack */
        if ( slaveAddr == 104 && registerAddr == 107 && (tempData[0] | 0x80 ))
            return INV_SUCCESS; /* Ignore that particular error. */
#endif
		_SerialError("inv_serial_single_write: Error. Could not twim_write.\n");
        return INV_ERROR_SERIAL_WRITE;
    }

		#endif

    return INV_SUCCESS;
}


/**
 *  @brief  used to write multiple bytes of data to DMP memory.
 *
 *  @param  sl_handle       Unused, pass as NULL
 *  @param  slaveAddr       I2C slave address of device.
 *  @param  memAddr         The location in the memory to write to. 16 bits.
 *  @param  length          Length of burst data. Must not cross 8-bit address boundary.
 *  @param  data            Pointer to block of data.
 *
 *  @return Zero if successful; an error code otherwise
 */
inv_error_t inv_serial_write_mem( void *sl_handle,
							 unsigned char slaveAddr,
                             unsigned short memAddr,
                             unsigned short length, 
                             unsigned char const *data )
{
    inv_error_t result;
	  result=0;

    #if 0

	unsigned char tmpAddr;
	unsigned char memAddress[1];
	unsigned char i2cWrite[SERIAL_MAX_TRANSFER_SIZE];
    uint_fast16_t bytesWritten = 0;
	
	 if ((memAddr & 0xFF) + length > MPU_MEM_BANK_SIZE) {
        _SerialDebug("inv_serial_write_mem: memory write length (%d B) "
                     "extends beyond its limits (%d)\n "
                     "if started at location %d\n", 
                     length, MPU_MEM_BANK_SIZE, memAddr & 0xFF);
        return INV_ERROR_INVALID_PARAMETER;
    }

	/* Write bank - first time only */
    tmpAddr = MPUREG_BANK_SEL;
    memAddress[0] = memAddr >> 8;

	result = twim_write((uint16_t)slaveAddr, (uint8_t)tmpAddr, memAddress, 1 );
	if (STATUS_OK != result){
		_SerialError("inv_serial_write_mem: "
                     "Could not initialize memory read.\n");
        return INV_ERROR_MEMORY_SET;
	}	

	while (bytesWritten<length) {
        unsigned short thisLen = min(SERIAL_MAX_TRANSFER_SIZE, length-bytesWritten);

        /* Write address */
        tmpAddr = MPUREG_MEM_START_ADDR;
        memAddress[0] = memAddr+bytesWritten; // valid because we don't allow going 
                                              // beyond the MPU_MEM_BANK_SIZE boundary
        result = twim_write((uint16_t)slaveAddr,(uint8_t)tmpAddr, memAddress, 1);
        if (STATUS_OK != result){
			_SerialError("inv_serial_write_mem: "
                         "Could not set memory location for read.\n");
			return INV_ERROR_MEMORY_SET;
		}	

        /* Write memory contents from data */
        memcpy (&i2cWrite[0], &data[bytesWritten], thisLen);
		result = twim_write(slaveAddr, MPUREG_MEM_R_W, i2cWrite, thisLen);	
        
        if (STATUS_OK != result){
            _SerialError("inv_serial_write_mem: "
                         "Could not write to memory.\n");
            return INV_ERROR_MEMORY_SET; 
        }

        bytesWritten += thisLen;
    }
    #endif
    return result;
}




/**
 *  @brief  used to write multiple bytes of data to the fifo.
 *          This should be sent by I2C.
 *
 *  @param  sl_handle       Unused, pass as NULL
 *  @param  slaveAddr       I2C slave address of device.
 *  @param  length          Length of burst of data.
 *  @param  data            Pointer to block of data.
 *
 *  @return Zero if successful; an error code otherwise
 */
inv_error_t inv_serial_write_fifo( void *sl_handle,
                              unsigned char slaveAddr,
                              unsigned short length, 
                              unsigned char const *data )
{
    #if 0
    inv_error_t result;
	if (length>FIFO_HW_SIZE) {
        _SerialDebug("inv_serial_write_fifo: "
                 "maximum fifo write length is %d\n", FIFO_HW_SIZE);
        return INV_ERROR_INVALID_PARAMETER;
    }

    result = twim_write(slaveAddr, MPUREG_FIFO_R_W, data, length);	
	if(result != STATUS_OK ) {
		_SerialError("inv_serial_write_fifo: Could not twim_write.\n");
        return INV_ERROR_SERIAL_WRITE; 
    }
    #endif
    return INV_SUCCESS;
}



/**
 *  @brief  used to read multiple bytes of data from registers.
 *          This should be sent by I2C.
 *
 *  @param  sl_handle       Unused, pass as NULL
 *  @param  slaveAddr       I2C slave address of device.
 *  @param  registerAddr    Register address to read.
 *  @param  length          Length of burst of data.
 *  @param  data            Pointer to block of data.
 *
 *  @return Zero if successful; an error code otherwise
 */
inv_error_t inv_serial_read( void *sl_handle,
						 unsigned char  slaveAddr,
                         unsigned char  registerAddr,
                         unsigned short length,
                         unsigned char  *data )
{
	  #if 0
    inv_error_t result;
    if (slaveAddr == 0x68 && 
        (registerAddr==MPUREG_FIFO_R_W ||
         registerAddr==MPUREG_MEM_R_W)) {
        _SerialDebug("inv_serial_read: Invalid register."
                     "Use proper MLSLSerial api to read FIFO and DMP memory\n");
        return INV_ERROR_INVALID_PARAMETER;
    }
	result = twim_read(slaveAddr, registerAddr, data, length);
    if(result != STATUS_OK ) {
		_SerialError("\rinv_serial_read: Error. Could not twim_read.\n");
        return INV_ERROR_SERIAL_READ;
    }
		#endif
    return INV_SUCCESS;
}

/**
 *  @brief  used to read multiple bytes of data from the memory.
 *          This should be sent by I2C.
 *
 *  @param  sl_handle       Unused, pass as NULL
 *  @param  slaveAddr       I2C slave address of device.
 *  @param  memAddr         The location in the memory to read from.
 *  @param  length          Length of burst data.
 *  @param  data            Pointer to block of data.
 *
 *  @return Zero if successful; an error code otherwise
 */
inv_error_t inv_serial_read_mem( void *sl_handle,
							unsigned char  slaveAddr, 
                            unsigned short memAddr, 
                            unsigned short length, 
                            unsigned char *data )
{
	  inv_error_t result;
	  result=0;

	  #if 0
	unsigned char tmpAddr;
	unsigned char memAddress[1];
    uint_fast16_t bytesRead = 0;
	
    if (memAddr & 0xFF + length > MPU_MEM_BANK_SIZE) {
        _SerialDebug("inv_serial_read_mem: memory read length (%d B) "
                     "extends beyond its limits (%d) "
                     "if started at location %d\n", 
                     length, MPU_MEM_BANK_SIZE, memAddr & 0xFF);
        return INV_ERROR_INVALID_PARAMETER;
    }
	/* Write bank - first time only */
    tmpAddr = MPUREG_BANK_SEL;
    memAddress[0] = memAddr >> 8;

	result = twim_write((uint16_t)slaveAddr, (uint8_t)tmpAddr, memAddress, 1 );
	if (STATUS_OK != result){
		_SerialError("inv_serial_read_mem: "
                     "Could not initialize memory read.\n");
        return INV_ERROR_SERIAL_WRITE;
	}	

	while (bytesRead < length) {
        unsigned short thisLen = min(SERIAL_MAX_TRANSFER_SIZE, length-bytesRead);

        /* Write address */
        tmpAddr = MPUREG_MEM_START_ADDR;
        memAddress[0] = memAddr+bytesRead; // valid because we don't allow going 
                                            // beyond the MPU_MEM_BANK_SIZE boundary
        result = twim_write((uint16_t)slaveAddr,(uint8_t)tmpAddr, memAddress, 1);
        if (STATUS_OK != result){
			_SerialError("inv_serial_read_mem: "
                         "Could not set memory location for read.\n");
			return INV_ERROR_SERIAL_WRITE;
		}	

        /* Read actual data */
		result = twim_read(slaveAddr, MPUREG_MEM_R_W, &data[bytesRead], thisLen);	
        
        if (STATUS_OK != result){
            _SerialError("inv_serial_read_mem: "
                         "Could not read DMP memory.\n");
            return INV_ERROR_SERIAL_READ;
        }

        bytesRead += thisLen;
    }
	#endif
	return result;
}


/**
 *  @brief  used to read multiple bytes of data from the fifo.
 *          This should be sent by I2C.
 *
 *  @param  sl_handle       Unused, pass as NULL
 *  @param  slaveAddr       I2C slave address of device.
 *  @param  length          Number of bytes to read from fifo.
 *  @param  data            Pointer to write fifo data.
 *
 *  @return Zero if successful; an error code otherwise
 */
inv_error_t inv_serial_read_fifo( void *sl_handle,
							 unsigned char  slaveAddr, 
                             unsigned short length, 
                             unsigned char *data )
{
	   #if 0
    uint16_t bytesRead = 0;
    inv_error_t result;
	if (length>FIFO_HW_SIZE) {
        _SerialDebug("inv_serial_read_fifo: "
                     "maximum fifo read length is %d\n", FIFO_HW_SIZE);
        return INV_ERROR_INVALID_PARAMETER;
    }

    while (bytesRead < length) {
        unsigned short thisLen = min(SERIAL_MAX_TRANSFER_SIZE, length-bytesRead);
        
        result = twim_read(slaveAddr, MPUREG_FIFO_R_W, data, thisLen);	
        if(result != STATUS_OK ){
            _SerialError("inv_serial_read_fifo: Could not twim_read.\n");
            return INV_ERROR_SERIAL_READ;
        }
        
        bytesRead += thisLen;
    }
    #endif
    return INV_SUCCESS;
}

/**
 *  @brief  used to get the calibration data.
 *          It is called by the MPL to get the calibration data used by the 
 *          motion library.
 *          This data would typically be saved in non-volatile memory.
 *
 *  @param  cfg     Pointer to the calibration data.
 *  @param  len     Length of the calibration data.
 *
 *  @return INV_SUCCESS if successful, a non-zero error code otherwise.
 */
inv_error_t inv_serial_read_cal( unsigned char *cal, unsigned int  len )
{
    /* UMPL does not implement the inv_serial_read_cal function.*/
    return INV_ERROR_FEATURE_NOT_IMPLEMENTED;
}

/**
 *  @brief  Get the calibration length.
 *  @param  len 
 *              lenght to be returned
 *  @return INV_SUCCESS if successful, a non-zero error code otherwise.
 */
inv_error_t inv_serial_get_cal_length(unsigned int *len)
{
    /* UMPL does not implement the inv_serial_get_cal_length function.*/
    return INV_ERROR_FEATURE_NOT_IMPLEMENTED;
}

/**
 *  @brief  used to save the calibration data.
 *          It is called by the MPL to save the calibration data used by the 
 *          motion library.
 *          This data would typically be saved in non-volatile memory.
 *
 *  @param cfg  Pointer to the calibration data.
 *  @param len  Length of the calibration data.
 *
 *  @return INV_SUCCESS if successful, a non-zero error code otherwise.
 */
inv_error_t inv_serial_write_cal( unsigned char *cal, unsigned int len )
{
    /* UMPL does not implement the inv_serial_write_cal function.*/
    return INV_ERROR_FEATURE_NOT_IMPLEMENTED;
}

