#ifndef __I2C_H
#define __I2C_H

#include "stm32wlxx_hal.h"
#include "stdbool.h"



typedef enum
{
	HTS221_OK,
	HTS221_ERROR
} HTS221_StatusTypeDef;


HTS221_StatusTypeDef I2C_IO_Read(uint8_t* pBuffer, uint8_t DeviceAddr, uint8_t RegisterAddr,uint16_t NumByteToRead);
HTS221_StatusTypeDef I2C_IO_Write(uint8_t* pBuffer, uint8_t DeviceAddr, uint8_t RegisterAddr,uint16_t NumByteToWrite);


#endif //__I2C_H
