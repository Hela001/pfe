/* Includes ------------------------------------------------------------------*/
#include "driver_i2c.h"
#include "hts221.h"


/**************************** define ***************************/

extern I2C_HandleTypeDef	hi2c3;

uint8_t ret=0;

/************************************ HTS221_Memory_Read **********************************/
//HTS221_StatusTypeDef st_I2C_IO_Read(uint8_t* pBuffer, uint8_t DeviceAddr, uint8_t RegisterAddr,uint16_t NumByteToRead)

HTS221_StatusTypeDef I2C_IO_Read(uint8_t* pBuffer, uint8_t DeviceAddr, uint8_t RegisterAddr,uint16_t NumByteToRead)
{
	HTS221_StatusTypeDef ret_val = HTS221_OK;
	
	HAL_ResumeTick();   // added by Maher
	
	/* call I2C_EXPBD Read data bus function */
	if(HAL_I2C_Mem_Read(&hi2c3, DeviceAddr, (uint16_t)RegisterAddr, I2C_MEMADD_SIZE_8BIT, pBuffer, NumByteToRead,/* 0x1000 */ 1000) != HAL_OK)
	{
		ret_val = HTS221_ERROR;
	}
	
	HAL_SuspendTick();   // added by Maher
	
	return ret_val;
}

/*********************************** HTS221_Memory_Write **********************************/
//HTS221_StatusTypeDef st_I2C_IO_Write(uint8_t* pBuffer, uint8_t DeviceAddr, uint8_t RegisterAddr,uint16_t NumByteToWrite)

HTS221_StatusTypeDef I2C_IO_Write(uint8_t* pBuffer, uint8_t DeviceAddr, uint8_t RegisterAddr,uint16_t NumByteToWrite)
{
	HTS221_StatusTypeDef ret_val = HTS221_OK;
	
	/* call I2C_EXPBD Read data bus function */
	//I2C_EXPBD_WriteData(pBuffer, DeviceAddr, RegisterAddr, NumByteToWrite);
	
	HAL_ResumeTick();   // added by Maher
	
	if(HAL_I2C_Mem_Write(&hi2c3, DeviceAddr, (uint16_t)RegisterAddr, I2C_MEMADD_SIZE_8BIT, pBuffer, NumByteToWrite,/* 0x1000 */ 1000)!= HAL_OK)
	{
		ret_val = HTS221_ERROR;
	}
	
	HAL_SuspendTick();   // added by Maher
	
	return ret_val;
}

