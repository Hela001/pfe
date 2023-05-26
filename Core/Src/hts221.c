/**
 ******************************************************************************
 * @file    hts221.c
 * @author  MEMS Application Team
 * @version V1.2.0
 * @date    11-February-2015
 * @brief   This file provides a set of functions needed to manage the hts221.
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; COPYRIGHT(c) 2015 STMicroelectronics</center></h2>
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *   1. Redistributions of source code must retain the above copyright notice,
 *      this list of conditions and the following disclaimer.
 *   2. Redistributions in binary form must reproduce the above copyright notice,
 *      this list of conditions and the following disclaimer in the documentation
 *      and/or other materials provided with the distribution.
 *   3. Neither the name of STMicroelectronics nor the names of its contributors
 *      may be used to endorse or promote products derived from this software
 *      without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ******************************************************************************

 */
/* Includes ------------------------------------------------------------------*/
#include "hts221.h"
//#include "delay.h"
#include <math.h>
#include "stm32_hal_legacy.h"
#include <stdint.h>
#include "driver_i2c.h"


/** @addtogroup BSP
 * @{
 */

/** @addtogroup Components
 * @{
 */

/** @addtogroup HTS221
 * @{
 */


/** @defgroup HTS221_Private_Variables HTS221_Private_Variables
 * @{
 */

//MEMS_DrvTypeDef Hts221Drv =
//{
//  HTS221_Init,
//  HTS221_Power_OFF,
//  HTS221_ReadID,
//  HTS221_RebootCmd,
//  0,
//  0,
//  0,
//  0,
//  0,
//  HTS221_GetHumidity,
//  HTS221_GetTemperature,
//  NULL
//};
/* ------------------------------------------------------- */
/* Here you should declare the variable that implements    */
/* the internal struct of extended features of HTS221.     */
/* Then you must update the NULL pointer in the variable   */
/* of the extended features below.                         */
/* See the example of LSM6DS3 in lsm6ds3.c                 */
/* ------------------------------------------------------- */

//MEMS_DrvExtTypeDef Hts221Drv_ext =
//{
//  MEMS_HTS221_COMPONENT, /* unique ID for HTS221 in the humidity and temperature driver class */
//  NULL /* pointer to internal struct of extended features of HTS221 */
//};


/* Temperature in degree for calibration  */
float T0_degC, T1_degC;

/* Output temperature value for calibration */
int16_t T0_out, T1_out;


/* Humidity for calibration  */
float H0_rh, H1_rh;

/* Output Humidity value for calibration */
int16_t H0_T0_out, H1_T0_out;

/**
 * @}
 */

//static MEMS_StatusTypeDef HTS221_Power_On(void);
static HTS221_StatusTypeDef HTS221_Calibration(void);
	
extern HTS221_StatusTypeDef HTS221_GetHT(float* pTemp, float* pHumidity);
/**
 * @brief  HTS221 Calibration procedure
 * @retval MEMS_OK in case of success, an error code otherwise
 */
 
 
/** @defgroup HTS221_Private_Functions HTS221_Private_Functions
 * @{
 */

/**
 * @brief  HTS221 Calibration procedure
 * @retval MEMS_OK in case of success, an error code otherwise
 */
 
 //HTS221_StatusTypeDef stFn_HTS221_Calibration(void)
static HTS221_StatusTypeDef HTS221_Calibration(void)
{
  /* Temperature Calibration */
  /* Temperature in degree for calibration ( "/8" to obtain float) */
  uint16_t T0_degC_x8_L, T0_degC_x8_H, T1_degC_x8_L, T1_degC_x8_H;
  uint8_t H0_rh_x2, H1_rh_x2;
  uint8_t tempReg[2] = {0, 0};
  
  if(I2C_IO_Read(tempReg, HTS221_ADDRESS, HTS221_T0_degC_X8_ADDR, 1) != HTS221_OK)
  {
    return HTS221_ERROR;
  }
  
  T0_degC_x8_L = (uint16_t)tempReg[0];
  
  if(I2C_IO_Read(tempReg, HTS221_ADDRESS, HTS221_T1_T0_MSB_X8_ADDR, 1) != HTS221_OK)
  {
    return HTS221_ERROR;
  }
  
  T0_degC_x8_H = (uint16_t) (tempReg[0] & 0x03);
  T0_degC = ((float)((T0_degC_x8_H << 8) | (T0_degC_x8_L))) / 8;
  
  if(I2C_IO_Read(tempReg, HTS221_ADDRESS, HTS221_T1_degC_X8_ADDR, 1) != HTS221_OK)
  {
    return HTS221_ERROR;
  }
  
  T1_degC_x8_L = (uint16_t)tempReg[0];
  
  if(I2C_IO_Read(tempReg, HTS221_ADDRESS, HTS221_T1_T0_MSB_X8_ADDR, 1) != HTS221_OK)
  {
    return HTS221_ERROR;
  }
  
  T1_degC_x8_H = (uint16_t) (tempReg[0] & 0x0C);
  T1_degC_x8_H = T1_degC_x8_H >> 2;
  T1_degC = ((float)((T1_degC_x8_H << 8) | (T1_degC_x8_L))) / 8;
  
  if(I2C_IO_Read(tempReg, HTS221_ADDRESS, (HTS221_T0_OUT_L_ADDR | HTS221_I2C_MULTIPLEBYTE_CMD), 2) != HTS221_OK)
  {
    return HTS221_ERROR;
  }
  
  T0_out = ((((int16_t)tempReg[1]) << 8) + (int16_t)tempReg[0]);
  
  if(I2C_IO_Read(tempReg, HTS221_ADDRESS, (HTS221_T1_OUT_L_ADDR | HTS221_I2C_MULTIPLEBYTE_CMD), 2) != HTS221_OK)
  {
    return HTS221_ERROR;
  }
  
  T1_out = ((((int16_t)tempReg[1]) << 8) + (int16_t)tempReg[0]);
  
  /* Humidity Calibration */
  /* Humidity in degree for calibration ( "/2" to obtain float) */
  
  if(I2C_IO_Read(&H0_rh_x2, HTS221_ADDRESS, HTS221_H0_RH_X2_ADDR, 1) != HTS221_OK)
  {
    return HTS221_ERROR;
  }
  
  if(I2C_IO_Read(&H1_rh_x2, HTS221_ADDRESS, HTS221_H1_RH_X2_ADDR, 1) != HTS221_OK)
  {
    return HTS221_ERROR;
  }
  
  if(I2C_IO_Read(&tempReg[0], HTS221_ADDRESS, (HTS221_H0_T0_OUT_L_ADDR | HTS221_I2C_MULTIPLEBYTE_CMD),
                    2) != HTS221_OK)
  {
    return HTS221_ERROR;
  }
  
  H0_T0_out = ((((int16_t)tempReg[1]) << 8) + (int16_t)tempReg[0]);
  
  if(I2C_IO_Read(&tempReg[0], HTS221_ADDRESS, (HTS221_H1_T0_OUT_L_ADDR  | HTS221_I2C_MULTIPLEBYTE_CMD),
                    2) != HTS221_OK)
  {
    return HTS221_ERROR;
  }
  
  H1_T0_out = ((((int16_t)tempReg[1]) << 8) + (int16_t)tempReg[0]);
  
  H0_rh = ((float)H0_rh_x2) / 2;
  H1_rh = ((float)H1_rh_x2) / 2;
  
  return HTS221_OK;
}

//void vFn_HTS221_IO_ITConfig(void)
void vHTS221_IO_ITConfig(void)
{
	 GPIO_InitTypeDef  gpioinitstruct = {0};
  
  /* Enable the GPIO_B Clock */
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /* Configure the GPIO pin */  
  gpioinitstruct.Mode   = GPIO_MODE_INPUT;
  gpioinitstruct.Pull   = GPIO_PULLUP;
///  gpioinitstruct.Speed  = GPIO_SPEED_HIGH;
  
  gpioinitstruct.Pin    = 	 GPIO_PIN_13;
  HAL_GPIO_Init(GPIOB, &gpioinitstruct);
}

/**
 * @brief  Set HTS221 Initialization
 * @param  HTS221_Init the configuration setting for the HTS221
 * @retval MEMS_OK in case of success, an error code otherwise
 */
 //HTS221_StatusTypeDef stFn_HTS221_Init(void)
HTS221_StatusTypeDef HTS221_Init(void)
{
  uint8_t tmp = 0x00;
  
  /* Configure the low level interface ---------------------------------------*/
  
  if(HTS221_Power_On() != HTS221_OK)
  {
    return HTS221_ERROR;
  }
  
  if(HTS221_Calibration() != HTS221_OK)
  {
    return HTS221_ERROR;
  }
  
  if(I2C_IO_Read(&tmp, HTS221_ADDRESS, HTS221_CTRL_REG1_ADDR, 1) != HTS221_OK)
  {
    return HTS221_ERROR;
  }
  
  /* Output Data Rate selection */
  tmp &= ~(HTS221_ODR_MASK | HTS221_BDU_MASK);
  tmp |= HTS221_ODR_ONE_SHOT | HTS221_BDU_NOT_UNTIL_READING;
  
  if(I2C_IO_Write(&tmp, HTS221_ADDRESS, HTS221_CTRL_REG1_ADDR, 1) != HTS221_OK)
  {
    return HTS221_ERROR;
  }
  
  
  
  if(I2C_IO_Read(&tmp, HTS221_ADDRESS, HTS221_CTRL_REG3_ADDR, 1) != HTS221_OK)
  {
	  return HTS221_ERROR;
  }
  
  tmp |= 0xC3;
  
  if(I2C_IO_Write(&tmp, HTS221_ADDRESS, HTS221_CTRL_REG3_ADDR, 1) != HTS221_OK)
  {
	  return HTS221_ERROR;
  }
  
  /* hbah */
  tmp = 0x12;
  if(I2C_IO_Write(&tmp, HTS221_ADDRESS, HTS221_RES_CONF_ADDR, 1) != HTS221_OK)
  {
	  return HTS221_ERROR;
  }
  
  
  //vHTS221_IO_ITConfig();
  return HTS221_OK;
}

/**
 * @brief  Read ID address of HTS221
 * @param  ht_id the pointer where the ID of the device is stored
 * @retval MEMS_OK in case of success, an error code otherwise
 */

 //HTS221_StatusTypeDef stFn_HTS221_ReadID(uint8_t *ht_id)
HTS221_StatusTypeDef HTS221_ReadID(uint8_t *ht_id)
{
  if(!ht_id)
  {
    return HTS221_ERROR;
  }
  
  return I2C_IO_Read(ht_id, HTS221_ADDRESS, HTS221_WHO_AM_I_ADDR, 1);
}

/**
 * @brief  Reboot memory content of HTS221
 * @retval MEMS_OK in case of success, an error code otherwise
 */

//HTS221_StatusTypeDef stFn_HTS221_RebootCmd(void)
HTS221_StatusTypeDef HTS221_RebootCmd(void)
{
  uint8_t tmpreg;
  
  /* Read CTRL_REG2 register */
  if(I2C_IO_Read(&tmpreg, HTS221_ADDRESS, HTS221_CTRL_REG2_ADDR, 1) != HTS221_OK)
  {
    return HTS221_ERROR;
  }
  
  /* Enable or Disable the reboot memory */
  tmpreg |= HTS221_BOOT_REBOOTMEMORY;
  
  /* Write value to MEMS CTRL_REG2 regsister */
  if(I2C_IO_Write(&tmpreg, HTS221_ADDRESS, HTS221_CTRL_REG2_ADDR, 1) != HTS221_OK)
  {
    return HTS221_ERROR;
  }
  
  return HTS221_OK;
}


/**
 * @brief  Read HTS221 output register, and calculate the humidity
 * @param  pfData the pointer to data output
 * @retval MEMS_OK in case of success, an error code otherwise
 */

//HTS221_StatusTypeDef stFn_HTS221_GetHumidity(float* pfData)
HTS221_StatusTypeDef HTS221_GetHumidity(float* pfData)
{
	
  int16_t H_T_out, humidity_t;
  uint8_t tempReg[2] = {0, 0};
  uint8_t tmp = 0x00;
  float H_rh;
  
	/* Default humidity value */
	*pfData = 8191;
	
  if(I2C_IO_Read(&tmp, HTS221_ADDRESS, HTS221_CTRL_REG1_ADDR, 1) != HTS221_OK)
  {
    return HTS221_ERROR;
  }
  
  /* Output Data Rate selection */
  tmp &= (HTS221_ODR_MASK);
  
  if(tmp == 0x00)
  {
    if(I2C_IO_Read(&tmp, HTS221_ADDRESS, HTS221_CTRL_REG2_ADDR, 1) != HTS221_OK)
    {
      return HTS221_ERROR;
    }
    
    /* Serial Interface Mode selection */
    tmp &= ~(HTS221_ONE_SHOT_MASK);
    tmp |= HTS221_ONE_SHOT_START;
    
    if(I2C_IO_Write(&tmp, HTS221_ADDRESS, HTS221_CTRL_REG2_ADDR, 1) != HTS221_OK)
    {
      return HTS221_ERROR;
    }
    
    do
    {
      if(I2C_IO_Read(&tmp, HTS221_ADDRESS, HTS221_STATUS_REG_ADDR, 1) != HTS221_OK)
      {
        return HTS221_ERROR;
      }
    }
    while(!(tmp & 0x03));
		if(I2C_IO_Read(&tempReg[0], HTS221_ADDRESS, (HTS221_HUMIDITY_OUT_L_ADDR | HTS221_I2C_MULTIPLEBYTE_CMD),2) != HTS221_OK)
		{
			return HTS221_ERROR;
		}
  
		H_T_out = ((((int16_t)tempReg[1]) << 8) + (int16_t)tempReg[0]);
  
		H_rh = ( float )(((( H_T_out - H0_T0_out ) * ( H1_rh - H0_rh )) / ( H1_T0_out - H0_T0_out )) + H0_rh );
  
		// Truncate to specific number of decimal digits
		humidity_t = (uint16_t)(H_rh * pow(10, HUM_DECIMAL_DIGITS));
		*pfData = ((float)humidity_t) / pow(10, HUM_DECIMAL_DIGITS);
  
		// Prevent data going below 0% and above 100% due to linear interpolation
		if ( *pfData <   0.0f ) *pfData =   0.0f;
		if ( *pfData > 100.0f ) *pfData = 100.0f;
	}
  
  return HTS221_OK;
}

/**
 * @brief  Read HTS221 output register, and calculate the temperature
 * @param  pfData the pointer to data output
 * @retval MEMS_OK in case of success, an error code otherwise
 */

//HTS221_StatusTypeDef stFn_HTS221_GetTemperature(float* pfData)
HTS221_StatusTypeDef HTS221_GetTemperature(float* pfData)
{
  int16_t T_out, temperature_t;
  uint8_t tempReg[2] = {0, 0};
  uint8_t tmp = 0x00;
  float T_degC;
	float pont;
  
	/* Default temperature value */
	*pfData = 8191;
	
  if(I2C_IO_Read(&tmp, HTS221_ADDRESS, HTS221_CTRL_REG1_ADDR, 1) != HTS221_OK)
  {
    return HTS221_ERROR;
  }
  
  /* Output Data Rate selection */
  tmp &= (HTS221_ODR_MASK);
  
  if(tmp == 0x00)
  {
    if(I2C_IO_Read(&tmp, HTS221_ADDRESS, HTS221_CTRL_REG2_ADDR, 1) != HTS221_OK)
    {
      return HTS221_ERROR;
    }
    
    /* Serial Interface Mode selection */
    tmp &= ~(HTS221_ONE_SHOT_MASK);
    tmp |= HTS221_ONE_SHOT_START;
    
    if(I2C_IO_Write(&tmp, HTS221_ADDRESS, HTS221_CTRL_REG2_ADDR, 1) != HTS221_OK)
    {
      return HTS221_ERROR;
    }
    
    do
    {
      if(I2C_IO_Read(&tmp, HTS221_ADDRESS, HTS221_STATUS_REG_ADDR, 1) != HTS221_OK)
      {
        return HTS221_ERROR;
      }
    }
    while(!(tmp & 0x03));
		uint8_t expression =(HTS221_TEMP_OUT_L_ADDR | HTS221_I2C_MULTIPLEBYTE_CMD);
				if(I2C_IO_Read(&tempReg[0], HTS221_ADDRESS, (HTS221_TEMP_OUT_L_ADDR | HTS221_I2C_MULTIPLEBYTE_CMD),2) != HTS221_OK)
				{
							return HTS221_ERROR;
				}

				T_out = ((((int16_t)tempReg[1]) << 8) + (int16_t)tempReg[0]);
  
				T_degC = ((float)(T_out - T0_out)) / (float)(T1_out - T0_out) * (float)(T1_degC - T0_degC) + T0_degC;
				
				/* 2 em méthode de calcul ******/
				//pont =(float)(T1_degC - T0_degC) /(float)(T1_out - T0_out);
				//T_degC =  T0_degC + pont * ((float)(T_out - T0_out));
		   
				temperature_t = (int16_t)(T_degC * pow(10, TEMP_DECIMAL_DIGITS));
  
				*pfData = ((float)temperature_t) / pow(10, TEMP_DECIMAL_DIGITS);
  }
  
  return HTS221_OK;
}

/*------------------------------------------------------------------------*/

////HTS221_StatusTypeDef stFn_HTS221_GetHT( float* pTemp,float* pHumidity)
HTS221_StatusTypeDef HTS221_GetHT( float* pTemp,float* pHumidity)
{

  int16_t T_out, temperature_t;
  uint8_t tempReg[2] = {0, 0};
  uint8_t tmp = 0x00;
  float T_degC,H_rh;
  int16_t H_T_out, humidity_t;
	uint8_t timeout=150;
	
	/* Default temperature value & humidity*/
	*pTemp = 8191;
	*pHumidity = 8191;
	
	if(I2C_IO_Read(&tmp, HTS221_ADDRESS, HTS221_CTRL_REG2_ADDR, 1) != HTS221_OK)
	{
		return HTS221_ERROR;
	}
	
	/* Serial Interface Mode selection */
	tmp &= ~(HTS221_ONE_SHOT_MASK);
	tmp |= HTS221_ONE_SHOT_START;
	
	if(I2C_IO_Write(&tmp, HTS221_ADDRESS, HTS221_CTRL_REG2_ADDR, 1) != HTS221_OK)
	{
		return HTS221_ERROR;
	}
	
	do
	{
		if(0==timeout){
			return HTS221_ERROR;
		}
		timeout--;
	
		if(I2C_IO_Read(&tmp, HTS221_ADDRESS, HTS221_STATUS_REG_ADDR, 1) != HTS221_OK)
		{
			return HTS221_ERROR;
		}
		
	}
	while(!(tmp & 0x03));
	

	if(I2C_IO_Read(&tempReg[0], HTS221_ADDRESS, (HTS221_HUMIDITY_OUT_L_ADDR | HTS221_I2C_MULTIPLEBYTE_CMD),2) != HTS221_OK)
	{
		return HTS221_ERROR;
	}
	H_T_out = ((((int16_t)tempReg[1]) << 8) + (int16_t)tempReg[0]);

	H_rh = ( float )(((( H_T_out - H0_T0_out ) * ( H1_rh - H0_rh )) / ( H1_T0_out - H0_T0_out )) + H0_rh );

	// Truncate to specific number of decimal digits
	humidity_t = (uint16_t)(H_rh * pow(10, HUM_DECIMAL_DIGITS));
	*pHumidity = ((float)humidity_t) / pow(10, HUM_DECIMAL_DIGITS);

	// Prevent data going below 0% and above 100% due to linear interpolation
	if ( *pHumidity <   0.0f ) *pHumidity =   0.0f;
	if ( *pHumidity > 100.0f ) *pHumidity = 100.0f;
	
	
	if(I2C_IO_Read(&tempReg[0], HTS221_ADDRESS, (HTS221_TEMP_OUT_L_ADDR | HTS221_I2C_MULTIPLEBYTE_CMD),2) != HTS221_OK)
	{
		return HTS221_ERROR;
	}

	T_out = ((((int16_t)tempReg[1]) << 8) + (int16_t)tempReg[0]);

	T_degC = ((float)(T_out - T0_out)) / (float)(T1_out - T0_out) * (float)(T1_degC - T0_degC) + T0_degC;

	temperature_t = (int16_t)(T_degC * pow(10, TEMP_DECIMAL_DIGITS));

	*pTemp = ((float)temperature_t) / pow(10, TEMP_DECIMAL_DIGITS);
	
  return HTS221_OK;
}

/* deactivate Data Ready*/
//void vFn_DeactivateDataReadyOutput(void)
void vDeactivateDataReadyOutput(void)
{
	uint8_t tmp = 0x00;
	/*
	The DRDY signal returns inactive after both HUMIDITY_OUT_H and TEMP_OUT_H 
	registers are read.
	*/
	
	I2C_IO_Read(&tmp, HTS221_ADDRESS, HTS221_HUMIDITY_OUT_H_ADDR, 1);
	I2C_IO_Read(&tmp, HTS221_ADDRESS, HTS221_TEMP_OUT_H_ADDR, 1);
}

/**
 * @brief  Exit the shutdown mode for HTS221
 * @retval MEMS_OK in case of success, an error code otherwise
 */
//HTS221_StatusTypeDef st_HTS221_Power_On(void)
HTS221_StatusTypeDef HTS221_Power_On(void)
{
  uint8_t tmpReg;
  
  /* Read the register content */
  if(I2C_IO_Read(&tmpReg, HTS221_ADDRESS, HTS221_CTRL_REG1_ADDR, 1) != HTS221_OK)
  {
    return HTS221_ERROR;
  }
  
  /* Set the power down bit */
  tmpReg |= HTS221_MODE_ACTIVE;
  
  /* Write register */
  if(I2C_IO_Write(&tmpReg, HTS221_ADDRESS, HTS221_CTRL_REG1_ADDR, 1) != HTS221_OK)
  {
    return HTS221_ERROR;
  }
  
  return HTS221_OK;
}

/**
 * @brief  Enter the shutdown mode for HTS221
 * @retval MEMS_OK in case of success, an error code otherwise
 */

//HTS221_StatusTypeDef st_HHTS221_Power_OFF(void)
HTS221_StatusTypeDef HTS221_Power_OFF(void)
{
  uint8_t tmpReg;
	
  /* Read the register content */
  if(I2C_IO_Read(&tmpReg, HTS221_ADDRESS, HTS221_CTRL_REG1_ADDR, 1) != HTS221_OK)
  {
    return HTS221_ERROR;
  }
  
  /* Reset the power down bit */
  tmpReg &= ~(HTS221_MODE_ACTIVE);
  
  /* Write register */
  if(I2C_IO_Write(&tmpReg, HTS221_ADDRESS, HTS221_CTRL_REG1_ADDR, 1) != HTS221_OK)
  {
    return HTS221_ERROR;
  }
  
  return HTS221_OK;
}

//Start Maher Merzougui code
//HTS221_StatusTypeDef st_HTS221_StartHeatingCycle(void)
HTS221_StatusTypeDef HTS221_StartHeatingCycle(void)
{
  uint8_t tmpreg;
  
  /* Read CTRL_REG2 register */
  if(I2C_IO_Read(&tmpreg, HTS221_ADDRESS, HTS221_CTRL_REG2_ADDR, 1) != HTS221_OK)
  {
    return HTS221_ERROR;
  }
  
  /* Enable or Disable the reboot memory */
  tmpreg |= HTS221_Heater_START;
  
  /* Write value to MEMS CTRL_REG2 regsister */
  if(I2C_IO_Write(&tmpreg, HTS221_ADDRESS, HTS221_CTRL_REG2_ADDR, 1) != HTS221_OK)
  {
    return HTS221_ERROR;
  }
  
  return HTS221_OK;
}

//HTS221_StatusTypeDef st_HTS221_EndHeatingCycle(void)
HTS221_StatusTypeDef HTS221_EndHeatingCycle(void)
{
  uint8_t tmpreg;
  
  /* Read CTRL_REG2 register */
  if(I2C_IO_Read(&tmpreg, HTS221_ADDRESS, HTS221_CTRL_REG2_ADDR, 1) != HTS221_OK)
  {
    return HTS221_ERROR;
  }
  
  /* Enable or Disable the reboot memory */
  tmpreg &= HTS221_Heater_END;
  
  /* Write value to MEMS CTRL_REG2 regsister */
  if(I2C_IO_Write(&tmpreg, HTS221_ADDRESS, HTS221_CTRL_REG2_ADDR, 1) != HTS221_OK)
  {
    return HTS221_ERROR;
  }
  
  return HTS221_OK;
}

// void vFn_HTS221_Heating_Process(int Heating_Time_ms){
void HTS221_Heating_Process(int Heating_Time_ms){
	if(Heating_Time_ms>0){
		HTS221_StartHeatingCycle();
		HAL_Delay(Heating_Time_ms);
		HTS221_EndHeatingCycle();
	}
}

/***********/
//End Maher Merzougui code
/**
 * @}
 */

/**
 * @}
 */

/**
 * @}
 */

/**
 * @}
 */


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
