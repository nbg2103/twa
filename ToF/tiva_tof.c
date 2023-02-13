/*
 * tiva_tof.c
 *
 *  Created on: Apr 17, 2022
 *      Author: rafeeq
 */




#include "tiva_tof.h"
#include "vl6180.h"
#include "i2c.h"

#include "FreeRTOS.h"


ToF_Data_t tivaToF = {0};


/* ToF driver structure initialization */
ToF_DrvTypeDef ToF_Driver =
{
    VL6180x_Init,
    VL6180x_ReadId,
    NULL,
    VL6180x_GetDistance,
    VL6180x_GetAmbientLight,
};

/**
  * @brief  Initialize the IO for ToF.
  * @param  None
  * @retval None
  */
uint8_t ToF_IO_Init(void)
{
  tivaToF.Initialised = true;

  return tivaToF.Initialised;
}



/**
  * @brief  Write IO for ToF.
  * @param  Reg: ToF register address
  * @retval I2C status
  */
uint8_t ToF_IO_Write(uint16_t Reg, uint8_t Value)
{
  if(tivaToF.Initialised != true)
    return false;

  if(HAL_I2C_Mem_Write(&hi2c2, VL6180X_I2C_ADDRESS, Reg, I2C_MEMADD_SIZE_16BIT, &Value,1,100) == HAL_OK)
    return true;

  return false;
}

/**
  * @brief  Write 16bit IO for ToF.
  * @param  Reg: ToF register address
  * @retval I2C status
  */
uint8_t ToF_IO_WriteHWord(uint16_t Reg, uint16_t Value)
{
  if(tivaToF.Initialised != true)
    return false;

  if(HAL_I2C_Mem_Write(&hi2c2, VL6180X_I2C_ADDRESS, Reg, I2C_MEMADD_SIZE_16BIT, (uint8_t *)&Value,2,100) == HAL_OK)
    return true;

  return false;
}



/**
  * @brief  Read IO for ToF.
  * @param  Reg     : ToF register address
  * @param  *Value  : Pointer for data buffer
  * @retval         : I2C status
  */
uint8_t ToF_IO_Read(uint16_t Reg, uint8_t *Value)
{
  if(tivaToF.Initialised != true)
    return false;

  if(HAL_I2C_Mem_Read(&hi2c2, VL6180X_I2C_ADDRESS, Reg, I2C_MEMADD_SIZE_16BIT, Value,1,100) == HAL_OK)
    return true;

  return false;
}


/**
  * @brief  Read IO 16bit for ToF.
  * @param  Reg     : ToF register address
  * @param  *Value  : Pointer for data buffer
  * @retval         : I2C status
  */
uint8_t ToF_IO_ReadHWord(uint16_t Reg, uint16_t *Value)
{
  if(tivaToF.Initialised != true)
    return false;

  if(HAL_I2C_Mem_Read(&hi2c2, VL6180X_I2C_ADDRESS, Reg, I2C_MEMADD_SIZE_16BIT, (uint8_t *)Value,2,100) == HAL_OK)
    return true;

  return false;
}




/**
  * @brief  IO Delay for ToF.
  * @param  Delay: Delay in mS
  * @retval none
  */
void ToF_IO_Delay(uint32_t Delay)
{
  osDelay(Delay);
}


/**
  * @brief  RTOS event update from ISR.
  * @param  pEvent: pointer to xEventGroup data type
  * @retval data : I2C status
  */
void  ToF_IO_UpdateEvent(void *pEvent)
{
  if((tivaToF.Initialised))
  {
//    BaseType_t xHigherPriorityTaskWoken = pdFALSE, xResult;
//
//    /* Set bit  xEventGroup. */
//    xResult = xEventGroupSetBitsFromISR(pEvent, TIVA_CTP_ISR_EVENT, &xHigherPriorityTaskWoken);
//
//    /* Was the message posted successfully? */
//    if( xResult != pdFAIL )
//    {
//        portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
//    }
  }
}


