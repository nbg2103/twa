/*
 * tiva_thermometer.h
 *
 *  Created on: 20 Apr 2022
 *      Author: rafeeq
 */

#ifndef TIVA_THERMOMETER_H_
#define TIVA_THERMOMETER_H_


#include <stdint.h>
#include <stdio.h>
#include "cmsis_os.h"


extern int32_t mlx90632_i2c_read(int16_t register_address, uint16_t *value);
extern int32_t mlx90632_i2c_write(int16_t register_address, uint16_t value);
extern void usleep(int min_range, int max_range);
extern void msleep(int msecs);


#define MLX90632_I2C_ADDRESS	0x3A << 1





/**
  * @brief Thermometer Types Definition
**/
typedef struct
{
 unsigned    	Initialised : 1;
 unsigned    	Calibrated : 1;
 unsigned    	Reserved : 6;
 double     	LiquidTemp; //changed from int32_t
 double    	CoilTemp;//changed from int32_t
 double     	HeadTemp;//changed from int32_t

 void        	*GateKeeper;     //RTOS gatekeeper
} Thermometer_Data_t;


/******************************** Temp Sensor Type  **************************/
/** @defgroup TEMP_SensTypeDef Temperature Sensor structure
  *
  */
typedef struct
{
  int8_t (*Init)(void);
  int8_t (*Measure)(void);

}Thermometer_TypeDef;

/* TRIAC data structure */
extern Thermometer_Data_t tivaThermometer;

/* TRIAC driver structure */
extern Thermometer_TypeDef Thermometer_Driver;


int8_t Thermometer_IO_Read(uint16_t RegAddr, uint16_t *pData);
int8_t Thermometer_IO_Write(uint16_t RegAddr, uint16_t Data);
void Thermometer_IO_mSDelay(uint32_t mSec);
void Thermometer_IO_uSDelay(int min_range, int max_range);


#endif /* TIVA_THERMOMETER_H_ */
