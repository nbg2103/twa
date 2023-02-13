/*
 * tiva_tof.h
 *
 *  Created on: Apr 17, 2022
 *      Author: rafeeq
 */

#ifndef TOF_TIVA_TOF_H_
#define TOF_TIVA_TOF_H_




#include <stdint.h>
#include <stdbool.h>


/**
  * @brief ToF controller Types Definition
**/
typedef struct
{
 unsigned    	Initialised : 1;
 unsigned    	Calibrated : 1;
 unsigned    	Reserved : 6;
 uint8_t	Level;

 void        	*GateKeeper;     //RTOS gatekeeper
} ToF_Data_t;


extern ToF_Data_t tivaToF;

uint8_t ToF_IO_Init(void);
uint8_t ToF_IO_Write(uint16_t Reg, uint8_t Value);
uint8_t ToF_IO_WriteHWord(uint16_t Reg, uint16_t Value);
uint8_t ToF_IO_Read(uint16_t Reg, uint8_t *Value);
uint8_t ToF_IO_Read(uint16_t Reg, uint8_t *Value);
uint8_t ToF_IO_ReadHWord(uint16_t Reg, uint16_t *Value);
void ToF_IO_Delay(uint32_t Delay);


/******************************** ToF Driver Type  **************************/
/** @defgroup ToF_DrvTypeDef  CTP Driver structure
  *
  */
typedef struct
{
  uint8_t       (*Init)(void);
  uint16_t   	(*ReadID)(void);
  void       	(*Reset)(void);
  uint8_t	(*GetDistance)(void);
  float	     	(*GetAmbientLight)(uint8_t);

}ToF_DrvTypeDef;

extern ToF_DrvTypeDef ToF_Driver;


#endif /* TOF_TIVA_TOF_H_ */
