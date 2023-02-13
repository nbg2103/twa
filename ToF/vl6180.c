/*
 * vl6180.c
 *
 *  Created on: Apr 17, 2022
 *      Author: rafeeq
 */


#include "vl6180.h"
#include "tiva_tof.h"



static uint8_t VL6180x_GetRegister(uint16_t registerAddr, uint8_t *data);
static uint8_t VL6180x_GetRegister16bit(uint16_t registerAddr, uint16_t *data);

static uint8_t VL6180x_SetRegister(uint16_t registerAddr, uint8_t data);
static uint8_t VL6180x_SetRegister16bit(uint16_t registerAddr, uint16_t data);


//Load structure provided by the user with identification info
//Structure example:
// struct VL6180xIdentification
//  {
//   uint8_t idModel;
//   uint8_t idModelRevMajor;
//   uint8_t idModelRevMinor;
//   uint8_t idModuleRevMajor;
//   uint8_t idModuleRevMinor;
//   uint16_t idDate;
//   uint16_t idTime;
//   };
static uint8_t VL6180x_GetIdentification(struct VL6180xIdentification *temp);


uint8_t VL6180x_Init(void){
  uint8_t data = 0, ret; //for temp data storage

  ToF_IO_Init();

  VL6180x_GetRegister(VL6180X_SYSTEM_FRESH_OUT_OF_RESET, &data);

  if(data != 1) return false;

  //Required by datasheet
  //http://www.st.com/st-web-ui/static/active/en/resource/technical/document/application_note/DM00122600.pdf
  ret = VL6180x_SetRegister(0x0207, 0x01);
  ret |= VL6180x_SetRegister(0x0208, 0x01);
  ret |= VL6180x_SetRegister(0x0096, 0x00);
  ret |= VL6180x_SetRegister(0x0097, 0xfd);
  ret |= VL6180x_SetRegister(0x00e3, 0x00);
  ret |= VL6180x_SetRegister(0x00e4, 0x04);
  ret |= VL6180x_SetRegister(0x00e5, 0x02);
  ret |= VL6180x_SetRegister(0x00e6, 0x01);
  ret |= VL6180x_SetRegister(0x00e7, 0x03);
  ret |= VL6180x_SetRegister(0x00f5, 0x02);
  ret |= VL6180x_SetRegister(0x00d9, 0x05);
  ret |= VL6180x_SetRegister(0x00db, 0xce);
  ret |= VL6180x_SetRegister(0x00dc, 0x03);
  ret |= VL6180x_SetRegister(0x00dd, 0xf8);
  ret |= VL6180x_SetRegister(0x009f, 0x00);
  ret |= VL6180x_SetRegister(0x00a3, 0x3c);
  ret |= VL6180x_SetRegister(0x00b7, 0x00);
  ret |= VL6180x_SetRegister(0x00bb, 0x3c);
  ret |= VL6180x_SetRegister(0x00b2, 0x09);
  ret |= VL6180x_SetRegister(0x00ca, 0x09);
  ret |= VL6180x_SetRegister(0x0198, 0x01);
  ret |= VL6180x_SetRegister(0x01b0, 0x17);
  ret |= VL6180x_SetRegister(0x01ad, 0x00);
  ret |= VL6180x_SetRegister(0x00ff, 0x05);
  ret |= VL6180x_SetRegister(0x0100, 0x05);
  ret |= VL6180x_SetRegister(0x0199, 0x05);
  ret |= VL6180x_SetRegister(0x01a6, 0x1b);
  ret |= VL6180x_SetRegister(0x01ac, 0x3e);
  ret |= VL6180x_SetRegister(0x01a7, 0x1f);
  ret |= VL6180x_SetRegister(0x0030, 0x00);

  return ret;
}


uint8_t VL6180x_DefautSettings(void){

  uint8_t ret;

  //Recommended settings from datasheet
  //http://www.st.com/st-web-ui/static/active/en/resource/technical/document/application_note/DM00122600.pdf

  //Enable Interrupts on Conversion Complete (any source)
  ret = VL6180x_SetRegister(VL6180X_SYSTEM_INTERRUPT_CONFIG_GPIO, (4 << 3)|(4) ); // Set GPIO1 high when sample complete


  ret |= VL6180x_SetRegister(VL6180X_SYSTEM_MODE_GPIO1, 0x10); // Set GPIO1 high when sample complete
  ret |= VL6180x_SetRegister(VL6180X_READOUT_AVERAGING_SAMPLE_PERIOD, 0x30); //Set Avg sample period
  ret |= VL6180x_SetRegister(VL6180X_SYSALS_ANALOGUE_GAIN, 0x46); // Set the ALS gain
  ret |= VL6180x_SetRegister(VL6180X_SYSRANGE_VHV_REPEAT_RATE, 0xFF); // Set auto calibration period (Max = 255)/(OFF = 0)
  ret |= VL6180x_SetRegister(VL6180X_SYSALS_INTEGRATION_PERIOD, 0x63); // Set ALS integration time to 100ms
  ret |= VL6180x_SetRegister(VL6180X_SYSRANGE_VHV_RECALIBRATE, 0x01); // perform a single temperature calibration
  //Optional settings from datasheet
  //http://www.st.com/st-web-ui/static/active/en/resource/technical/document/application_note/DM00122600.pdf
  ret |= VL6180x_SetRegister(VL6180X_SYSRANGE_INTERMEASUREMENT_PERIOD, 0x09); // Set default ranging inter-measurement period to 100ms
  ret |= VL6180x_SetRegister(VL6180X_SYSALS_INTERMEASUREMENT_PERIOD, 0x0A); // Set default ALS inter-measurement period to 100ms
  ret |= VL6180x_SetRegister(VL6180X_SYSTEM_INTERRUPT_CONFIG_GPIO, 0x24); // Configures interrupt on ‘New Sample Ready threshold event’
  //Additional settings defaults from community
  ret |= VL6180x_SetRegister(VL6180X_SYSRANGE_MAX_CONVERGENCE_TIME, 0x32);
  ret |= VL6180x_SetRegister(VL6180X_SYSRANGE_RANGE_CHECK_ENABLES, 0x10 | 0x01);
  ret |= VL6180x_SetRegister16bit(VL6180X_SYSRANGE_EARLY_CONVERGENCE_ESTIMATE, 0x7B );
  ret |= VL6180x_SetRegister16bit(VL6180X_SYSALS_INTEGRATION_PERIOD, 0x64);

  ret |= VL6180x_SetRegister(VL6180X_READOUT_AVERAGING_SAMPLE_PERIOD,0x30);
  ret |= VL6180x_SetRegister(VL6180X_SYSALS_ANALOGUE_GAIN,0x40);
  ret |= VL6180x_SetRegister(VL6180X_FIRMWARE_RESULT_SCALER,0x01);

  return ret;
}





uint8_t VL6180x_GetIdentification(struct VL6180xIdentification *temp){

  uint8_t ret;

  ret = VL6180x_GetRegister(VL6180X_IDENTIFICATION_MODEL_ID, &temp->idModel);
  ret |= VL6180x_GetRegister(VL6180X_IDENTIFICATION_MODEL_REV_MAJOR, &temp->idModelRevMajor);
  ret |= VL6180x_GetRegister(VL6180X_IDENTIFICATION_MODEL_REV_MINOR, &temp->idModelRevMinor);
  ret |= VL6180x_GetRegister(VL6180X_IDENTIFICATION_MODULE_REV_MAJOR, &temp->idModuleRevMajor);
  ret |= VL6180x_GetRegister(VL6180X_IDENTIFICATION_MODULE_REV_MINOR, &temp->idModuleRevMinor);

  ret |= VL6180x_GetRegister16bit(VL6180X_IDENTIFICATION_DATE, &temp->idDate);
  ret |= VL6180x_GetRegister16bit(VL6180X_IDENTIFICATION_TIME, &temp->idTime);

  return ret;
}


uint16_t VL6180x_ReadId(void){

  uint8_t ret;


  return ret;
}


uint8_t VL6180x_ChangeAddress(uint8_t old_address, uint8_t new_address){

  //NOTICE:  IT APPEARS THAT CHANGING THE ADDRESS IS NOT STORED IN NON-VOLATILE MEMORY
  // POWER CYCLING THE DEVICE REVERTS ADDRESS BACK TO 0X29

  if( old_address == new_address) return old_address;
  if( new_address > 127) return old_address;

//  if(VL6180x_SetRegister(VL6180X_I2C_ADDRESS, new_address))
//    VL6180x_GetRegister(VL6180X_I2C_ADDRESS, &old_address);

  return old_address;
}


uint8_t VL6180x_GetDistance(void)
{
  uint8_t distance = 0;

  VL6180x_SetRegister(VL6180X_SYSRANGE_START, 0x01); //Start Single shot mode
  ToF_IO_Delay(10);

  if(VL6180x_GetRegister(VL6180X_RESULT_RANGE_VAL, &distance))
    VL6180x_SetRegister(VL6180X_SYSTEM_INTERRUPT_CLEAR, 0x07);

  return distance;
}


float VL6180x_GetAmbientLight(uint8_t als_gain)
{
  uint16_t alsRaw, alsIntegrationPeriodRaw;
  //First load in Gain we are using, do it everytime incase someone changes it on us.
  //Note: Upper nibble shoudl be set to 0x4 i.e. for ALS gain of 1.0 write 0x46
  VL6180x_SetRegister(VL6180X_SYSALS_ANALOGUE_GAIN, (0x40 | als_gain)); // Set the ALS gain

  //Start ALS Measurement
  VL6180x_SetRegister(VL6180X_SYSALS_START, 0x01);

  ToF_IO_Delay(100); //give it time...

  VL6180x_SetRegister(VL6180X_SYSTEM_INTERRUPT_CLEAR, 0x07);

  //Retrieve the Raw ALS value from the sensoe
  VL6180x_GetRegister16bit(VL6180X_RESULT_ALS_VAL, &alsRaw);

  //Get Integration Period for calculation, we do this everytime incase someone changes it on us.
  VL6180x_GetRegister16bit(VL6180X_SYSALS_INTEGRATION_PERIOD, &alsIntegrationPeriodRaw);

  float alsIntegrationPeriod = 100.0 / alsIntegrationPeriodRaw ;

  //Calculate actual LUX from Appnotes

  float alsGain = 0.0;

  switch (als_gain){
    case GAIN_20: alsGain = 20.0; break;
    case GAIN_10: alsGain = 10.32; break;
    case GAIN_5: alsGain = 5.21; break;
    case GAIN_2_5: alsGain = 2.60; break;
    case GAIN_1_67: alsGain = 1.72; break;
    case GAIN_1_25: alsGain = 1.28; break;
    case GAIN_1: alsGain = 1.01; break;
    case GAIN_40: alsGain = 40.0; break;
  }

//Calculate LUX from formula in AppNotes

  float alsCalculated = (float)0.32 * ((float)alsRaw / alsGain) * alsIntegrationPeriod;

  return alsCalculated;
}

// --- Private Functions --- //

uint8_t VL6180x_GetRegister(uint16_t registerAddr, uint8_t *data)
{
  return ToF_IO_Read(registerAddr, data);
}

uint8_t VL6180x_GetRegister16bit(uint16_t registerAddr, uint16_t *data)
{
  return ToF_IO_ReadHWord(registerAddr, data);
}

uint8_t VL6180x_SetRegister(uint16_t registerAddr, uint8_t data)
{
  return ToF_IO_Write(registerAddr, data);
}

uint8_t VL6180x_SetRegister16bit(uint16_t registerAddr, uint16_t data)
{
  return ToF_IO_WriteHWord(registerAddr, data);
}
