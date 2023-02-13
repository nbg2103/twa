/*
 * tiva_thermometer.c
 *
 *  Created on: 20 Apr 2022
 *      Author: rafeeq
 */




#include "tim.h"
#include "adc.h"
#include "mlx90632.h"
#include "../tiVA_Application/tiVA_App/tiva_app.h"
//#include "sprk_mlx.h"


int8_t Thermometer_IO_Read(uint16_t RegAddr, uint16_t *pData);
int8_t Thermometer_IO_Write(uint16_t RegAddr, uint16_t Data);
void Thermometer_IO_mSDelay(uint32_t mSec);
void Thermometer_IO_uSDelay(int min_range, int max_range);
int8_t TempCalcDefault(void);
void MeasureTemp();
void MeasureTempExtended();
void MeasureTempBurst();
void MeasureTempExtendedBurst();


/*
 * Static Functions Related to Temperature Sensor
 */
static float FilterLoop(float nextInputValue);
static int8_t Thermometer_Init(void);
static int8_t Thermometer_CalculateTemperature(void);

/* TEMP data structure */
Thermometer_Data_t tivaThermometer = {.Initialised = 0};



/* Temperature Sensor structure initialization */
Thermometer_TypeDef Thermometer_Driver =
{
    Thermometer_Init,
    Thermometer_CalculateTemperature
};





/* HAL_I2C_Mem_Read()/Write() are used instead of Master_Transmit()/Receive() because repeated start condition is needed */
/* Implementation of I2C read for 16-bit values */
int32_t mlx90632_i2c_read(int16_t register_address, uint16_t *value)
{
  uint8_t data[2]="";
  int32_t ret;
  ret = HAL_I2C_Mem_Read(&hi2c2, MLX90632_I2C_ADDRESS, register_address, 2, data, sizeof(data), 100);
  //Endianness
  *value = data[1]|(data[0]<<8);
  return ret;
}

/* Implementation of I2C read for 32-bit values */
int32_t mlx90632_i2c_read32(int16_t register_address, uint32_t *value)
{
  uint8_t data[4]="";
  int32_t ret;
  ret = HAL_I2C_Mem_Read(&hi2c2, MLX90632_I2C_ADDRESS, register_address, 2, data, sizeof(data), 100);

  //Endianness
  *value = data[2]<<24|data[3]<<16|data[0]<<8|data[1];

  return ret;
}

/* Implementation of I2C write for 16-bit values */
int32_t mlx90632_i2c_write(int16_t register_address, uint16_t value) {
  uint8_t data[2]="";
  data[0] = value >> 8;
  data[1] = value;
  return HAL_I2C_Mem_Write(&hi2c2, MLX90632_I2C_ADDRESS, register_address, 2, data, 2, 100);
}

void usleep(int min_range, int max_range) {
  while(--min_range);
}

//void msleep(int msecs) {
 // Thermometer_IO_mSDelay(msecs);
//}



/* Implementation of reading all calibration parameters for calucation of Ta and To */
static int mlx90632_read_eeprom(int32_t *PR, int32_t *PG, int32_t *PO, int32_t *PT, int32_t *Ea, int32_t *Eb, int32_t *Fa, int32_t *Fb, int32_t *Ga, int16_t *Gb, int16_t *Ha, int16_t *Hb, int16_t *Ka)
{
	int32_t ret;
	ret = mlx90632_i2c_read32(MLX90632_EE_P_R, (uint32_t *) PR);
	if(ret < 0)
		return ret;
	ret = mlx90632_i2c_read32(MLX90632_EE_P_G, (uint32_t *) PG);
	if(ret < 0)
		return ret;
	ret = mlx90632_i2c_read32(MLX90632_EE_P_O, (uint32_t *) PO);
	if(ret < 0)
		return ret;
	ret = mlx90632_i2c_read32(MLX90632_EE_P_T, (uint32_t *) PT);
	if(ret < 0)
		return ret;
	ret = mlx90632_i2c_read32(MLX90632_EE_Ea, (uint32_t *) Ea);
	if(ret < 0)
		return ret;
	ret = mlx90632_i2c_read32(MLX90632_EE_Eb, (uint32_t *) Eb);
	if(ret < 0)
		return ret;
	ret = mlx90632_i2c_read32(MLX90632_EE_Fa, (uint32_t *) Fa);
	if(ret < 0)
		return ret;
	ret = mlx90632_i2c_read32(MLX90632_EE_Fb, (uint32_t *) Fb);
	if(ret < 0)
		return ret;
	ret = mlx90632_i2c_read32(MLX90632_EE_Ga, (uint32_t *) Ga);
	if(ret < 0)
		return ret;
	ret = mlx90632_i2c_read(MLX90632_EE_Gb, (uint16_t *) Gb);
	if(ret < 0)
		return ret;
	ret = mlx90632_i2c_read(MLX90632_EE_Ha, (uint16_t *) Ha);
	if(ret < 0)
		return ret;
	ret = mlx90632_i2c_read(MLX90632_EE_Hb, (uint16_t *) Hb);
	if(ret < 0)
		return ret;
	ret = mlx90632_i2c_read(MLX90632_EE_Ka, (uint16_t *) Ka);
	if(ret < 0)
		return ret;
	return 0;
}


/* Definition of MLX90632 calibration parameters */
volatile int16_t ambient_new_raw, ambient_old_raw, object_new_raw, object_old_raw;
volatile int32_t PR = 0x00587f5b, PG = 0x04a10289, PT = 0xfff966f8, PO = 0x00001e0f,
    Ea = 4859535, Eb = 5686508, Fa = 53855361, Fb = 42874149, Ga = -14556410;
volatile int16_t Ha = 16384, Hb = 0, Gb = 9728, Ka = 10752;


double pre_ambient, pre_object, pre_reflected;
//double tivaHead,tivaObject;

/**
  * @brief  Get Raw Temperature.
  * @param  None
  * @retval None
  */
int8_t Thermometer_CalculateTemperature(void)
{
  //TempCalcDefault();
  //SparkfunTemp();
   MeasureTemp();
  // MeasureTempExtended();
  // MeasureTempBurst();
  //MeasureTempExtendedBurst();
}

int8_t TempCalcDefault(void)
{
  int32_t ret = 0;

   /* Read sensor EEPROM registers needed for calcualtions */

 #ifdef __TEMPERATURE_EXTENDED
   /* Now we read current ambient and object temperature */
   mlx90632_read_temp_raw_extended(&ambient_new_raw, &ambient_old_raw,
   				       &object_new_raw);
 #else
   /* Now we read current ambient and object temperature */
   ret = mlx90632_read_temp_raw(&ambient_new_raw, &ambient_old_raw,
   			       &object_new_raw, &object_old_raw);
 #endif



   if(ret < 0)
       /* Something went wrong - abort */
       return ret;

 #ifdef __TEMPERATURE_EXTENDED
   /* Now start calculations (no more i2c accesses) */
   /* Calculate ambient temperature */
   tivaThermometer.HeadTemp = mlx90632_calc_temp_ambient_extended(ambient_new_raw, ambient_old_raw,
 								 PT, PR, PG, PO, Gb);
 #else
   /* Now start calculations (no more i2c accesses) */
   /* Calculate ambient temperature */
   tivaThermometer.HeadTemp = mlx90632_calc_temp_ambient(ambient_new_raw, ambient_old_raw,
 				       PT, PR, PG, PO, Gb);
 #endif


 #ifdef __TEMPERATURE_EXTENDED
   /* Get preprocessed temperatures needed for object temperature calculation */
   pre_ambient = mlx90632_preprocess_temp_ambient_extended(ambient_new_raw,
 							ambient_old_raw, Gb);
   pre_object = mlx90632_preprocess_temp_object_extended(object_new_raw, ambient_new_raw, ambient_old_raw, Ka);

   pre_reflected = mlx90632_calc_temp_object_reflected(pre_object, pre_ambient, pre_reflected, Ea, Eb, Ga, Fa, Fb, Ha, Hb);

   /* Calculate object temperature */
   tivaThermometer.LiquidTemp = mlx90632_calc_temp_object_extended(pre_object, pre_ambient, pre_reflected, Ea, Eb, Ga, Fa, Fb, Ha, Hb);


 #else
   /* Get preprocessed temperatures needed for object temperature calculation */
   pre_ambient = mlx90632_preprocess_temp_ambient(ambient_new_raw,
 							ambient_old_raw, Gb);
   pre_object = mlx90632_preprocess_temp_object(object_new_raw, object_old_raw,
 						      ambient_new_raw, ambient_old_raw,
 						      Ka);
   /* Calculate object temperature */
   tivaThermometer.LiquidTemp = mlx90632_calc_temp_object(pre_object, pre_ambient, Ea, Eb, Ga, Fa, Fb, Ha, Hb);

 #endif

   /* Get preprocessed temperatures needed for object temperature calculation */
   pre_ambient = mlx90632_preprocess_temp_ambient(ambient_new_raw,
 							ambient_old_raw, Gb);
   pre_object = mlx90632_preprocess_temp_object(object_new_raw, object_old_raw,
 						      ambient_new_raw, ambient_old_raw,
 						      Ka);
   /* Calculate object temperature */
   tivaThermometer.LiquidTemp = mlx90632_calc_temp_object(pre_object, pre_ambient, Ea, Eb, Ga, Fa, Fb, Ha, Hb);


   return true;
 }



/**
  * @brief  Initializes Temperature Sensor ADC.
  * @param  None
  * @retval true @ success else false
  */
static int8_t Thermometer_Init(void)
{
  int8_t ret = false;
  uint16_t temp[2] = {0};


  //start ADC in DMA mode
//  HAL_ADC_Start_DMA(&hadc1, (uint32_t*) &tivaIO.AdcValue[0], 5);
//  HAL_TIM_Base_Start(&htim4);

  /* Check the internal version and prepare a clean start */
  mlx90632_init();



  /* Read EEPROM calibration parameters */
  mlx90632_read_eeprom(&PR, &PG, &PO, &PT, &Ea, &Eb, &Fa, &Fb, &Ga, &Gb, &Ha, &Hb, &Ka);

  //Set Flag
  tivaThermometer.Initialised = 1;

  return ret;
}
///////////////////////////////////////////////////////////////////////////////////////////////////





///////////////////////////////////////////////////////////////////////////////////////////////////
void MeasureTemp()
{   int32_t ret = 0;
    ret = mlx90632_read_temp_raw(&ambient_new_raw, &ambient_old_raw,
                                 &object_new_raw, &object_old_raw);
    if(ret < 0)
        /* Something went wrong - abort */
        return ret;
    //Ha=4000;Hb=0;

    /* Get preprocessed temperatures needed for object temperature calculation */
    pre_ambient = mlx90632_preprocess_temp_ambient(ambient_new_raw,
                                                          ambient_old_raw, Gb);
    pre_object = mlx90632_preprocess_temp_object(object_new_raw, object_old_raw,
                                                        ambient_new_raw, ambient_old_raw,
                                                        Ka);
    mlx90632_set_emissivity(1.0);
    /* Now start calculations (no more i2c accesses) */

    /* Calculate ambient temperature */
    // tivaHead = mlx90632_calc_temp_ambient(ambient_new_raw, ambient_old_raw,PT, PR, PG, PO, Gb);
    //tivaThermometer.HeadTemp = tivaHead;
    tivaThermometer.HeadTemp = mlx90632_calc_temp_ambient(ambient_new_raw, ambient_old_raw,PT, PR, PG, PO, Gb);

    /* Calculate object temperature */
    //tivaObject = mlx90632_calc_temp_object(pre_object, pre_ambient, Ea, Eb, Ga, Fa, Fb, Ha, Hb);
    //tivaThermometer.LiquidTemp = tivaObject;
    tivaThermometer.LiquidTemp = mlx90632_calc_temp_object(pre_object, pre_ambient, Ea, Eb, Ga, Fa, Fb, Ha, Hb);

    //tivaThermometer.LiquidTemp = mlx90632_calc_temp_object_reflected(pre_object, pre_ambient,pre_object, Ea, Eb, Ga, Fa, Fb, Ha, Hb);
}

void MeasureTempExtended()
{
 int32_t ret = 0; /**< Variable will store return values */

    /* Read sensor EEPROM registers needed for calcualtions */

    /* You can check if the device supports extended measurement mode */
    ret = mlx90632_init();
    //if(status == ERANGE)
    {
       /* Extended mode is supported */
    }

    /* Set MLX90632 in extended mode */
    ret = mlx90632_set_meas_type(MLX90632_MTYP_EXTENDED);
    if(ret < 0)
        /* Something went wrong - abort */
        return ret;

    /* Now we read current ambient and object temperature */
    ret = mlx90632_read_temp_raw_extended(&ambient_new_raw, &ambient_old_raw, &object_new_raw);
    if(ret < 0)
        /* Something went wrong - abort */
        return ret;

    /* Now start calculations (no more i2c accesses) */
    /* Calculate ambient temperature */
   tivaThermometer.HeadTemp = mlx90632_calc_temp_ambient_extended(ambient_new_raw, ambient_old_raw,
                                                  PT, PR, PG, PO, Gb);

    /* Get preprocessed temperatures needed for object temperature calculation */
    pre_ambient = mlx90632_preprocess_temp_ambient_extended(ambient_new_raw,
                                                                   ambient_old_raw, Gb);
    pre_object = mlx90632_preprocess_temp_object_extended(object_new_raw, ambient_new_raw,
                                                                 ambient_old_raw, Ka);

    /* Calculate object temperature assuming the reflected temperature equals ambient*/
    tivaThermometer.LiquidTemp = mlx90632_calc_temp_object_extended(pre_object, pre_ambient, tivaThermometer.HeadTemp, Ea, Eb, Ga, Fa, Fb, Ha, Hb);
}

void MeasureTempBurst()
{
    int32_t ret = 0; /**< Variable will store return values */
    double ambient; /**< Ambient temperature in degrees Celsius */
    double object; /**< Object temperature in degrees Celsius */

    /* Read sensor EEPROM registers needed for calcualtions */

    /* Set MLX90632 in burst mode */
    ret = mlx90632_set_meas_type(MLX90632_MTYP_MEDICAL_BURST);
    if(ret < 0)
        /* Something went wrong - abort */
        return ret;

    /* Now we read current ambient and object temperature */
    ret = mlx90632_read_temp_raw_burst(&ambient_new_raw, &ambient_old_raw,
                                       &object_new_raw, &object_old_raw);
    if(ret < 0)
        /* Something went wrong - abort */
        return ret;

    /* Now start calculations (no more i2c accesses) */
    /* Calculate ambient temperature */
    tivaThermometer.HeadTemp = mlx90632_calc_temp_ambient(ambient_new_raw, ambient_old_raw,
			 				       PT, PR, PG, PO, Gb);

    /* Get preprocessed temperatures needed for object temperature calculation */
    pre_ambient = mlx90632_preprocess_temp_ambient(ambient_new_raw,
                                                          ambient_old_raw, Gb);
    pre_object = mlx90632_preprocess_temp_object(object_new_raw, object_old_raw,
                                                        ambient_new_raw, ambient_old_raw,
                                                        Ka);
    /* Calculate object temperature assuming the reflected temperature equals ambient*/
    tivaThermometer.LiquidTemp = mlx90632_calc_temp_object_reflected(pre_object, pre_ambient,ambient, Ea, Eb, Ga, Fa, Fb, Ha, Hb);

}
void MeasureTempExtendedBurst()
{
    int32_t ret = 0; /**< Variable will store return values */


    /* Read sensor EEPROM registers needed for calcualtions */

    /* You can check if the device supports extended measurement mode */
    ret = mlx90632_init();
   // if(status == ERANGE)
    {
       /* Extended mode is supported */
    }

    /* Set MLX90632 in extended burst mode */
    ret = mlx90632_set_meas_type(MLX90632_MTYP_EXTENDED_BURST);
    if(ret < 0)
        /* Something went wrong - abort */
        return ret;

    /* Now we read current ambient and object temperature */
    ret = mlx90632_read_temp_raw_extended_burst(&ambient_new_raw, &ambient_old_raw, &object_new_raw);
    if(ret < 0)
        /* Something went wrong - abort */
        return ret;

    /* Now start calculations (no more i2c accesses) */
    /* Calculate ambient temperature */
    tivaThermometer.HeadTemp = mlx90632_calc_temp_ambient_extended(ambient_new_raw, ambient_old_raw,
                                                  PT, PR, PG, PO, Gb);

    /* Get preprocessed temperatures needed for object temperature calculation */
    pre_ambient = mlx90632_preprocess_temp_ambient_extended(ambient_new_raw,
                                                                   ambient_old_raw, Gb);
    pre_object = mlx90632_preprocess_temp_object_extended(object_new_raw, ambient_new_raw,
                                                                 ambient_old_raw, Ka);

    /* Calculate object temperature assuming the reflected temperature equals ambient*/
    tivaThermometer.LiquidTemp = mlx90632_calc_temp_object_extended(pre_object, pre_ambient, tivaThermometer.HeadTemp, Ea, Eb, Ga, Fa, Fb, Ha, Hb);
}
///////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////
/**
  * @brief  Read Thermometer I2C Values.
  * @param  None
  * @retval None
  */
int8_t Thermometer_IO_Read(uint16_t RegAddr, uint16_t *pData)
{
  return HAL_I2C_Mem_Read(&hi2c2, MLX90632_I2C_ADDRESS, RegAddr, I2C_MEMADD_SIZE_16BIT, (uint8_t *)pData,2,100);
}

/**
  * @brief  Read Thermometer I2C Values.
  * @param  None
  * @retval None
  */
int8_t Thermometer_IO_Write(uint16_t RegAddr, uint16_t Data)
{
  return HAL_I2C_Mem_Write(&hi2c2, MLX90632_I2C_ADDRESS, RegAddr, I2C_MEMADD_SIZE_16BIT,(uint8_t *)&Data,2,100);
}


/**
  * @brief  Read Thermometer IO Delay in m Sec.
  * @param  None
  * @retval None
  */
void Thermometer_IO_mSDelay(uint32_t mSec)
{
  osDelay(mSec);
}

/**
  * @brief  Read Thermometer ADC Values.
  * @param  None
  * @retval None
  */
static void Thermometer_IO_AdcUpdate(void)
{
  float Vdda, error = 10;

  //Vdda Calculation with respect to Vref 1.8V
  Vdda = 1.8 * 4095.0/tivaIO.AdcValue[VREF_ADC];

  Vdda = 3.3;

  //TMJ = (VOUT)/(5 mV/°C)
  Vdda = (VREFINT_CAL_VREF * (*VREFINT_CAL_ADDR)/tivaIO.AdcValue[VREF_INT_ADC]);

  //TMJ = (VOUT)/(5 mV/°C)
  //LiquidTemp Calculation with respect to Vrefint
//  tivaThermometer.LiquidTemp = Vdda * tivaIO.AdcValue[C_TEMP_ADC] / (4095.0 * 0.005) - error;

  //TMJ = (VOUT)/(5 mV/°C)
  //CoilTemp Calculation with respect to Vrefint
//  tivaThermometer.CoilTemp = FilterLoop(Vdda * tivaIO.AdcValue[C_TEMP_ADC]/ (4095.0 * 0.005) - error);

  //HeadTemp Calculation with respect to Vrefint
//  tivaThermometer.HeadTemp = tivaIO.AdcValue[H_TEMP_ADC] * (25.0)/2047.0;

//  tivaThermometer.HeadTemp = ((Vdda * tivaIO.AdcValue[H_TEMP_ADC]/4095.0) - 0.76) / 0.025;

}

#define NZEROS 5
#define NPOLES 5
#define GAIN   1.672358808e+04

static float xv[NZEROS+1], yv[NPOLES+1];

static float FilterLoop(float nextInputValue)
  {
  float nextOutputValue = 0;

    xv[0] = xv[1]; xv[1] = xv[2]; xv[2] = xv[3]; xv[3] = xv[4]; xv[4] = xv[5];
    xv[5] = nextInputValue / GAIN;
    yv[0] = yv[1]; yv[1] = yv[2]; yv[2] = yv[3]; yv[3] = yv[4]; yv[4] = yv[5];
    yv[5] =   (xv[0] + xv[5]) + 5 * (xv[1] + xv[4]) + 10 * (xv[2] + xv[3])
	 + (  0.3599282451 * yv[0]) + ( -2.1651329097 * yv[1])
	 + (  5.2536151704 * yv[2]) + ( -6.4348670903 * yv[3])
	 + (  3.9845431196 * yv[4]);
    nextOutputValue = yv[5];

    return nextOutputValue;
  }
