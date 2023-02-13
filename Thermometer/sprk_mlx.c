/*
  This is a library written for the MLX90632 Non-contact thermal sensor
  SparkFun sells these at its website: www.sparkfun.com
  Do you like this library? Help support SparkFun. Buy a board!
  https://www.sparkfun.com/products/14569

  Written by Nathan Seidle @ SparkFun Electronics, December 28th, 2017

  The MLX90632 can remotely measure object temperatures within 1 degree C.

  This library handles the initialization of the MLX90632 and the calculations
  to get the temperatures.

  https://github.com/sparkfun/SparkFun_MLX90632_Arduino_Library

  Development environment specifics:
  Arduino IDE 1.8.3

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/
#include "sprk_mlx.h"
#include "math.h"
#include "main.h"
#include "i2c.h"
/*
  TODO:
  check EEPROM write
  check timing - how fast to take reading? Setting the SOC twice may be doubling time
  set emissivity
*/

  //Variables
    //TwoWire *_i2cPort; //The generic connection to user's chosen I2C hardware
    uint8_t _deviceAddress; //Keeps track of I2C address. setI2CAddress changes this. Either 0x3A or 0x3B (default)

    //Stream *_debugPort; //The stream to send debug messages to if enabled
    bool _printDebug = false; //Flag to print debugging variables

    const int MAX_WAIT = 750; //Number of ms to wait before giving up. Some sensor actions take 512ms.



//Declare global variables for the calibration values
 double P_R;
 double P_G;
 double P_T;
 double P_O;
extern double Ea;
extern double Eb;
extern double Fa;
extern double Fb;
extern double Ga;
extern double Gb;
extern double Ka;
extern double Ha;
extern double Hb;

double TOdut = 25.0; //Assume 25C for first iteration
double TO0 = 25.0; //object temp from previous calculation
double TA0 = 25.0; //ambient temp from previous calculation
double sensorTemp; //Internal temp of the MLX sensor

#include "sprk_mlx.h"

//Attempt communication with the device
//Return true if we got a 'Polo' back from Marco

status returnError;
//This begins the communication with the device
//Returns a status error if anything went wrong
bool begin(void)
{


  //We require caller to begin their I2C port, with the speed of their choice
  //external to the library
  //_i2cPort->begin();
  //We could to a check here to see if user has init the Wire or not. Would
  //need to be for different platforms

  //Check communication with IC
  uint16_t thisAddress;

  returnError = readRegister16(EE_I2C_ADDRESS, &thisAddress);
  if (thisAddress != _deviceAddress >> 1)
  {

    returnError = SENSOR_ID_ERROR;
    return (false); //Error
  }

  if (_printDebug)
  {
    uint16_t thisVersion;
    returnError = readRegister16(EE_VERSION, &thisVersion);
    //_debugPort->print(F("Sensor EEPROM version (usually 0x205): 0x"));
    //_debugPort->println(thisVersion, HEX);
  }

  //Wait for eeprom_busy to clear
  uint16_t counter = 0;
  while (eepromBusy())
  {
    HAL_Delay(1);
    counter++;
    if (counter == MAX_WAIT)
    {
      //if (_printDebug) _debugPort->println(F("EEPROM busy timeout"));
      //returnError = SENSOR_TIMEOUT_ERROR;
      return (false); //Error
    }
  }

  setMode(MODE_SLEEP); //Before reading EEPROM sensor needs to stop taking readings

  //Load all the static calibration factors
  int16_t tempValue16;
  int32_t tempValue32;
  readRegister32(EE_P_R, (uint32_t*)&tempValue32);
  P_R = (double)tempValue32 * pow(2, -8);
  readRegister32(EE_P_G, (uint32_t*)&tempValue32);
  P_G = (double)tempValue32 * pow(2, -20);
  readRegister32(EE_P_T, (uint32_t*)&tempValue32);
  P_T = (double)tempValue32 * pow(2, -44);
  readRegister32(EE_P_O, (uint32_t*)&tempValue32);
  P_O = (double)tempValue32 * pow(2, -8);
  readRegister32(EE_Ea, (uint32_t*)&tempValue32);
  Ea = (double)tempValue32 * pow(2, -16);
  readRegister32(EE_Eb, (uint32_t*)&tempValue32);
  Eb = (double)tempValue32 * pow(2, -8);
  readRegister32(EE_Fa, (uint32_t*)&tempValue32);
  Fa = (double)tempValue32 * pow(2, -46);
  readRegister32(EE_Fb, (uint32_t*)&tempValue32);
  Fb = (double)tempValue32 * pow(2, -36);
  readRegister32(EE_Ga, (uint32_t*)&tempValue32);
  Ga = (double)tempValue32 * pow(2, -36);

  readRegister16(EE_Gb, (uint16_t*)&tempValue16);
  Gb = (double)tempValue16 * pow(2, -10);
  readRegister16(EE_Ka, (uint16_t*)&tempValue16);
  Ka = (double)tempValue16 * pow(2, -10);
  readRegister16(EE_Ha, (uint16_t*)&tempValue16);
  Ha = (double)tempValue16 * pow(2, -14); //Ha!
  readRegister16(EE_Hb, (uint16_t*)&tempValue16);
  Hb = (double)tempValue16 * pow(2, -14);

  //Note, sensor is in sleep mode

  return (true);


}

//Read all calibration values and calculate the temperature of the thing we are looking at
//Depending on mode, initiates a measurement
//If in sleep or step mode, clears the new_data bit, sets the SOC bit
float getObjectTemp1()
{
  status returnError;
  return (getObjectTemp2(returnError));
}
float getObjectTemp2(status returnError)
{
  returnError = SENSOR_SUCCESS;

  //If the sensor is not in continuous mode then the tell sensor to take reading
  if(getMode1() != MODE_CONTINUOUS) setSOC();

  //Write new_data = 0
  clearNewData();

  //Check when new_data = 1
  uint16_t counter = 0;
  while (dataAvailable() == false)
  {
    HAL_Delay(1);
    counter++;
    if (counter == MAX_WAIT)
    {
      //if (_printDebug) _debugPort->println(F("Data available timeout"));
      //returnError = SENSOR_TIMEOUT_ERROR;
      return (0.0); //Error
    }
  }

  gatherSensorTemp(returnError);
  if (returnError != SENSOR_SUCCESS)
  {
    if (_printDebug)
    {
     // _debugPort->println(F("Sensor temperature not found"));
     // if(returnError == SENSOR_TIMEOUT_ERROR) _debugPort->println(F("Timeout"));
    }
    return (0.0); //Error
  }

  int16_t lowerRAM = 0;
  int16_t upperRAM = 0;

  //Get RAM_6 and RAM_9
  int16_t sixRAM;
  readRegister16(RAM_6, (uint16_t*)&sixRAM);
  int16_t nineRAM;
  readRegister16(RAM_9, (uint16_t*)&nineRAM);

  //Read cycle_pos to get measurement pointer
  int cyclePosition = getCyclePosition();

  //If cycle_pos = 1
  //Calculate TA and TO based on RAM_4, RAM_5, RAM_6, RAM_9
  if (cyclePosition == 1)
  {
    readRegister16(RAM_4, (uint16_t*)&lowerRAM);
    readRegister16(RAM_5, (uint16_t*)&upperRAM);
  }
  //If cycle_pos = 2
  //Calculate TA and TO based on RAM_7, RAM_8, RAM_6, RAM_9
  else if (cyclePosition == 2)
  {
    readRegister16(RAM_7, (uint16_t*)&lowerRAM);
    readRegister16(RAM_8, (uint16_t*)&upperRAM);
  }
  else
  {
    //if (_printDebug) _debugPort->println(F("Found a cycle position that was not 1 or 2"));
    readRegister16(RAM_4, (uint16_t*)&lowerRAM);
    readRegister16(RAM_5, (uint16_t*)&upperRAM);
  }

  //Object temp requires 3 iterations
  for (uint8_t i = 0 ; i < 3 ; i++)
  {
    double VRta = nineRAM + Gb * (sixRAM / 12.0);

    double AMB = (sixRAM / 12.0) / VRta * pow(2, 19);

    double sensorTemp = P_O + (AMB - P_R) / P_G + P_T * pow((AMB - P_R), 2);

    float S = (float)(lowerRAM + upperRAM) / 2.0;
    double VRto = nineRAM + Ka * (sixRAM / 12.0);
    double Sto = (S / 12.0) / VRto * (double)pow(2, 19);

    double TAdut = (AMB - Eb) / Ea + 25.0;

    double ambientTempK = TAdut + 273.15;

    double bigFraction = Sto / (1 * Fa * Ha * (1 + Ga * (TOdut - TO0) + Fb * (TAdut - TA0)));

    double objectTemp = bigFraction + pow(ambientTempK, 4);
    objectTemp = pow(objectTemp, 0.25); //Take 4th root
    objectTemp = objectTemp - 273.15 - Hb;

    TO0 = objectTemp;

    if (_printDebug)
    {
//      _debugPort->println();
//      _debugPort->print(F("VRta: "));
//      _debugPort->println(VRta);
//      _debugPort->print(F("AMB: "));
//      _debugPort->println(AMB, 10);
//      _debugPort->print(F("sensorTemp (Ta): "));
//      _debugPort->println(sensorTemp, 4);
//      _debugPort->print(F("S: "));
//      _debugPort->println(S);
//      _debugPort->print(F("VRto: "));
//      _debugPort->println(VRto);
//      _debugPort->print(F("Sto: "));
//      _debugPort->println(Sto);
//      _debugPort->print(F("TAdut: "));
//      _debugPort->println(TAdut, 10);
//      _debugPort->print(F("ambientTempK: "));
//      _debugPort->println(ambientTempK, 10);
//      _debugPort->print(F("bigFraction: "));
//      _debugPort->println(bigFraction, 4);
//
//      _debugPort->print(F("Object temp "));
//      _debugPort->print(i);
//      _debugPort->print(F(": "));
//      _debugPort->println(objectTemp, 7);
    }
  }

  return (TO0);
}

//Convert temp to F
float getObjectTempF()
{
  float tempC = getObjectTemp();
  float tempF = tempC * 9.0/5.0 + 32.0;
  return(tempF);
}

//Returns the current temperature of the sensor
float getSensorTemp1()
{
  status tempFlag;
  return (getSensorTemp2(tempFlag));
}

float getSensorTemp2(status returnError)
{
  returnError = SENSOR_SUCCESS;

  //If the sensor is not in continuous mode then the tell sensor to take reading
  if(getMode1() != MODE_CONTINUOUS) setSOC();

  //Write new_data = 0
  clearNewData();

  //Wait for new data
  uint16_t counter = 0;
  while (dataAvailable() == false)
  {
    HAL_Delay(1);
    counter++;
    if (counter == MAX_WAIT)
    {
      returnError = SENSOR_TIMEOUT_ERROR;
      return (0.0); //Error
    }
  }

  return (gatherSensorTemp(returnError));
}

//This reads all the temperature calibration factors for the sensor itself
//This is needed so that it can be called from getObjectTemp *and* users can call getSensorTemp
//without causing a double read
float gatherSensorTemp(status returnError)
{
  returnError = SENSOR_SUCCESS;

  //Get RAM_6 and RAM_9
  int16_t sixRAM;
  readRegister16(RAM_6, (uint16_t*)&sixRAM);
  int16_t nineRAM;
  readRegister16(RAM_9, (uint16_t*)&nineRAM);

  double VRta = nineRAM + Gb * (sixRAM / 12.0);

  double AMB = (sixRAM / 12.0) / VRta * pow(2, 19);

  double sensorTemp = P_O + (AMB - P_R) / P_G + P_T * pow((AMB - P_R), 2);

  if (_printDebug)
  {
//    _debugPort->print(F("sixRAM: "));
//    _debugPort->println(sixRAM);
//    _debugPort->print(F("nineRAM: "));
//    _debugPort->println(nineRAM);
//    _debugPort->print(F("VRta: "));
//    _debugPort->println(VRta, 7);
//    _debugPort->print(F("AMB: "));
//    _debugPort->println(AMB, 10);
//    _debugPort->print(F("sensorTemp (Ta): "));
//    _debugPort->println(sensorTemp, 4);
  }

  return(sensorTemp);
}

//Returns true if device is busy doing measurement
//Always true in continuous mode
bool deviceBusy()
{
  if (getStatus1() & ((uint16_t)1 << BIT_DEVICE_BUSY)) return (true);
  return (false);
}

//Returns true if eeprom is busy
//EEPROM is busy during boot up and during EEPROM write/erase
bool eepromBusy()
{
  if (getStatus1() & ((uint16_t)1 << BIT_EEPROM_BUSY)) return (true);
  return (false);
}

//Returns the cycle_pos from status register. cycle_pos is 0 to 31
uint8_t getCyclePosition()
{
  uint16_t status = getStatus1() >> BIT_CYCLE_POS; //Shave off last two bits
  status &= 0x1F; //Get lower 5 bits.
  return (status);
}

//Returns true if new data is available
bool dataAvailable()
{
  if (getStatus1() & ((uint16_t)1 << BIT_NEW_DATA)) return (true);
  return (false);
}

//Sets the brown_out bit. Datasheet says 'Customer should set bit to 1'. Ok.
void setBrownOut()
{
  uint16_t reg = getStatus(); //Get current bits
  reg |= (1 << BIT_BROWN_OUT); //Set the bit
  writeRegister16(REG_STATUS, reg); //Set the mode bits
}

//Clear the new_data bit. This is done after a measurement is complete
void clearNewData()
{
  uint16_t reg = getStatus1(); //Get current bits
  reg &= ~(1 << BIT_NEW_DATA); //Clear the bit
  writeRegister16(REG_STATUS, reg); //Set the mode bits
}

//Returns the REG_STATUS 16-bit register
uint16_t getStatus1()
{
  status returnError;
  return (getStatus2(returnError));
}
uint16_t getStatus2(status returnError)
{
  uint16_t deviceStatus;
  returnError = readRegister16(REG_STATUS, &deviceStatus);
  return (deviceStatus);
}

//Changes the mode to sleep
void sleepMode()
{
  setMode(MODE_SLEEP);
}

//Changes the mode to step
void stepMode()
{
  setMode(MODE_STEP);
}

//Changes the mode to continuous read
void continuousMode()
{
  setMode(MODE_CONTINUOUS);
}

//Sets the Start of Conversion (SOC) bit
status setSOC()
{
  uint16_t reg;
  status returnError = readRegister16(REG_CONTROL, &reg); //Get current bits
  reg |= (1 << 3); //Set the bit
  writeRegister16(REG_CONTROL, reg); //Set the bit
  return (returnError);
}

//Sets the sensing mode (3 modes availabe)
status setMode(uint8_t mode)
{
  uint16_t reg;
  status returnError = readRegister16(REG_CONTROL, &reg); //Get current bits
  reg &= ~(0x0003 << 1); //Clear the mode bits
  reg |= (mode << 1); //Set the bits
  writeRegister16(REG_CONTROL, reg); //Set the mode bits
  return (returnError);
}

//Returns the mode of the sensor
uint8_t getMode1()
{
  status returnError;
  return (getMode2(returnError));
}
uint8_t getMode2(status returnError)
{
  uint16_t mode;
  returnError = readRegister16(REG_CONTROL, &mode); //Get current register settings
  mode = (mode >> 1) & 0x0003; //Clear all other bits
  return (mode);
}

//Writes a new device I2C address into EEPROM
//This is probably a function to use carefully. If you set the address
//to something unknown you'll need to use the I2C scanner sketch to detect it again
void writeI2CAddress(uint8_t newAddress)
{

}

//Calling this function with nothing sets the debug port to Serial
//You can also call it with other streams like Serial1, SerialUSB, etc.
//void enableDebugging(Stream debugPort)
//{
//  //_debugPort = &debugPort;
//  //_printDebug = true;
//}

void disableDebugging(void)
{
  _printDebug = false;
}

//Reads two consecutive bytes from a given location
//Stores the result at the provided outputPointer
status readRegister16(uint16_t addr, uint16_t* outputPointer)
{
//  status returnError = SENSOR_SUCCESS; //By default, return success
//
//  _i2cPort->beginTransmission(_deviceAddress);
//  _i2cPort->write(addr >> 8); //MSB
//  _i2cPort->write(addr & 0xFF); //LSB
//  //_i2cPort->endTransmission(false); //Send a restart command. Do not release bus.
//  if (_i2cPort->endTransmission(false) != 0) //Send a restart command. Do not release bus.
//  {
//    //Sensor did not ACK
//    if (_printDebug) _debugPort->println(F("I2C Error: End transmission"));
//    returnError = SENSOR_I2C_ERROR;
//  }
//
//  _i2cPort->requestFrom((uint8_t)_deviceAddress, (uint8_t)2);
//  if (_i2cPort->available())
//  {
//    uint8_t msb = _i2cPort->read();
//    uint8_t lsb = _i2cPort->read();
//
//    outputPointer = (uint16_t)msb << 8 | lsb;
//  }
//  else
//  {
//    //Sensor did not respond
//    if (_printDebug) _debugPort->println(F("I2C Error: No read response"));
//    returnError = SENSOR_I2C_ERROR;
//  }
//
   uint8_t data[2]="";
   int32_t ret;
   ret = HAL_I2C_Mem_Read(&hi2c2, MLX90632_I2C_ADDRESS, addr, 2, data, sizeof(data), 100);
   //Endianness
   *outputPointer = data[1]|(data[0]<<8);
   if (ret == HAL_ERROR){
        returnError = SENSOR_I2C_ERROR;}

  return (returnError); //Report whether we were successful or not
}

//Reads two 16-bit registers and combines them into 32 bits
status readRegister32(uint16_t addr, uint32_t* outputPointer)
{
  status returnError = SENSOR_SUCCESS; //By default, return success

    uint8_t data[4];
    int32_t ret;
    ret = HAL_I2C_Mem_Read(&hi2c2, MLX90632_I2C_ADDRESS, addr, 2, data, sizeof(data), 100);
    //Endianness
    *outputPointer = data[2]<<24|data[3]<<16|data[0]<<8|data[1];
    if (ret == HAL_ERROR){
      returnError = SENSOR_I2C_ERROR;

  }


  return (returnError); //Report whether we were successful or not
}

//Write two bytes to a spot
status writeRegister16(uint16_t addr, uint16_t val)
{
  status returnError = SENSOR_SUCCESS; //By default, return success

    uint8_t data[2];
    data[0] = val >> 8;
    data[1] = val;
    if(HAL_I2C_Mem_Write(&hi2c2, MLX90632_I2C_ADDRESS, addr, 2, data, 2, 100)== HAL_ERROR)
    {
      returnError = SENSOR_I2C_ERROR;
    }
  return (returnError); //Report whether we were successful or not
}

//Write a value to EEPROM
//Requires unlocking the EEPROM, writing 0x0000, unlocking again, then writing value
//The datasheet doesn't go a good job of explaining how writing to EEPROM works.
//This should work but doesn't. It seems the IC is very sensitive to I2C traffic while
//the sensor is recording the new EEPROM.
void writeEEPROM(uint16_t addr, uint16_t val)
{
  //Put device into halt mode (page 15)
  uint8_t originalMode = getMode1();
  setMode(MODE_SLEEP);

  //Wait for complete
  while (deviceBusy()) HAL_Delay(1);

  //Magic unlock (page 17)
  writeRegister16(0x3005, 0x554C);

  //Wait for complete
  HAL_Delay(100);

  //Now we can write to one EEPROM word
  //Write 0x0000 to user's location (page 16) to erase
  writeRegister16(addr, 0x0000);

  //Wait for complete
  HAL_Delay(100);
  //while (eepromBusy()) delay(1);
  //while (deviceBusy()) delay(1);

  //Magic unlock again
  writeRegister16(0x3005, 0x554C);

  //Wait for complete
  HAL_Delay(100);

  //Now we can write to one EEPROM word
  writeRegister16(addr, val);

  //Wait for complete
  HAL_Delay(100);
  //while (eepromBusy()) delay(1);
  //while (deviceBusy()) delay(1);

  //Return to original mode
  setMode(originalMode);

}
