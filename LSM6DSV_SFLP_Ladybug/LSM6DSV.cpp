/* 09/24/2023 Copyright Tlera Corporation

    Created by Kris Winer

  This sketch uses SDA/SCL on pins 21/20 (Ladybugy default), respectively, and it uses the Ladybug (STM32L432KC) Breakout Board.

  https://www.tindie.com/products/tleracorp/ladybug-stm32l432-development-board/?pt=ac_prod_search
  
  The LSM6DSV is a combo accel and gyro, withsensor hub and and a finite state machine, with a lot of
  embedded functionality including a 6 DoF sensor fusion (gaming) engine.

  Library may be used freely and without limit with attribution.

*/

#include "LSM6DSV.h"
#include "I2Cdev.h"

LSM6DSV::LSM6DSV(I2Cdev* i2c_bus)
{
  _i2c_bus = i2c_bus;
}


uint8_t LSM6DSV::getChipID()
{
  uint8_t c = _i2c_bus->readByte(LSM6DSV_ADDRESS, LSM6DSV_WHO_AM_I);
  return c;
}


void LSM6DSV::accelPowerDown()
{
  uint8_t temp = _i2c_bus->readByte(LSM6DSV_ADDRESS, LSM6DSV_CTRL1);
  _i2c_bus->writeByte(LSM6DSV_ADDRESS, LSM6DSV_CTRL1, temp & ~(0x0F) ); // clear ODR bits to power down accel
}


void LSM6DSV::accelPowerUp(uint8_t AODR)
{
  uint8_t temp = _i2c_bus->readByte(LSM6DSV_ADDRESS, LSM6DSV_CTRL1);
  _i2c_bus->writeByte(LSM6DSV_ADDRESS, LSM6DSV_CTRL1, temp | AODR);  // restore accel ODR to power up
}


void LSM6DSV::gyroPowerDown()
{
  uint8_t temp = _i2c_bus->readByte(LSM6DSV_ADDRESS, LSM6DSV_CTRL2);
  _i2c_bus->writeByte(LSM6DSV_ADDRESS, LSM6DSV_CTRL2,  temp & ~(0x0F) ); // clear ODR bits to power down accel
}


void LSM6DSV::gyroPowerUp(uint8_t GODR)
{
  uint8_t temp = _i2c_bus->readByte(LSM6DSV_ADDRESS, LSM6DSV_CTRL2); 
  _i2c_bus->writeByte(LSM6DSV_ADDRESS, LSM6DSV_CTRL2,  temp | GODR); // restore gyro ODR to power up
}


void LSM6DSV::gyroSleep()
{
  uint8_t tempg = _i2c_bus->readByte(LSM6DSV_ADDRESS, LSM6DSV_CTRL2);
  tempg = tempg & ~(0x70); // clear bits 4 - 6
  _i2c_bus->writeByte(LSM6DSV_ADDRESS, LSM6DSV_CTRL2,  tempg | 0x40 );  // enable gyro sleep mode for faster wake up
}


void LSM6DSV::gyroWake(uint8_t G_PwrMode)
{
  uint8_t tempg = _i2c_bus->readByte(LSM6DSV_ADDRESS, LSM6DSV_CTRL2);
  tempg =  tempg & ~(0x70);   // clear bits 4 - 6
  _i2c_bus->writeByte(LSM6DSV_ADDRESS, LSM6DSV_CTRL2,  tempg | G_PwrMode ); // restore gyro Op Mode
}


float LSM6DSV::getAres(uint8_t Ascale) {
  switch (Ascale)
  {
    // Possible accelerometer scales (and their register bit settings) are:
    // 2 Gs (00), 4 Gs (01), 8 Gs (10), and 16 Gs  (11).
    case AFS_2G:
      _aRes = 2.0f / 32768.0f;
      return _aRes;
      break;
    case AFS_4G:
      _aRes = 4.0f / 32768.0f;
      return _aRes;
      break;
    case AFS_8G:
      _aRes = 8.0f / 32768.0f;
      return _aRes;
      break;
    case AFS_16G:
      _aRes = 16.0f / 32768.0f;
      return _aRes;
      break;
  }
}

float LSM6DSV::getGres(uint8_t Gscale) {
  switch (Gscale)
  {
    case GFS_125DPS:
      _gRes = 125.0f / 32768.0f;
      return _gRes;
      break;
    case GFS_250DPS:
      _gRes = 250.0f / 32768.0f;
      return _gRes;
      break;
    case GFS_500DPS:
      _gRes = 500.0f / 32768.0f;
      return _gRes;
      break;
    case GFS_1000DPS:
      _gRes = 1000.0f / 32768.0f;
      return _gRes;
      break;
    case GFS_2000DPS:
      _gRes = 2000.0f / 32768.0f;
      return _gRes;
      break;
    case GFS_4000DPS:
      _gRes = 4000.0f / 32768.0f;
      return _gRes;
      break;
  }
}


void LSM6DSV::reset()
{
  // reset device
  uint8_t temp = _i2c_bus->readByte(LSM6DSV_ADDRESS, LSM6DSV_CTRL3);
  _i2c_bus->writeByte(LSM6DSV_ADDRESS, LSM6DSV_CTRL3, temp | 0x01); // Set bit 0 to 1 to reset LSM6DSV
  delay(1); // Wait for all registers to reset
}


uint8_t LSM6DSV::DRstatus()
{
  uint8_t temp = _i2c_bus->readByte(LSM6DSV_ADDRESS, LSM6DSV_STATUS_REG); // read ststus register
  return temp;
}


uint8_t LSM6DSV::ACTstatus()
{
  uint8_t temp = _i2c_bus->readByte(LSM6DSV_ADDRESS, LSM6DSV_ALL_INT_SRC); // read INT SRC register
  return temp;
}


void LSM6DSV::ActivityDetect(uint8_t sleepMode)
{
  uint8_t temp = _i2c_bus->readByte(LSM6DSV_ADDRESS, LSM6DSV_FUNCTIONS_ENABLE);    // preserve existing configuration
  temp = temp & ~(0x03);                                                           // clear bits 0 and 1
  // set bit 3 == 1 to allow not resetting INT_ALL SRC status register before other status registers can be read
  _i2c_bus->writeByte(LSM6DSV_ADDRESS, LSM6DSV_FUNCTIONS_ENABLE, temp | sleepMode);      

  _i2c_bus->writeByte(LSM6DSV_ADDRESS, LSM6DSV_INACTIVITY_DUR, 0x34);              // set LSB to 62.5 mg, accel ODR to 15 Hz
  _i2c_bus->writeByte(LSM6DSV_ADDRESS, LSM6DSV_INACTIVITY_THS, 0x01);              // set wake THS to 1 x 62.5 mg

  temp = _i2c_bus->readByte(LSM6DSV_ADDRESS, LSM6DSV_WAKE_UP_DUR);                 // preserve existing configuration
  temp = temp & ~(0x0F);                                                           // clear bits 0 - 3
  _i2c_bus->writeByte(LSM6DSV_ADDRESS, LSM6DSV_WAKE_UP_DUR, temp | 0x05);          // set sleep duration to 5 x 512/accel_ODR ~10 s at 240 Hz
}


void LSM6DSV::singleTapDetect()
{
  uint8_t temp = _i2c_bus->readByte(LSM6DSV_ADDRESS, LSM6DSV_TAP_CFG0);    // preserve existing configuration
  temp = temp & ~(0x0E);                                                   // clear bits 1 - 3
//  _i2c_bus->writeByte(LSM6DSV_ADDRESS, LSM6DSV_TAP_CFG0, temp | 0x0E);     // enable tap detection on X, Y, Z
  _i2c_bus->writeByte(LSM6DSV_ADDRESS, LSM6DSV_TAP_CFG0, temp | 0x0F);     // enable tap detection on X, Y, Z, latch interrupt
  
  _i2c_bus->writeByte(LSM6DSV_ADDRESS, LSM6DSV_TAP_CFG1, 0x09);            // set x-axis threshold and axes priority
  _i2c_bus->writeByte(LSM6DSV_ADDRESS, LSM6DSV_TAP_CFG2, 0x09);            // set y-axis threshold and enable interrupt

   temp = _i2c_bus->readByte(LSM6DSV_ADDRESS, LSM6DSV_TAP_THS_6D);         // preserve interrupt existing configuration  
   temp = temp & ~(0x1F);                                                  // clear bits 0 - 4
  _i2c_bus->writeByte(LSM6DSV_ADDRESS, LSM6DSV_TAP_THS_6D, temp | 0x09);   // set z-axis threshold 
  
   temp = _i2c_bus->readByte(LSM6DSV_ADDRESS, LSM6DSV_TAP_DUR);            // preserve existing configuration  
   temp = temp & ~(0x0F);                                                  // clear bits 0 - 3
  _i2c_bus->writeByte(LSM6DSV_ADDRESS, LSM6DSV_TAP_DUR, temp | 0x06);      // set quiet and shock time windows
}


void LSM6DSV::D6DOrientationDetect()
{
  uint8_t temp = _i2c_bus->readByte(LSM6DSV_ADDRESS, LSM6DSV_TAP_CFG0);    // preserve existing configuration
  _i2c_bus->writeByte(LSM6DSV_ADDRESS, LSM6DSV_TAP_CFG0, temp | 0x41);     // set latch mode with reset on read
  temp = _i2c_bus->readByte(LSM6DSV_ADDRESS, LSM6DSV_TAP_THS_6D);         // preserve interrupt existing configuration  
  temp = temp & ~(0x60);                                                  // clear bits 5 and 6
 _i2c_bus->writeByte(LSM6DSV_ADDRESS, LSM6DSV_TAP_THS_6D, temp | 0x40);   // set 6D threshold to 60 degrees
}


uint8_t LSM6DSV::EMBstatus()
{
  _i2c_bus->writeByte(LSM6DSV_ADDRESS, LSM6DSV_FUNC_CFG_ACCESS, 0x80);         // enable embedded function access
  uint8_t temp = _i2c_bus->readByte(LSM6DSV_ADDRESS, LSM6DSV_EMB_FUNC_STATUS); // read embedded function status register
  _i2c_bus->writeByte(LSM6DSV_ADDRESS, LSM6DSV_FUNC_CFG_ACCESS, 0x00);         // disable embedded function access
  return temp;
}


uint8_t LSM6DSV::wakeSource()
{
  uint8_t temp = _i2c_bus->readByte(LSM6DSV_ADDRESS, LSM6DSV_WAKE_UP_SRC); // read WAKE_UP source register
  return temp;
}


uint8_t LSM6DSV::D6DSource()
{
  uint8_t temp = _i2c_bus->readByte(LSM6DSV_ADDRESS, LSM6DSV_D6D_SRC); // read D6D source register
  return temp;
}


uint8_t LSM6DSV::TapSource()
{
  uint8_t temp = _i2c_bus->readByte(LSM6DSV_ADDRESS, LSM6DSV_TAP_SRC); // read tap source register
  return temp;
}


void LSM6DSV::SFLP(uint8_t SFLPODR)
{
  // embedded function-Tilt detect
  _i2c_bus->writeByte(LSM6DSV_ADDRESS, LSM6DSV_FUNC_CFG_ACCESS, 0x80);                 // enable embedded function access
  _i2c_bus->writeByte(LSM6DSV_ADDRESS, LSM6DSV_EMB_FUNC_EN_A, 0x02);                   // enable SFLP_GAME
  _i2c_bus->writeByte(LSM6DSV_ADDRESS, LSM6DSV_EMB_FUNC_FIFO_EN_A, 0x12);              // enable batching of game vector and gravity
  _i2c_bus->writeByte(LSM6DSV_ADDRESS, LSM6DSV_SFLP_ODR, 0x40 | SFLPODR << 3 | 0x03);  // set SFLP ODR
  _i2c_bus->writeByte(LSM6DSV_ADDRESS, LSM6DSV_EMB_FUNC_INIT_A, 0x02);                 // initialize SLFP game vectors
  _i2c_bus->writeByte(LSM6DSV_ADDRESS, LSM6DSV_FUNC_CFG_ACCESS, 0x00);                 // disable embedded function access
}


void LSM6DSV::initFIFO(uint8_t fmode, uint8_t wtm, uint8_t FIFO_AODR, uint8_t FIFO_GODR) 
{
   _i2c_bus->writeByte(LSM6DSV_ADDRESS, LSM6DSV_FIFO_CTRL1, wtm);          // set FIFO watermark
   _i2c_bus->writeByte(LSM6DSV_ADDRESS, LSM6DSV_FIFO_CTRL2, 0x80);         // enable FIFO stop on watermark 
   _i2c_bus->writeByte(LSM6DSV_ADDRESS, LSM6DSV_FIFO_CTRL3, FIFO_GODR << 4 | FIFO_AODR);   // store accel/gyro data in FIFO at specified rate (<= AODR, GODR)
   _i2c_bus->writeByte(LSM6DSV_ADDRESS, LSM6DSV_FIFO_CTRL4, fmode);        // set FIFO to FIFO mode
   uint8_t temp = _i2c_bus->readByte(LSM6DSV_ADDRESS, LSM6DSV_INT1_CTRL);  // preserve existing configuration
   temp = temp & ~(0x38);                                                  // clear bits 3 - 5
   _i2c_bus->writeByte(LSM6DSV_ADDRESS, LSM6DSV_INT1_CTRL, temp | 0x38);   // route FIFO interrupts to INT1
}


void LSM6DSV::FIFOStatus(uint8_t * dest)
{
  uint8_t rawData[2]= {0, 0};
  _i2c_bus->readBytes(LSM6DSV_ADDRESS, LSM6DSV_FIFO_STATUS1, 2, &rawData[0]);
  dest[0] = rawData[0];
  dest[1] = rawData[1];
}


void LSM6DSV::FIFOReset()
{
   _i2c_bus->writeByte(LSM6DSV_ADDRESS, LSM6DSV_FIFO_CTRL1, 0x00);        // reset FIFO watermark to 0
   _i2c_bus->writeByte(LSM6DSV_ADDRESS, LSM6DSV_FIFO_CTRL4, 0x00);        // set FIFO to BYPASS mode
}


void LSM6DSV::FIFOOutput(uint8_t * dest)
{
    uint8_t rawData[7];  // FIFO data stored here
    _i2c_bus->readBytes(LSM6DSV_ADDRESS, LSM6DSV_FIFO_DATA_OUT_TAG, 7, &rawData[0]); 
    dest[0] = rawData[0]; 
    dest[1] = rawData[1]; 
    dest[2] = rawData[2]; 
    dest[3] = rawData[3]; 
    dest[4] = rawData[4]; 
    dest[5] = rawData[5]; 
    dest[6] = rawData[6]; 
 }
 
void LSM6DSV::init(uint8_t Ascale, uint8_t Gscale, uint8_t AODR, uint8_t GODR, uint8_t A_PwrMode, uint8_t G_PwrMode)
{
  _i2c_bus->writeByte(LSM6DSV_ADDRESS, LSM6DSV_CTRL1, A_PwrMode << 4 | AODR); // set accel power mode and sample rate

  _i2c_bus->writeByte(LSM6DSV_ADDRESS, LSM6DSV_CTRL2, G_PwrMode << 4 | GODR); // set gyro power mode and sample rate

  // enable block update (bit 6 = 1), auto-increment registers (bit 2 = 1)
  _i2c_bus->writeByte(LSM6DSV_ADDRESS, LSM6DSV_CTRL3, 0x40 | 0x04);
  
  // enable pulsed data ready interrupt signal (bit 1 = 1), set INT2 active HIGH for embedded functions (bit 0 = 1)
  _i2c_bus->writeByte(LSM6DSV_ADDRESS, LSM6DSV_CTRL4, 0x03);

  // Set gyro FS (bits 0 - 3, gyro LPF1 bandwidth (bits 4 - 6)
  _i2c_bus->writeByte(LSM6DSV_ADDRESS, LSM6DSV_CTRL6, Gscale);
//  _i2c_bus->writeByte(LSM6DSV_ADDRESS, LSM6DSV_CTRL7, 0x01);  // enable gyro LPF1 filter
//  uint8_t LPF1 = 0x04;  // ~100 Hz at 240 - 7680 Hz
//  _i2c_bus->writeByte(LSM6DSV_ADDRESS, LSM6DSV_CTRL6, LPF1 << 4 } Gscale);

  // Set accel FS (bits 0 - 3, accel LPF2 and HP filter bandwidth (bits 4 - 6)
  _i2c_bus->writeByte(LSM6DSV_ADDRESS, LSM6DSV_CTRL8, Ascale); // LPF is ODR/2 unless LPF1 is enabled
//  _i2c_bus->writeByte(LSM6DSV_ADDRESS, LSM6DSV_CTRL9, 0x08); // enable LPF2
//  uint8_t LPF2 = 0x01; // ODR/10
//  _i2c_bus->writeByte(LSM6DSV_ADDRESS, LSM6DSV_CTRL8, LPF2 << 5 | Ascale);

  // interrupt handling
  _i2c_bus->writeByte(LSM6DSV_ADDRESS, LSM6DSV_INT1_CTRL, 0x00);        // route gyro and accel data ready interrupts to INT1
  _i2c_bus->writeByte(LSM6DSV_ADDRESS, LSM6DSV_FUNCTIONS_ENABLE, 0x80); // enable all embedded functions interrupts 
  _i2c_bus->writeByte(LSM6DSV_ADDRESS, LSM6DSV_MD2_CFG, 0xE6);          // route sleep change, single tap, 6D, and EMB Fncts to INT2
}

 
void LSM6DSV::selfTest()
{
  int16_t temp[7] = {0, 0, 0, 0, 0, 0, 0};
  int16_t accelPTest[3] = {0, 0, 0}, accelNTest[3] = {0, 0, 0}, gyroPTest[3] = {0, 0, 0}, gyroNTest[3] = {0, 0, 0};
  int16_t accelNom[3] = {0, 0, 0}, gyroNom[3] = {0, 0, 0};

  readAccelGyroData(temp);
  accelNom[0] = temp[4];
  accelNom[1] = temp[5];
  accelNom[2] = temp[6];
  gyroNom[0]  = temp[1];
  gyroNom[1]  = temp[2];
  gyroNom[2]  = temp[3];

  _i2c_bus->writeByte(LSM6DSV_ADDRESS, LSM6DSV_CTRL10, 0x01); // positive accel self test
  delay(100); // let accel respond
  readAccelGyroData(temp);
  accelPTest[0] = temp[4];
  accelPTest[1] = temp[5];
  accelPTest[2] = temp[6];

  _i2c_bus->writeByte(LSM6DSV_ADDRESS, LSM6DSV_CTRL10, 0x02); // negative accel self test
  delay(100); // let accel respond
  readAccelGyroData(temp);
  accelNTest[0] = temp[4];
  accelNTest[1] = temp[5];
  accelNTest[2] = temp[6];

  _i2c_bus->writeByte(LSM6DSV_ADDRESS, LSM6DSV_CTRL10, 0x04); // positive gyro self test
  delay(100); // let gyro respond
  readAccelGyroData(temp);
  gyroPTest[0] = temp[1];
  gyroPTest[1] = temp[2];
  gyroPTest[2] = temp[3];

  _i2c_bus->writeByte(LSM6DSV_ADDRESS, LSM6DSV_CTRL10, 0x08); // negative gyro self test
  delay(100); // let gyro respond
  readAccelGyroData(temp);
  gyroNTest[0] = temp[1];
  gyroNTest[1] = temp[2];
  gyroNTest[2] = temp[3];

  _i2c_bus->writeByte(LSM6DSV_ADDRESS, LSM6DSV_CTRL10, 0x00); // normal mode
  delay(100); // let accel and gyro respond

  Serial.println("Accel Self Test:");
  Serial.print("+Ax results:"); Serial.print(  (accelPTest[0] - accelNom[0]) * _aRes * 1000.0); Serial.println(" mg");
  Serial.print("-Ax results:"); Serial.println((accelNTest[0] - accelNom[0]) * _aRes * 1000.0);
  Serial.print("+Ay results:"); Serial.println((accelPTest[1] - accelNom[1]) * _aRes * 1000.0);
  Serial.print("-Ay results:"); Serial.println((accelNTest[1] - accelNom[1]) * _aRes * 1000.0);
  Serial.print("+Az results:"); Serial.println((accelPTest[2] - accelNom[2]) * _aRes * 1000.0);
  Serial.print("-Az results:"); Serial.println((accelNTest[2] - accelNom[2]) * _aRes * 1000.0);
  Serial.println("Should be between 90 and 1700 mg");
  Serial.println(" ");

  Serial.println("Gyro Self Test:");
  Serial.print("+Gx results:"); Serial.print((gyroPTest[0] - gyroNom[0]) * _gRes); Serial.println(" dps");
  Serial.print("-Gx results:"); Serial.println((gyroNTest[0] - gyroNom[0]) * _gRes);
  Serial.print("+Gy results:"); Serial.println((gyroPTest[1] - gyroNom[1]) * _gRes);
  Serial.print("-Gy results:"); Serial.println((gyroNTest[1] - gyroNom[1]) * _gRes);
  Serial.print("+Gz results:"); Serial.println((gyroPTest[2] - gyroNom[2]) * _gRes);
  Serial.print("-Gz results:"); Serial.println((gyroNTest[2] - gyroNom[2]) * _gRes);
  Serial.println("Should be between 20 and 80 dps");
  Serial.println(" ");
  delay(2000);
}


void LSM6DSV::AGoffsetBias(float * dest1, float * dest2)
{
  int16_t temp[7] = {0, 0, 0, 0, 0, 0, 0};
  int32_t sum[7] = {0, 0, 0, 0, 0, 0, 0};

  Serial.println("Calculate accel and gyro offset biases: keep sensor flat and motionless!");
  delay(10000);

  for (uint8_t ii = 0; ii < 128; ii++)
  {
    readAccelGyroData(temp);
    sum[1] += temp[1];
    sum[2] += temp[2];
    sum[3] += temp[3];
    sum[4] += temp[4];
    sum[5] += temp[5];
    sum[6] += temp[6];
    delay(50);
  }

  dest1[0] = sum[1] * _gRes / 128.0f;
  dest1[1] = sum[2] * _gRes / 128.0f;
  dest1[2] = sum[3] * _gRes / 128.0f;
  dest2[0] = sum[4] * _aRes / 128.0f;
  dest2[1] = sum[5] * _aRes / 128.0f;
  dest2[2] = sum[6] * _aRes / 128.0f;

  if (dest2[0] > 0.75f)  {
    dest2[0] -= 1.0f; // Remove gravity from the x-axis accelerometer bias calculation
  }
  if (dest2[0] < -0.75f) {
    dest2[0] += 1.0f; // Remove gravity from the x-axis accelerometer bias calculation
  }
  if (dest2[1] > 0.75f)  {
    dest2[1] -= 1.0f; // Remove gravity from the y-axis accelerometer bias calculation
  }
  if (dest2[1] < -0.75f) {
    dest2[1] += 1.0f; // Remove gravity from the y-axis accelerometer bias calculation
  }
  if (dest2[2] > 0.75f)  {
    dest2[2] -= 1.0f; // Remove gravity from the z-axis accelerometer bias calculation
  }
  if (dest2[2] < -0.75f) {
    dest2[2] += 1.0f; // Remove gravity from the z-axis accelerometer bias calculation
  }

}

 
void LSM6DSV::readAccelGyroData(int16_t * destination)
{
  uint8_t rawData[14];  // x/y/z accel register data stored here
  _i2c_bus->readBytes(LSM6DSV_ADDRESS, LSM6DSV_OUT_TEMP_L, 14, &rawData[0]);  // Read the 14 raw data registers into data array
  destination[0] = (int16_t)((int16_t)rawData[1] << 8)  | rawData[0] ;  // Turn the MSB and LSB into a signed 16-bit value
  destination[1] = (int16_t)((int16_t)rawData[3] << 8)  | rawData[2] ;
  destination[2] = (int16_t)((int16_t)rawData[5] << 8)  | rawData[4] ;
  destination[3] = (int16_t)((int16_t)rawData[7] << 8)  | rawData[6] ;
  destination[4] = (int16_t)((int16_t)rawData[9] << 8)  | rawData[8] ;
  destination[5] = (int16_t)((int16_t)rawData[11] << 8) | rawData[10] ;
  destination[6] = (int16_t)((int16_t)rawData[13] << 8) | rawData[12] ;
}


uint16_t LSM6DSV::FloattoHalf(float f)
{
  uint32_t x = *((uint32_t*)&f);
  uint16_t h = ((x>>16)&0x8000)|((((x&0x7F800000)-0x38000000)>>13)&0x7C00)|((x>>13)&0x03FF);
  return h;
}


float LSM6DSV::HalftoFloat(uint16_t n)
{
    uint16_t frac = (n & 0x03FF) | 0x0400;
    int  exp = ((n & 0x7C00) >> 10) - 25;
    float m;

    if(frac == 0 && exp == 0x1F)
       m = INFINITY;
    else if (frac || exp)
        m = frac * pow(2, exp);
    else
        m = 0;

    return (n & 0x8000) ? -m : m;
}


void LSM6DSV::convertQ(uint16_t * fifoHF, float * sflpq)
{
  // construct three quaternions
  sflpq[1] = HalftoFloat(fifoHF[0]); // x, convert HF to float quaternions
  sflpq[2] = HalftoFloat(fifoHF[1]); // y
  sflpq[3] = HalftoFloat(fifoHF[2]); // z
 
  // derive the fourth quaternion from the normalization condition
  float sumsq = sflpq[1]*sflpq[1] + sflpq[2]*sflpq[2] + sflpq[3]*sflpq[3];
  
  if (sumsq > 1.0f) {
  float n = sqrtf(sumsq);
  sflpq[1] /= n;
  sflpq[2] /= n;
  sflpq[3] /= n;
  sumsq = 1.0f;
  }
  
  sflpq[0] = sqrtf( 1.0f - sumsq); // w
}
