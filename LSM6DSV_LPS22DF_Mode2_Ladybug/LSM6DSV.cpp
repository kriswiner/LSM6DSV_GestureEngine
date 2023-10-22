/* 09/24/2023 Copyright Tlera Corporation

    Created by Kris Winer

  This sketch uses SDA/SCL on pins 21/20 (Ladybug default), respectively, and it uses the Ladybug (STM32L432KC) Breakout Board.

  https://www.tindie.com/products/tleracorp/ladybug-stm32l432-development-board/
  
  The LSM6DSV is a combo accel and gyro, withsensor hub and and a finite state machine, with a lot of
  embedded functionality including a 6 DoF sensor fusion (gaming) engine.

  Library may be used freely and without limit with attribution.

*/

#include "LSM6DSV.h"
#include "LPS22DF.h"
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
  temp = temp & ~(0x0F); // clear bits 0 - 3
  _i2c_bus->writeByte(LSM6DSV_ADDRESS, LSM6DSV_CTRL1, temp | AODR);  // restore accel ODR to power up
}


void LSM6DSV::gyroPowerDown()
{
  uint8_t temp = _i2c_bus->readByte(LSM6DSV_ADDRESS, LSM6DSV_CTRL2);
  _i2c_bus->writeByte(LSM6DSV_ADDRESS, LSM6DSV_CTRL2,  temp & ~(0x0F) ); // clear ODR bits to power down gyro
}


void LSM6DSV::gyroPowerUp(uint8_t GODR)
{
  uint8_t temp = _i2c_bus->readByte(LSM6DSV_ADDRESS, LSM6DSV_CTRL2); 
  temp = temp & ~(0x0F); // clear bits 0 - 3
  _i2c_bus->writeByte(LSM6DSV_ADDRESS, LSM6DSV_CTRL2,  temp | GODR); // restore gyro ODR to power up
}


void LSM6DSV::gyroSleep()
{
  uint8_t temp = _i2c_bus->readByte(LSM6DSV_ADDRESS, LSM6DSV_CTRL2);
  temp = temp & ~(0x70); // clear bits 4 - 6
  _i2c_bus->writeByte(LSM6DSV_ADDRESS, LSM6DSV_CTRL2,  temp | 0x40 );  // enable gyro sleep mode for faster wake up
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
      _gRes = 0.004375f;
      return _gRes;
      break;
    case GFS_250DPS:
      _gRes = 0.00875f;
      return _gRes;
      break;
    case GFS_500DPS:
      _gRes = 0.01750f;
      return _gRes;
      break;
    case GFS_1000DPS:
      _gRes = 0.0350f;
      return _gRes;
      break;
    case GFS_2000DPS:
      _gRes = 0.070f;
      return _gRes;
      break;
    case GFS_4000DPS:
      _gRes = 0.140f;
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


void LSM6DSV::ActivityDetect(uint8_t inactiveTime, uint8_t inactODR, uint8_t sleepMode)
{
  uint8_t temp = _i2c_bus->readByte(LSM6DSV_ADDRESS, LSM6DSV_FUNCTIONS_ENABLE);             // preserve existing configuration
  temp = temp & ~(0x03);                                                                    // clear bits 0 and 1
//  when bit 3 is set, reading the ALL_INT_SRC (1Dh) register does not reset the latched interrupt signals.// 
// _i2c_bus->writeByte(LSM6DSV_ADDRESS, LSM6DSV_FUNCTIONS_ENABLE, temp | 0x08 | sleepMode);    // set sleepMode    
  _i2c_bus->writeByte(LSM6DSV_ADDRESS, LSM6DSV_FUNCTIONS_ENABLE, temp | sleepMode);         // set sleepMode    

  _i2c_bus->writeByte(LSM6DSV_ADDRESS, LSM6DSV_INACTIVITY_DUR, 0x03 << 4 | inactODR << 2);  // set LSB to 62.5 mg, inactivity accel ODR  
  _i2c_bus->writeByte(LSM6DSV_ADDRESS, LSM6DSV_INACTIVITY_THS, 0x01);                       // set wake THS to 1 x 62.5 mg

  temp = _i2c_bus->readByte(LSM6DSV_ADDRESS, LSM6DSV_WAKE_UP_DUR);                          // preserve existing configuration
  temp = temp & ~(0x0F);                                                                    // clear bits 0 - 3
  _i2c_bus->writeByte(LSM6DSV_ADDRESS, LSM6DSV_WAKE_UP_DUR, temp | inactiveTime);           // set inactive duration to inactiveTime x 512/accel_ODR  seconds

  temp = _i2c_bus->readByte(LSM6DSV_ADDRESS, LSM6DSV_MD1_CFG);                              // preserve interrupt existing routing
  _i2c_bus->writeByte(LSM6DSV_ADDRESS, LSM6DSV_MD1_CFG, temp | 0x80);                       // add sleep change detect routing to INT1
}


void LSM6DSV::singleTapDetect()
{
  uint8_t temp = _i2c_bus->readByte(LSM6DSV_ADDRESS, LSM6DSV_TAP_CFG0);    // preserve existing configuration
  temp = temp & ~(0x0E);                                                   // clear bits 1 - 3
  _i2c_bus->writeByte(LSM6DSV_ADDRESS, LSM6DSV_TAP_CFG0, temp | 0x0F);     // enable tap detection on X, Y, Z, don't latch interrupt
  
  _i2c_bus->writeByte(LSM6DSV_ADDRESS, LSM6DSV_TAP_CFG1, 0x09);            // set x-axis threshold and axes priority
  _i2c_bus->writeByte(LSM6DSV_ADDRESS, LSM6DSV_TAP_CFG2, 0x09);            // set y-axis threshold and enable interrupt

   temp = _i2c_bus->readByte(LSM6DSV_ADDRESS, LSM6DSV_TAP_THS_6D);         // preserve interrupt existing configuration  
   temp = temp & ~(0x1F);                                                  // clear bits 0 - 4
  _i2c_bus->writeByte(LSM6DSV_ADDRESS, LSM6DSV_TAP_THS_6D, temp | 0x09);   // set z-axis threshold 
  
   temp = _i2c_bus->readByte(LSM6DSV_ADDRESS, LSM6DSV_TAP_DUR);            // preserve existing configuration  
   temp = temp & ~(0x0F);                                                  // clear bits 0 - 3
  _i2c_bus->writeByte(LSM6DSV_ADDRESS, LSM6DSV_TAP_DUR, temp | 0x06);      // set quiet and shock time windows

  temp = _i2c_bus->readByte(LSM6DSV_ADDRESS, LSM6DSV_MD1_CFG);             // preserve interrupt existing routing
  _i2c_bus->writeByte(LSM6DSV_ADDRESS, LSM6DSV_MD1_CFG, temp | 0x40);      // add single tap routing to INT1
}


void LSM6DSV::D6DOrientationDetect()
{
  uint8_t temp = _i2c_bus->readByte(LSM6DSV_ADDRESS, LSM6DSV_TAP_CFG0);    // preserve existing configuration
  _i2c_bus->writeByte(LSM6DSV_ADDRESS, LSM6DSV_TAP_CFG0, temp | 0x40);     // set low-pass filter for 6D detection
  temp = _i2c_bus->readByte(LSM6DSV_ADDRESS, LSM6DSV_TAP_THS_6D);          // preserve interrupt existing configuration  
  temp = temp & ~(0x60);                                                   // clear bits 5 and 6
 _i2c_bus->writeByte(LSM6DSV_ADDRESS, LSM6DSV_TAP_THS_6D, temp | 0x40);    // set 6D threshold to 60 degrees
  temp = _i2c_bus->readByte(LSM6DSV_ADDRESS, LSM6DSV_MD1_CFG);             // preserve interrupt existing routing
 _i2c_bus->writeByte(LSM6DSV_ADDRESS, LSM6DSV_MD1_CFG, temp | 0x04);       // add 6D rotation detect routing to INT1
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


void LSM6DSV::TiltDetect()
{
  // embedded function-Tilt detect
  _i2c_bus->writeByte(LSM6DSV_ADDRESS, LSM6DSV_FUNC_CFG_ACCESS, 0x80);         // enable embedded function access
  uint8_t temp = _i2c_bus->readByte(LSM6DSV_ADDRESS, LSM6DSV_EMB_FUNC_EN_A);   // preserve existing configuration
  _i2c_bus->writeByte(LSM6DSV_ADDRESS, LSM6DSV_EMB_FUNC_EN_A, temp | 0x10);    // enable tilt detection
  temp = _i2c_bus->readByte(LSM6DSV_ADDRESS, LSM6DSV_EMB_FUNC_INT1);           // preserve existing configuration
  _i2c_bus->writeByte(LSM6DSV_ADDRESS, LSM6DSV_EMB_FUNC_INT1, temp | 0x10);    // route tilt detect interrupt to INT1
  _i2c_bus->writeByte(LSM6DSV_ADDRESS, LSM6DSV_FUNC_CFG_ACCESS, 0x00);         // disable embedded function access
  temp = _i2c_bus->readByte(LSM6DSV_ADDRESS, LSM6DSV_MD1_CFG);                 // preserve interrupt existing routing
  _i2c_bus->writeByte(LSM6DSV_ADDRESS, LSM6DSV_MD1_CFG, temp | 0x02);          // add embedded interrupt routing to INT1
}


void LSM6DSV::SFLP(uint8_t SFLPODR)
{
  // embedded function-SFLP
  _i2c_bus->writeByte(LSM6DSV_ADDRESS, LSM6DSV_FUNC_CFG_ACCESS, 0x80);                 // enable embedded function access
   uint8_t temp = _i2c_bus->readByte(LSM6DSV_ADDRESS, LSM6DSV_EMB_FUNC_EN_A);          // preserve existing configuration
  _i2c_bus->writeByte(LSM6DSV_ADDRESS, LSM6DSV_EMB_FUNC_EN_A, temp | 0x02);            // enable SFLP_GAME
  _i2c_bus->writeByte(LSM6DSV_ADDRESS, LSM6DSV_EMB_FUNC_FIFO_EN_A, 0x12);              // enable batching of game vector and gravity
  _i2c_bus->writeByte(LSM6DSV_ADDRESS, LSM6DSV_SFLP_ODR, 0x40 | SFLPODR << 3 | 0x03);  // set SFLP ODR
  _i2c_bus->writeByte(LSM6DSV_ADDRESS, LSM6DSV_FUNC_CFG_ACCESS, 0x00);                 // disable embedded function access
}


void LSM6DSV::initFIFO(uint8_t fmode, uint8_t wtm, uint8_t FIFO_AODR, uint8_t FIFO_GODR) 
{
   _i2c_bus->writeByte(LSM6DSV_ADDRESS, LSM6DSV_FIFO_CTRL1, wtm);          // set FIFO watermark
   _i2c_bus->writeByte(LSM6DSV_ADDRESS, LSM6DSV_FIFO_CTRL2, 0x80);         // enable FIFO stop on watermark 
   _i2c_bus->writeByte(LSM6DSV_ADDRESS, LSM6DSV_FIFO_CTRL3, FIFO_GODR << 4 | FIFO_AODR);   // store accel/gyro data in FIFO at specified rate (<= AODR, GODR)
   _i2c_bus->writeByte(LSM6DSV_ADDRESS, LSM6DSV_FIFO_CTRL4, fmode);        // set FIFO to FIFO mode
   _i2c_bus->writeByte(LSM6DSV_ADDRESS, LSM6DSV_INT1_CTRL,  0x38);         // route FIFO interrupts to INT1
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

//
//  copied from FSM example in ST AN5907 
//https://www.st.com/resource/en/application_note/an5907-lsm6dsv-finite-state-machine-stmicroelectronics.pdf
//
void LSM6DSV::FSMprograms() {
    _i2c_bus->writeByte(LSM6DSV_ADDRESS, LSM6DSV_FUNC_CFG_ACCESS, 0x80);                        // enable embedded function access
    _i2c_bus->writeByte(LSM6DSV_ADDRESS, LSM6DSV_EMB_FUNC_EN_B, 0x01);                          // enable FSM  
    uint8_t temp = _i2c_bus->readByte(LSM6DSV_ADDRESS, LSM6DSV_FSM_ODR);                        // preserve default register contants
    temp &= ~(0x38); // clear bits 3 - 5
    _i2c_bus->writeByte(LSM6DSV_ADDRESS, LSM6DSV_FSM_ODR, temp | 0x01 << 3);                    // select 30 Hz FSM ODR
    _i2c_bus->writeByte(LSM6DSV_ADDRESS, LSM6DSV_FSM_ENABLE, 0x0F);                             // Enable FSM engine 1, 2, 3, and 4
    _i2c_bus->writeByte(LSM6DSV_ADDRESS, LSM6DSV_FSM_INT1,   0x0F);                             // Route all FSM interrupts to INT1
    _i2c_bus->writeByte(LSM6DSV_ADDRESS, LSM6DSV_PAGE_RW,    0x40);                             // Enable page write

    _i2c_bus->writeByte(LSM6DSV_ADDRESS, LSM6DSV_PAGE_SEL,   0x11);                             // Select page 1 (bit 0 must always be 1)
    _i2c_bus->writeByte(LSM6DSV_ADDRESS, LSM6DSV_PAGE_ADDR,  0x7A);                             // Select FSM_LC_TIMEOUT_L register on page 1
    _i2c_bus->writeByte(LSM6DSV_ADDRESS, LSM6DSV_PAGE_VALUE, 0x00);                             // write 0 to FSM_LONG_COUNTER_L (register auto increments)
    _i2c_bus->writeByte(LSM6DSV_ADDRESS, LSM6DSV_PAGE_VALUE, 0x00);                             // write 0 to FSM_LONG_COUNTER_H
    _i2c_bus->writeByte(LSM6DSV_ADDRESS, LSM6DSV_PAGE_VALUE, 0x04);                             // write 4 to FSM_PROGRAMS  
    _i2c_bus->writeByte(LSM6DSV_ADDRESS, LSM6DSV_PAGE_VALUE, 0x01);                             // dummy write to skip to FSM_START_ADD_L register
    _i2c_bus->writeByte(LSM6DSV_ADDRESS, LSM6DSV_PAGE_VALUE, 0x00);                             // write to FSM_START_ADDRESS_L, 256-byte program here
    _i2c_bus->writeByte(LSM6DSV_ADDRESS, LSM6DSV_PAGE_VALUE, 0x04);                             // write to FSM_START_ADDRESS_H, page number here

    _i2c_bus->writeByte(LSM6DSV_ADDRESS, LSM6DSV_PAGE_SEL,   0x41);                             // Select page 4 (bit 0 must always be 1)
    _i2c_bus->writeByte(LSM6DSV_ADDRESS, LSM6DSV_PAGE_ADDR,  0x00);                             // Select first address on program page 4

// wrist tilt program 1
    _i2c_bus->writeByte(LSM6DSV_ADDRESS, LSM6DSV_PAGE_VALUE, 0x51);                             // write to CONFIG_A (one threshold, one mask, one short timer)
    _i2c_bus->writeByte(LSM6DSV_ADDRESS, LSM6DSV_PAGE_VALUE, 0x00);                             // write to CONFIG_B
    _i2c_bus->writeByte(LSM6DSV_ADDRESS, LSM6DSV_PAGE_VALUE, 0x10);                             // write to SIZE (16 byte program)
    _i2c_bus->writeByte(LSM6DSV_ADDRESS, LSM6DSV_PAGE_VALUE, 0x00);                             // write to SETTINGS
    _i2c_bus->writeByte(LSM6DSV_ADDRESS, LSM6DSV_PAGE_VALUE, 0x00);                             // write to RESET POINTER
    _i2c_bus->writeByte(LSM6DSV_ADDRESS, LSM6DSV_PAGE_VALUE, 0x00);                             // write to PROGRAM POINTER
    _i2c_bus->writeByte(LSM6DSV_ADDRESS, LSM6DSV_PAGE_VALUE, 0xAE);                             // write to THRESH1 LSB
    _i2c_bus->writeByte(LSM6DSV_ADDRESS, LSM6DSV_PAGE_VALUE, 0xB7);                             // write to THRESH1 MSB  (-0.4797 g in half precision floating point)
    _i2c_bus->writeByte(LSM6DSV_ADDRESS, LSM6DSV_PAGE_VALUE, 0x80);                             // write to MASKA  (+x axis)
    _i2c_bus->writeByte(LSM6DSV_ADDRESS, LSM6DSV_PAGE_VALUE, 0x00);                             // write to TMASKA
    _i2c_bus->writeByte(LSM6DSV_ADDRESS, LSM6DSV_PAGE_VALUE, 0x00);                             // write to TC
    _i2c_bus->writeByte(LSM6DSV_ADDRESS, LSM6DSV_PAGE_VALUE, 0x10);                             // write to TIMER3 (16 samples at 30 Hz = 0.62 seconds)
    _i2c_bus->writeByte(LSM6DSV_ADDRESS, LSM6DSV_PAGE_VALUE, 0x53);                             // write to GNTH1 | TI3
    _i2c_bus->writeByte(LSM6DSV_ADDRESS, LSM6DSV_PAGE_VALUE, 0x99);                             // write to OUTC
    _i2c_bus->writeByte(LSM6DSV_ADDRESS, LSM6DSV_PAGE_VALUE, 0x50);                             // write to GNTH1 | NOP
    _i2c_bus->writeByte(LSM6DSV_ADDRESS, LSM6DSV_PAGE_VALUE, 0x00);                             // write to STOP

// shake detect program 2
    _i2c_bus->writeByte(LSM6DSV_ADDRESS, LSM6DSV_PAGE_VALUE, 0xC0 | 0x20 | 0x02);               // write to CONFIG_A (three thresholds, two masks, two short timers)
    _i2c_bus->writeByte(LSM6DSV_ADDRESS, LSM6DSV_PAGE_VALUE, 0x00);                             // write to CONFIG_B
    _i2c_bus->writeByte(LSM6DSV_ADDRESS, LSM6DSV_PAGE_VALUE, 0x1E);                             // write to SIZE (30 byte program)
    _i2c_bus->writeByte(LSM6DSV_ADDRESS, LSM6DSV_PAGE_VALUE, 0x00);                             // write to SETTINGS
    _i2c_bus->writeByte(LSM6DSV_ADDRESS, LSM6DSV_PAGE_VALUE, 0x00);                             // write to RESET POINTER
    _i2c_bus->writeByte(LSM6DSV_ADDRESS, LSM6DSV_PAGE_VALUE, 0x00);                             // write to PROGRAM POINTER
    _i2c_bus->writeByte(LSM6DSV_ADDRESS, LSM6DSV_PAGE_VALUE, 0x66);                             // write to THRESH1 LSB
    _i2c_bus->writeByte(LSM6DSV_ADDRESS, LSM6DSV_PAGE_VALUE, 0x3E);                             // write to THRESH1 MSB  (set to + 1.6 g)
    _i2c_bus->writeByte(LSM6DSV_ADDRESS, LSM6DSV_PAGE_VALUE, 0x66);                             // write to THRESH2 LSB
    _i2c_bus->writeByte(LSM6DSV_ADDRESS, LSM6DSV_PAGE_VALUE, 0xBE);                             // write to THRESH2 MSB  (set to -1.6 g)
    _i2c_bus->writeByte(LSM6DSV_ADDRESS, LSM6DSV_PAGE_VALUE, 0xCD);                             // write to THRESH3 LSB
    _i2c_bus->writeByte(LSM6DSV_ADDRESS, LSM6DSV_PAGE_VALUE, 0x3C);                             // write to THRESH3 MSB  (set to + 1.2 g)
    _i2c_bus->writeByte(LSM6DSV_ADDRESS, LSM6DSV_PAGE_VALUE, 0xC0);                             // write to MASKA (+X and -X)
    _i2c_bus->writeByte(LSM6DSV_ADDRESS, LSM6DSV_PAGE_VALUE, 0x00);                             // write to TMASKA
    _i2c_bus->writeByte(LSM6DSV_ADDRESS, LSM6DSV_PAGE_VALUE, 0x02);                             // write to MASKB (+V)
    _i2c_bus->writeByte(LSM6DSV_ADDRESS, LSM6DSV_PAGE_VALUE, 0x00);                             // write to TMASKB
    _i2c_bus->writeByte(LSM6DSV_ADDRESS, LSM6DSV_PAGE_VALUE, 0x00);                             // write to TC
    _i2c_bus->writeByte(LSM6DSV_ADDRESS, LSM6DSV_PAGE_VALUE, 0x10);                             // write to TIMER3 (16 samples at 30 Hz = 0.62 seconds)
    _i2c_bus->writeByte(LSM6DSV_ADDRESS, LSM6DSV_PAGE_VALUE, 0x05);                             // write to TIMER4 ( 5 samples at 30 Hz = 0.19 seconds)
    _i2c_bus->writeByte(LSM6DSV_ADDRESS, LSM6DSV_PAGE_VALUE, 0x66);                             // write to SELMA Select MASKA and TMASKA as current mask
    _i2c_bus->writeByte(LSM6DSV_ADDRESS, LSM6DSV_PAGE_VALUE, 0xCC);                             // write to SELTHR1 Selects THRESH1 instead of THRESH3
    _i2c_bus->writeByte(LSM6DSV_ADDRESS, LSM6DSV_PAGE_VALUE, 0x35);                             // write to TI3 ! GNTH1  look for over/under/over threshold events == shaking
    _i2c_bus->writeByte(LSM6DSV_ADDRESS, LSM6DSV_PAGE_VALUE, 0x38);                             // write to TI3 ! LNTH2
    _i2c_bus->writeByte(LSM6DSV_ADDRESS, LSM6DSV_PAGE_VALUE, 0x35);                             // write to TI3 ! GNTH1
    _i2c_bus->writeByte(LSM6DSV_ADDRESS, LSM6DSV_PAGE_VALUE, 0x77);                             // write to SELMB Select MASKB and TMASKB as current mask
    _i2c_bus->writeByte(LSM6DSV_ADDRESS, LSM6DSV_PAGE_VALUE, 0xDD);                             // write to SELTHR3 Selects THRESH3 instead of THRESH1
    _i2c_bus->writeByte(LSM6DSV_ADDRESS, LSM6DSV_PAGE_VALUE, 0x03);                             // write to NOP | TI3
    _i2c_bus->writeByte(LSM6DSV_ADDRESS, LSM6DSV_PAGE_VALUE, 0x54);                             // write to GNTH1 | TI4
    _i2c_bus->writeByte(LSM6DSV_ADDRESS, LSM6DSV_PAGE_VALUE, 0x22);                             // write to CONTREL Continues execution from reset pointer, resetting temporary mask
    _i2c_bus->writeByte(LSM6DSV_ADDRESS, LSM6DSV_PAGE_VALUE, 0x00);                             // write to STOP

// motion detect program 3
    _i2c_bus->writeByte(LSM6DSV_ADDRESS, LSM6DSV_PAGE_VALUE, 0x51);                             // write to CONFIG_A
    _i2c_bus->writeByte(LSM6DSV_ADDRESS, LSM6DSV_PAGE_VALUE, 0x00);                             // write to CONFIG_B
    _i2c_bus->writeByte(LSM6DSV_ADDRESS, LSM6DSV_PAGE_VALUE, 0x14);                             // write to SIZE (20 byte program)
    _i2c_bus->writeByte(LSM6DSV_ADDRESS, LSM6DSV_PAGE_VALUE, 0x00);                             // write to SETTINGS
    _i2c_bus->writeByte(LSM6DSV_ADDRESS, LSM6DSV_PAGE_VALUE, 0x00);                             // write to RESET POINTER
    _i2c_bus->writeByte(LSM6DSV_ADDRESS, LSM6DSV_PAGE_VALUE, 0x00);                             // write to PROGRAM POINTER
    _i2c_bus->writeByte(LSM6DSV_ADDRESS, LSM6DSV_PAGE_VALUE, 0x66);                             // write to THRESH1 LSB
    _i2c_bus->writeByte(LSM6DSV_ADDRESS, LSM6DSV_PAGE_VALUE, 0x3C);                             // write to THRESH1 MSB  (set to + 1.0996 g)
    _i2c_bus->writeByte(LSM6DSV_ADDRESS, LSM6DSV_PAGE_VALUE, 0x02);                             // write to MASKA (+V)
    _i2c_bus->writeByte(LSM6DSV_ADDRESS, LSM6DSV_PAGE_VALUE, 0x00);                             // write to TMASKA
    _i2c_bus->writeByte(LSM6DSV_ADDRESS, LSM6DSV_PAGE_VALUE, 0x00);                             // write to TC
    _i2c_bus->writeByte(LSM6DSV_ADDRESS, LSM6DSV_PAGE_VALUE, 0x7D);                             // write to TIMER3 (125 samples at 30 Hz = 4.2 seconds)
    _i2c_bus->writeByte(LSM6DSV_ADDRESS, LSM6DSV_PAGE_VALUE, 0x05);                             // write to NOP | GNTH1 
    _i2c_bus->writeByte(LSM6DSV_ADDRESS, LSM6DSV_PAGE_VALUE, 0xC7);                             // write to UMSKIT Unmask interrupt generation when setting OUTS
    _i2c_bus->writeByte(LSM6DSV_ADDRESS, LSM6DSV_PAGE_VALUE, 0x99);                             // write to OUTC
    _i2c_bus->writeByte(LSM6DSV_ADDRESS, LSM6DSV_PAGE_VALUE, 0x33);                             // write to SRP Reset pointer to next address 
    _i2c_bus->writeByte(LSM6DSV_ADDRESS, LSM6DSV_PAGE_VALUE, 0x53);                             // write to GNTH1 | TI3
    _i2c_bus->writeByte(LSM6DSV_ADDRESS, LSM6DSV_PAGE_VALUE, 0x44);                             // write to CRP clear reset pointer to first program line
    _i2c_bus->writeByte(LSM6DSV_ADDRESS, LSM6DSV_PAGE_VALUE, 0xF5);                             // write to MSKIT Mask interrupt generation when setting OUTS
    _i2c_bus->writeByte(LSM6DSV_ADDRESS, LSM6DSV_PAGE_VALUE, 0x22);                             // write to CONTREL Continues execution from reset pointer, resetting temporary mask

// glance detection program 4
    _i2c_bus->writeByte(LSM6DSV_ADDRESS, LSM6DSV_PAGE_VALUE, 0x80 | 0x30 | 0x08 | 0x02);        // write to CONFIG_A (two thresholds, three masks, two long timers and two short timers)
    _i2c_bus->writeByte(LSM6DSV_ADDRESS, LSM6DSV_PAGE_VALUE, 0x00);                             // write to CONFIG_B
    _i2c_bus->writeByte(LSM6DSV_ADDRESS, LSM6DSV_PAGE_VALUE, 0x28);                             // write to SIZE (40 byte program)
    _i2c_bus->writeByte(LSM6DSV_ADDRESS, LSM6DSV_PAGE_VALUE, 0x00);                             // write to SETTINGS
    _i2c_bus->writeByte(LSM6DSV_ADDRESS, LSM6DSV_PAGE_VALUE, 0x00);                             // write to RESET POINTER
    _i2c_bus->writeByte(LSM6DSV_ADDRESS, LSM6DSV_PAGE_VALUE, 0x00);                             // write to PROGRAM POINTER
    _i2c_bus->writeByte(LSM6DSV_ADDRESS, LSM6DSV_PAGE_VALUE, 0x00);                             // write to THRESH1 LSB
    _i2c_bus->writeByte(LSM6DSV_ADDRESS, LSM6DSV_PAGE_VALUE, 0x38);                             // write to THRESH1 MSB  (set to + 0.5 g)
    _i2c_bus->writeByte(LSM6DSV_ADDRESS, LSM6DSV_PAGE_VALUE, 0x66);                             // write to THRESH2 LSB
    _i2c_bus->writeByte(LSM6DSV_ADDRESS, LSM6DSV_PAGE_VALUE, 0x2E);                             // write to THRESH2 MSB  (set to + 0.1 g)
    _i2c_bus->writeByte(LSM6DSV_ADDRESS, LSM6DSV_PAGE_VALUE, 0x80);                             // write to MASKA (+X)
    _i2c_bus->writeByte(LSM6DSV_ADDRESS, LSM6DSV_PAGE_VALUE, 0x00);                             // write to TMASKA
    _i2c_bus->writeByte(LSM6DSV_ADDRESS, LSM6DSV_PAGE_VALUE, 0x20);                             // write to MASKB (+Y)
    _i2c_bus->writeByte(LSM6DSV_ADDRESS, LSM6DSV_PAGE_VALUE, 0x00);                             // write to TMASKB
    _i2c_bus->writeByte(LSM6DSV_ADDRESS, LSM6DSV_PAGE_VALUE, 0x08);                             // write to MASKC (+Z)
    _i2c_bus->writeByte(LSM6DSV_ADDRESS, LSM6DSV_PAGE_VALUE, 0x00);                             // write to TMASKC
    _i2c_bus->writeByte(LSM6DSV_ADDRESS, LSM6DSV_PAGE_VALUE, 0x00);                             // write to TC
    _i2c_bus->writeByte(LSM6DSV_ADDRESS, LSM6DSV_PAGE_VALUE, 0x00);                             // write to TC
    _i2c_bus->writeByte(LSM6DSV_ADDRESS, LSM6DSV_PAGE_VALUE, 0x02);                             // write to TIMER1 ( 2 samples at 30 Hz = 0.066 seconds) LSB
    _i2c_bus->writeByte(LSM6DSV_ADDRESS, LSM6DSV_PAGE_VALUE, 0x00);                             //  MSB
    _i2c_bus->writeByte(LSM6DSV_ADDRESS, LSM6DSV_PAGE_VALUE, 0x20);                             // write to TIMER2 (32 samples at 32 Hz = 1.066 seconds) LSB
    _i2c_bus->writeByte(LSM6DSV_ADDRESS, LSM6DSV_PAGE_VALUE, 0x00);                             //  MSB
    _i2c_bus->writeByte(LSM6DSV_ADDRESS, LSM6DSV_PAGE_VALUE, 0x0A);                             // write to TIMER3 (16 samples at 30 Hz = 0.533 seconds)
    _i2c_bus->writeByte(LSM6DSV_ADDRESS, LSM6DSV_PAGE_VALUE, 0x03);                             // write to TIMER4 ( 3 samples at 30 Hz = 0.10 seconds)
    _i2c_bus->writeByte(LSM6DSV_ADDRESS, LSM6DSV_PAGE_VALUE, 0x23);                             // write to SINMUX Set input multiplexer
    _i2c_bus->writeByte(LSM6DSV_ADDRESS, LSM6DSV_PAGE_VALUE, 0x01);                             // select gyroscope
    _i2c_bus->writeByte(LSM6DSV_ADDRESS, LSM6DSV_PAGE_VALUE, 0x66);                             // write to SELMA Select MASKA and TMASKA as current mask
    _i2c_bus->writeByte(LSM6DSV_ADDRESS, LSM6DSV_PAGE_VALUE, 0x16);                             // write to TI1 ! GNTH2   look for >thresh2 in TI1 seconds
    _i2c_bus->writeByte(LSM6DSV_ADDRESS, LSM6DSV_PAGE_VALUE, 0x23);                             // write to SINMUX Set input multiplexer
    _i2c_bus->writeByte(LSM6DSV_ADDRESS, LSM6DSV_PAGE_VALUE, 0x07);                             // select integrated gyroscope
    _i2c_bus->writeByte(LSM6DSV_ADDRESS, LSM6DSV_PAGE_VALUE, 0x25);                             // write to TI2 ! GNTH1   look for >thresh1 in TI2 seconds
    _i2c_bus->writeByte(LSM6DSV_ADDRESS, LSM6DSV_PAGE_VALUE, 0x23);                             // write to SINMUX Set input multiplexer
    _i2c_bus->writeByte(LSM6DSV_ADDRESS, LSM6DSV_PAGE_VALUE, 0x00);                             // select accelerometer
    _i2c_bus->writeByte(LSM6DSV_ADDRESS, LSM6DSV_PAGE_VALUE, 0x03);                             // write to NOP | TI3
    _i2c_bus->writeByte(LSM6DSV_ADDRESS, LSM6DSV_PAGE_VALUE, 0x77);                             // write to SELMB Select MASKB and TMASKB as current mask
    _i2c_bus->writeByte(LSM6DSV_ADDRESS, LSM6DSV_PAGE_VALUE, 0x84);                             // write to LNTH2 | TI4
    _i2c_bus->writeByte(LSM6DSV_ADDRESS, LSM6DSV_PAGE_VALUE, 0x88);                             // write to SELMC Select MASKC and TMASKC as current mask
    _i2c_bus->writeByte(LSM6DSV_ADDRESS, LSM6DSV_PAGE_VALUE, 0x84);                             // write to LNTH2 | TI4
    _i2c_bus->writeByte(LSM6DSV_ADDRESS, LSM6DSV_PAGE_VALUE, 0x22);                             // write to CONTREL Continues execution from reset pointer, resetting temporary mask
    _i2c_bus->writeByte(LSM6DSV_ADDRESS, LSM6DSV_PAGE_VALUE, 0x00);                             // write to STOP

    _i2c_bus->writeByte(LSM6DSV_ADDRESS, LSM6DSV_PAGE_SEL,   0x01);                             // Disable access to embedded advanced features
    _i2c_bus->writeByte(LSM6DSV_ADDRESS, LSM6DSV_PAGE_RW,    0x00);                             // Disable page write
    _i2c_bus->writeByte(LSM6DSV_ADDRESS, LSM6DSV_FUNC_CFG_ACCESS, 0x00);                        // disable embedded function access

    temp = _i2c_bus->readByte(LSM6DSV_ADDRESS, LSM6DSV_MD1_CFG);                                // preserve interrupt existing routing
    _i2c_bus->writeByte(LSM6DSV_ADDRESS, LSM6DSV_MD1_CFG, temp | 0x02);                         // add embedded functions interrupt routing to INT1

    temp = _i2c_bus->readByte(LSM6DSV_ADDRESS, LSM6DSV_CTRL1);
    temp &= ~(0x0F);  // clear bits 0 - 3
    _i2c_bus->writeByte(LSM6DSV_ADDRESS, LSM6DSV_CTRL1, temp | 0x04);                          // set accel sample rate to 30 Hz
}


uint8_t LSM6DSV::FSMstatus()
 {
  _i2c_bus->writeByte(LSM6DSV_ADDRESS, LSM6DSV_FUNC_CFG_ACCESS, 0x80);         // enable embedded function access
  uint8_t temp = _i2c_bus->readByte(LSM6DSV_ADDRESS, LSM6DSV_FSM_STATUS);      // read FSM status register
  _i2c_bus->writeByte(LSM6DSV_ADDRESS, LSM6DSV_FUNC_CFG_ACCESS, 0x00);         // disable embedded function access
  return temp;
 }


void LSM6DSV::init(uint8_t Ascale, uint8_t Gscale, uint8_t AODR, uint8_t GODR, uint8_t A_PwrMode, uint8_t G_PwrMode)
{
  _i2c_bus->writeByte(LSM6DSV_ADDRESS, LSM6DSV_CTRL1, A_PwrMode << 4 | AODR); // set accel power mode and sample rate

  _i2c_bus->writeByte(LSM6DSV_ADDRESS, LSM6DSV_CTRL2, G_PwrMode << 4 | GODR); // set gyro power mode and sample rate

  // enable block update (bit 6 = 1), auto-increment registers (bit 2 = 1)
  _i2c_bus->writeByte(LSM6DSV_ADDRESS, LSM6DSV_CTRL3, 0x40 | 0x04);
  
  // enable pulsed data ready interrupt signal (bit 1 = 1), set INT2 active HIGH for embedded functions (bit 0 = 1)
//  _i2c_bus->writeByte(LSM6DSV_ADDRESS, LSM6DSV_CTRL4, 0x03);

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
//  _i2c_bus->writeByte(LSM6DSV_ADDRESS, LSM6DSV_INT1_CTRL, 0x00);        // route gyro and accel data ready interrupts to INT1
// Sensor Hub (bit 0), Enbedded functions (bit 1),  6D tilt (bit 2), wakeup (bit 5), single tap (bit 6), sleep change (bit 7)
  _i2c_bus->writeByte(LSM6DSV_ADDRESS, LSM6DSV_FUNCTIONS_ENABLE, 0x80); // enable embedded functions interrupts 
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


/*  Sensor Hub functions */
void LSM6DSV::passthruMode()
{
  _i2c_bus->writeByte(LSM6DSV_ADDRESS, LSM6DSV_FUNC_CFG_ACCESS, 0x40);                    // enable sensor hub access 
  uint8_t temp = _i2c_bus->readByte(LSM6DSV_ADDRESS, LSM6DSV_MASTER_CONFIG);              // preserve MASTER_CONFIG register
  // START_CONFIG = bit 5, PASSTHRU = bit 4, MASTER_ON = bit 2
  _i2c_bus->writeByte(LSM6DSV_ADDRESS, LSM6DSV_MASTER_CONFIG, temp  & ~(0x04));           // set MASTER_ON bit (bit 2) to 0
  delayMicroseconds(300);
  _i2c_bus->writeByte(LSM6DSV_ADDRESS, LSM6DSV_FUNC_CFG_ACCESS, 0x00);                    // disable sensor hub access

  // SHUB_PU_EN = bit 6 
  temp = _i2c_bus->readByte(LSM6DSV_ADDRESS, LSM6DSV_IF_CFG);                             // preserve IF_CFG register 
  _i2c_bus->writeByte(LSM6DSV_ADDRESS, LSM6DSV_IF_CFG, temp & ~(0x40));                   // disable pullups on master I2C lines

  _i2c_bus->writeByte(LSM6DSV_ADDRESS, LSM6DSV_FUNC_CFG_ACCESS, 0x40);                    // enable sensor hub access 
  temp = _i2c_bus->readByte(LSM6DSV_ADDRESS, LSM6DSV_MASTER_CONFIG);                      // preserve MASTER_CONFIG register
  // START_CONFIG = bit 5, PASSTHRU = bit 4, MASTER_ON = bit 2
  _i2c_bus->writeByte(LSM6DSV_ADDRESS, LSM6DSV_MASTER_CONFIG, temp | 0x10);               // enable pass through  
  _i2c_bus->writeByte(LSM6DSV_ADDRESS, LSM6DSV_FUNC_CFG_ACCESS, 0x00);                    // disable sensor hub access
}


void LSM6DSV::masterMode()
{
  _i2c_bus->writeByte(LSM6DSV_ADDRESS, LSM6DSV_FUNC_CFG_ACCESS, 0x40);                    // enable sensor hub access

  // Disable passthrough
  _i2c_bus->writeByte(LSM6DSV_ADDRESS, LSM6DSV_MASTER_CONFIG, 0x00);                      // disable passthrough mode
 
  // Configure LPS22DF slave
  _i2c_bus->writeByte(LSM6DSV_ADDRESS, LSM6DSV_SLV0_ADD, (LPS22DF_ADDRESS << 1) | 0x01);  // enable slave sensor read operation
  _i2c_bus->writeByte(LSM6DSV_ADDRESS, LSM6DSV_SLV0_SUBADD, LPS22DF_PRESS_OUT_XL);        // select slave sensor starting register for read operation
  // master rate (bits 5 - 7, send data to FIFO (bit 3), number of bytes to read (bits 0 - 2)
  _i2c_bus->writeByte(LSM6DSV_ADDRESS, LSM6DSV_SLV0_CONFIG, 0x20 | 0x08 | 0x05);          // 30 Hz, enable FIFO batching, five bytes to read

  // set write-once bit (bit 6) to 1, start config external trigger (bit 5 = 1), enable master mode (bit 2 = 1)    
  _i2c_bus->writeByte(LSM6DSV_ADDRESS, LSM6DSV_MASTER_CONFIG, 0x40 | 0x04 | 0x20);                        

  _i2c_bus->writeByte(LSM6DSV_ADDRESS, LSM6DSV_FUNC_CFG_ACCESS, 0x00);                    // disable sensor hub access

  // SHUB_PU_EN = bit 6 
  uint8_t temp = _i2c_bus->readByte(LSM6DSV_ADDRESS, LSM6DSV_IF_CFG);                     // preserve IF_CFG register 
  _i2c_bus->writeByte(LSM6DSV_ADDRESS, LSM6DSV_IF_CFG, temp | 0x40);                      // enable pullups on master I2C lines
}


 void LSM6DSV::idleLPS22DF()
{
  _i2c_bus->writeByte(LSM6DSV_ADDRESS, LSM6DSV_FUNC_CFG_ACCESS, 0x40);                   // enable sensor hub access

  _i2c_bus->writeByte(LSM6DSV_ADDRESS, LSM6DSV_MASTER_CONFIG, 0x00);                     // Disable master mode register
  delayMicroseconds(300);

  // Change Sensor Hub configuration
  _i2c_bus->writeByte(LSM6DSV_ADDRESS, LSM6DSV_SLV0_ADD, (LPS22DF_ADDRESS << 1));        // enable slave sensor write operation
  _i2c_bus->writeByte(LSM6DSV_ADDRESS, LSM6DSV_SLV0_SUBADD, LPS22DF_CTRL_REG1);          // select slave sensor starting register for read operation
  _i2c_bus->writeByte(LSM6DSV_ADDRESS, LSM6DSV_SLV0_CONFIG, 0x00);                       // 1.875 Hz 
  _i2c_bus->writeByte(LSM6DSV_ADDRESS, LSM6DSV_DATAWRITE_SLV0, 0x08);                    // put into lowest power mode that keeps INT2 working
  // Because we are using the save signal to trigger the sensor hub in master mode we need to use it here too
  _i2c_bus->writeByte(LSM6DSV_ADDRESS, LSM6DSV_MASTER_CONFIG,  0x64);                    // write once

  uint8_t statusMaster = _i2c_bus->readByte(LSM6DSV_ADDRESS, LSM6DSV_STATUS_MASTER);                                                            
  while(!(statusMaster & 0x80)) {
          statusMaster = _i2c_bus->readByte(LSM6DSV_ADDRESS, LSM6DSV_STATUS_MASTER);     // wait for WR_ONCE_DONE bit to be set
  }

  _i2c_bus->writeByte(LSM6DSV_ADDRESS, LSM6DSV_MASTER_CONFIG, 0x00);                     // Disable master mode register
  delayMicroseconds(300);
  
  _i2c_bus->writeByte(LSM6DSV_ADDRESS, LSM6DSV_FUNC_CFG_ACCESS, 0x00);                   // disable sensor hub access

  // SHUB_PU_EN = bit 6 
  uint8_t temp = _i2c_bus->readByte(LSM6DSV_ADDRESS, LSM6DSV_IF_CFG);                     // preserve IF_CFG register 
  _i2c_bus->writeByte(LSM6DSV_ADDRESS, LSM6DSV_IF_CFG, temp & ~(0x40));                   // disable pullups on master I2C lines
}


void LSM6DSV::resumeLPS22DF(uint8_t PODR, uint8_t AVG)
{
  // SHUB_PU_EN = bit 6 
  uint8_t temp = _i2c_bus->readByte(LSM6DSV_ADDRESS, LSM6DSV_IF_CFG);                     // preserve IF_CFG register 
  _i2c_bus->writeByte(LSM6DSV_ADDRESS, LSM6DSV_IF_CFG, temp | 0x40);                      // enable pullups on master I2C lines

  _i2c_bus->writeByte(LSM6DSV_ADDRESS, LSM6DSV_FUNC_CFG_ACCESS, 0x40);                    // enable sensor hub access

  // Change Sensor Hub configuration
  _i2c_bus->writeByte(LSM6DSV_ADDRESS, LSM6DSV_SLV0_ADD, (LPS22DF_ADDRESS << 1));         // enable slave sensor write operation
  _i2c_bus->writeByte(LSM6DSV_ADDRESS, LSM6DSV_SLV0_SUBADD, LPS22DF_CTRL_REG1);           // select slave sensor starting register for read operation
  _i2c_bus->writeByte(LSM6DSV_ADDRESS, LSM6DSV_SLV0_CONFIG, 0x00);                        // 1.875 Hz 
  _i2c_bus->writeByte(LSM6DSV_ADDRESS, LSM6DSV_DATAWRITE_SLV0, PODR << 3 | AVG);          // resume LPS22DF
  // Because we are using the save signal to trigger the sensor hub in master mode we need to use it here too
  _i2c_bus->writeByte(LSM6DSV_ADDRESS, LSM6DSV_MASTER_CONFIG,  0x64 );                    // write once

  uint8_t statusMaster = _i2c_bus->readByte(LSM6DSV_ADDRESS, LSM6DSV_STATUS_MASTER);                                                            
  while(!(statusMaster & 0x80)) {
          statusMaster = _i2c_bus->readByte(LSM6DSV_ADDRESS, LSM6DSV_STATUS_MASTER);     // wait for WR_ONCE_DONE bit to be set
}

  // Configure LPS22DF slave
  _i2c_bus->writeByte(LSM6DSV_ADDRESS, LSM6DSV_SLV0_ADD, (LPS22DF_ADDRESS << 1) | 0x01);  // enable slave sensor read operation
  _i2c_bus->writeByte(LSM6DSV_ADDRESS, LSM6DSV_SLV0_SUBADD, LPS22DF_PRESS_OUT_XL);        // select slave sensor starting register for read operation
  // master rate (bits 5 - 7, send data to FIFO (bit 3), number of bytes to read (bits 0 - 2)
  _i2c_bus->writeByte(LSM6DSV_ADDRESS, LSM6DSV_SLV0_CONFIG, 0x20 | 0x08 | 0x05);          // 30 Hz, enable FIFO batching, five bytes to read

  // set write-once bit (bit 6) to 1, start config external trigger (bit 5 = 1), enable master mode (bit 2 = 1)    
  _i2c_bus->writeByte(LSM6DSV_ADDRESS, LSM6DSV_MASTER_CONFIG, 0x40 | 0x04 | 0x20);       
   
  _i2c_bus->writeByte(LSM6DSV_ADDRESS, LSM6DSV_FUNC_CFG_ACCESS, 0x00);                   // disable sensor hub access
}


int32_t LSM6DSV::readBaroData()
{
    uint8_t rawData[3];  // 24-bit pressure register data stored here
  _i2c_bus->writeByte(LSM6DSV_ADDRESS, LSM6DSV_FUNC_CFG_ACCESS, 0x40);                   // enable sensor hub access
  _i2c_bus->readBytes(LSM6DSV_ADDRESS, LSM6DSV_SENSOR_HUB_1, 3, &rawData[0]);  
  _i2c_bus->writeByte(LSM6DSV_ADDRESS, LSM6DSV_FUNC_CFG_ACCESS, 0x00);                   // disable sensor hub access
  return (int32_t) ((int32_t) rawData[2] << 16 | (int32_t) rawData[1] << 8 | rawData[0]);
}


int16_t LSM6DSV::readBaroTemp()
{
    uint8_t rawData[2];  // 16-bit pressure register data stored here
  _i2c_bus->writeByte(LSM6DSV_ADDRESS, LSM6DSV_FUNC_CFG_ACCESS, 0x40);                   // enable sensor hub access
  _i2c_bus->readBytes(LSM6DSV_ADDRESS, LSM6DSV_SENSOR_HUB_4, 2, &rawData[0]);  
  _i2c_bus->writeByte(LSM6DSV_ADDRESS, LSM6DSV_FUNC_CFG_ACCESS, 0x00);                   // disable sensor hub access
  return (int16_t)((int16_t) rawData[1] << 8 | rawData[0]);
}
