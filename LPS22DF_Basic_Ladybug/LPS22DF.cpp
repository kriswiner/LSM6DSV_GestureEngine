/*
 * Copyright (c) 2020 Tlera Corp.  All rights reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to
 * deal with the Software without restriction, including without limitation the
 * rights to use, copy, modify, merge, publish, distribute, sublicense, and/or
 * sell copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 *  1. Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimers.
 *  2. Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimers in the
 *     documentation and/or other materials provided with the distribution.
 *  3. Neither the name of Tlera Corp, nor the names of its contributors
 *     may be used to endorse or promote products derived from this Software
 *     without specific prior written permission.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL THE
 * CONTRIBUTORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
 * WITH THE SOFTWARE.
 * 
 * 
 * Library may be used freely and without limit with attribution.
 */

#include "LPS22DF.h"
#include "I2Cdev.h"

LPS22DF::LPS22DF(I2Cdev* i2c_bus)
{
  _i2c_bus = i2c_bus;
}

uint8_t LPS22DF::getChipID()
{
  // Read the WHO_AM_I register of the altimeter this is a good test of communication
  uint8_t temp = _i2c_bus->readByte(LPS22DF_ADDRESS, LPS22DF_WHOAMI);  // Read WHO_AM_I register for LPS22DF
  return temp;
}


void LPS22DF::boot()
  {
  uint8_t temp = _i2c_bus->readByte(LPS22DF_ADDRESS, LPS22DF_CTRL_REG2);  
  _i2c_bus->writeByte(LPS22DF_ADDRESS, LPS22DF_CTRL_REG2, temp | 0x80); // re-boot the LPS28DFW
  uint8_t status = 0;
  while(!status) { // wait for boot process to complete
   status = _i2c_bus->readByte(LPS22DF_ADDRESS, LPS22DF_INT_SOURCE) & 0x80; 
  } 
  }


uint8_t LPS22DF::status()
{
  // Read the status register of the altimeter  
  uint8_t temp = _i2c_bus->readByte(LPS22DF_ADDRESS, LPS22DF_STATUS);   
  return temp;
}


int32_t LPS22DF::Pressure()
{
    uint8_t rawData[3];  // 24-bit pressure register data stored here
    _i2c_bus->readBytes(LPS22DF_ADDRESS, LPS22DF_PRESS_OUT_XL, 3, &rawData[0]);  
    int32_t temp = (int32_t) ((int32_t) rawData[2] << 24 | (int32_t) rawData[1] << 16 | rawData[0] << 8);
    return (temp >> 8);
}


int16_t LPS22DF::Temperature()
{
    uint8_t rawData[2];  // 16-bit pressure register data stored here
    _i2c_bus->readBytes(LPS22DF_ADDRESS, LPS22DF_TEMP_OUT_L, 2, &rawData[0]);  
    return (int16_t)((int16_t) rawData[1] << 8 | rawData[0]);
}


void LPS22DF::reset()
{
    uint8_t temp = _i2c_bus->readByte(LPS22DF_ADDRESS, LPS22DF_CTRL_REG2);  
    _i2c_bus->writeByte(LPS22DF_ADDRESS, LPS22DF_CTRL_REG2, temp | 0x04);  // set software reset bit to reset device
    delay(1);
    }


void LPS22DF::powerDown()
{
    uint8_t temp = _i2c_bus->readByte(LPS22DF_ADDRESS, LPS22DF_CTRL_REG1);  
    _i2c_bus->writeByte(LPS22DF_ADDRESS, LPS22DF_CTRL_REG1, temp & ~(0x78) );  // clear bits 3 - 6
}


void LPS22DF::powerUp(uint8_t PODR)
{
    uint8_t temp = _i2c_bus->readByte(LPS22DF_ADDRESS, LPS22DF_CTRL_REG1);  
    _i2c_bus->writeByte(LPS22DF_ADDRESS, LPS22DF_CTRL_REG1, temp | PODR << 3 );  // write ODR to bits 3 - 6 to start continuous mode
}


void LPS22DF::oneShot()
{
    uint8_t temp = _i2c_bus->readByte(LPS22DF_ADDRESS, LPS22DF_CTRL_REG2);  
    _i2c_bus->writeByte(LPS22DF_ADDRESS, LPS22DF_CTRL_REG2, temp | 0x01 );  // set bit 0 for one shot measurement
}


void LPS22DF::Init(uint8_t PODR, uint8_t AVG, uint8_t LPF)
{
    _i2c_bus->writeByte(LPS22DF_ADDRESS, LPS22DF_IC3_IF_CTRL, 0x02);  // disable internal CS pullup
    _i2c_bus->writeByte(LPS22DF_ADDRESS, LPS22DF_CTRL_REG1, PODR << 3 | AVG);  
  // enable low-pass filter by setting bit 4 to one
  // bit 5 == 0 means bandwidth is odr/4, bit 5 == 1 means bandwidth is odr/4
  // make sure data not updated during read by setting block data update (bit 3) to 1    
    _i2c_bus->writeByte(LPS22DF_ADDRESS, LPS22DF_CTRL_REG2, LPF << 5 | 0x10 | 0x08); 
  // interrupt is push-pull (bit 1 = 0), active HIGH (bit 3 = 0) by default    
    _i2c_bus->writeByte(LPS22DF_ADDRESS, LPS22DF_CTRL_REG3, 0x01);  // enable auto increment of register addresses
  // Configure interrupt
  // Enable data ready on INT pin (bit 5 = 1), enable data ready pulse (bit 6 = 1)
  // Enable interrupt on INT pin (bit 4 = 1)
  // EnableFIFO full (bit = 2), WTM reached (bit = 1) and FIFO overflow (bit = 0) interrupts
    _i2c_bus->writeByte(LPS22DF_ADDRESS, LPS22DF_CTRL_REG4, 0x77);   
}


void LPS22DF::FIFOStatus(uint8_t * dest)
{
  uint8_t rawData[2]= {0, 0};
  _i2c_bus->readBytes(LPS22DF_ADDRESS, LPS22DF_FIFO_STATUS1, 2, &rawData[0]);
  dest[0] = rawData[0];
  dest[1] = rawData[1];
}


void LPS22DF::FIFOReset()
{
 _i2c_bus->writeByte(LPS22DF_ADDRESS, LPS22DF_FIFO_CTRL,  0x00); // Disable water mark and enable BYPASS mode 
}


void LPS22DF::initFIFO(uint8_t fmode, uint8_t wtm)
{
     _i2c_bus->writeByte(LPS22DF_ADDRESS, LPS22DF_FIFO_WTM,  wtm); // define watermark
     _i2c_bus->writeByte(LPS22DF_ADDRESS, LPS22DF_FIFO_CTRL,  0x08 | fmode); // Enable water mark and enable FIFO mode 
}


int32_t LPS22DF::FIFOPressure()
{
    uint8_t rawData[3];  // 24-bit pressure register data stored here
    _i2c_bus->readBytes(LPS22DF_ADDRESS, LPS22DF_FIFO_DATA_OUT_PRESS_XL, 3, &rawData[0]);  
    return (int32_t) (((int32_t) rawData[2] << 24 | (int32_t) rawData[1] << 16 | rawData[0] << 8 | 0x00) >> 8);
}
