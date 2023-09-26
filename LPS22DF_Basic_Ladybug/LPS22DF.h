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

#ifndef LPS22DF_h
#define LPS22DF_h

#include "Arduino.h"
#include <Wire.h>
#include "I2Cdev.h"

// See LPS22DF "Low-power and high-precision MEMS nano pressure sensor: 260-1260 hPa
// absolute digital output barometer" Data Sheet
// https://www.st.com/resource/en/datasheet/lps22df.pdf
// https://www.st.com/resource/en/application_note/an5699-lps22df-lowpower-and-highprecision-mems-nano-pressure-sensor-stmicroelectronics.pdf
#define LPS22DF_INTERRUPT_CFG 0x0B
#define LPS22DF_THS_P_L       0x0C
#define LPS22DF_THS_P_H       0x0D
#define LPS22DF_IF_CTRL       0x0E
#define LPS22DF_WHOAMI        0x0F // should return 0xB4
#define LPS22DF_CTRL_REG1     0x10
#define LPS22DF_CTRL_REG2     0x11
#define LPS22DF_CTRL_REG3     0x12
#define LPS22DF_CTRL_REG4     0x13
#define LPS22DF_FIFO_CTRL     0x14
#define LPS22DF_FIFO_WTM      0x15
#define LPS22DF_REF_P_L       0x16
#define LPS22DF_REF_P_H       0x17
#define LPS22DF_IC3_IF_CTRL   0x19
#define LPS22DF_RPDS_L        0x1A
#define LPS22DF_RPDS_H        0x1B
#define LPS22DF_INT_SOURCE    0x24
#define LPS22DF_FIFO_STATUS1  0x25
#define LPS22DF_FIFO_STATUS2  0x26
#define LPS22DF_STATUS        0x27
#define LPS22DF_PRESS_OUT_XL  0x28
#define LPS22DF_PRESS_OUT_L   0x29
#define LPS22DF_PRESS_OUT_H   0x2A
#define LPS22DF_TEMP_OUT_L    0x2B
#define LPS22DF_TEMP_OUT_H    0x2C
#define LPS22DF_FIFO_DATA_OUT_PRESS_XL   0x78
#define LPS22DF_FIFO_DATA_OUT_PRESS_L    0x79
#define LPS22DF_FIFO_DATA_OUT_PRESS_H    0x7A

#define LPS22DF_ADDRESS 0x5C   // Address of altimeter

// Altimeter output data rate
#define    P_1shot  0x00 // power down/1-shot
#define    P_1Hz    0x01
#define    P_4Hz    0x02
#define    P_10Hz   0x03
#define    P_25Hz   0x04  // 25 Hz output data rate
#define    P_50Hz   0x05
#define    P_75Hz   0x06
#define    P_100Hz  0x07
#define    P_200Hz  0x08

// Pressure, temperature averaging
#define avg_4    0x00
#define avg_8    0x01
#define avg_16   0x02
#define avg_32   0x03
#define avg_64   0x04
#define avg_128  0x05
#define avg_512  0x07

#define lpf_odr4 0x00
#define lpf_odr9 0x01

// define FIFO modes
#define BYPASS     0x00
#define FIFOMODE   0x01
#define CONTINUOUS 0x02
// When trigger mode is enabled
#define BYPASS_TO_FIFO         0x01
#define BYPASS_TO_CONTINUOUS   0x02
#define CONTINUOUS_TO_FIFO     0x03


class LPS22DF
{
  public: 
  LPS22DF(I2Cdev* i2c_bus);
  void Init(uint8_t PODR, uint8_t AVG, uint8_t LPF);
  uint8_t getChipID();
  uint8_t status();
  void reset();
  void boot();
  void powerDown();
  void powerUp(uint8_t PODR);
  void oneShot();
  int32_t Pressure();
  int16_t Temperature();
  void initFIFO(uint8_t fmode, uint8_t wtm);
  void FIFOStatus(uint8_t * dest);
  void FIFOReset();
  int32_t FIFOPressure();
  private:
    I2Cdev* _i2c_bus;
};

#endif
