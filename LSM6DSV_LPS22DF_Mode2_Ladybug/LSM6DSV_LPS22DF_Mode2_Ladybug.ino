/*
 * Copyright (c) 2023 Tlera Corp.  All rights reserved.
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
 * Basic example of using LSM6DSV accel/gyro combo IMU  
 * 
 * Library may be used freely and without limit with attribution.
 */
#include "LSM6DSV.h"
#include "LPS22DF.h"
#include <RTC.h>
#include "I2Cdev.h"

#define I2C_BUS          Wire                           // Define the I2C bus (Wire instance) you wish to use

I2Cdev                   i2c_0(&I2C_BUS);               // Instantiate the I2Cdev object and point to the desired I2C bus

#define SerialDebug true  // set to true to get Serial output for debugging
#define myLed    13

const char        *build_date = __DATE__;   // 11 characters MMM DD YYYY
const char        *build_time = __TIME__;   // 8 characters HH:MM:SS

volatile bool sleepFlag = true;

//LSM6DSV definitions
#define LSM6DSV_intPin1 8  // interrupt pin  

/* Specify sensor parameters (sample rate is twice the bandwidth)
 * choices are:
      AFS_2G, AFS_4G, AFS_8G, AFS_16G  
      GFS_125DPS, GFS_250DPS, GFS_500DPS, GFS_1000DPS, GFS_2000DPS, GFS_4000DPS 
      AODR_1_875Hz, AODR_7_5Hz, AODR_15Hz, AODR_30Hz, AODR_60Hz, AODR_120Hz, AODR_240Hz, AODR_480Hz, AODR_960Hz, AODR_1920Hz, AODR_3840Hz, AODR_7680Hz
      GODR_7_5Hz, GODR_15Hz, GODR_30Hz, GODR_60Hz, GODR_120Hz, GODR_240Hz, GODR_480Hz, GODR_960Hz, GODR_1920Hz, GODR_3840Hz, GODR_7680Hz
      A_PWRMD_HiPerf (default), A_PWRMD_HiAccu, A_PWRMD_ODR_Trig, A_PWRMD_LPWR1, A_PWRMD_LPWR2, A_PWRMD_LPWR3, A_PWRMD_Normal
      G_PWRMD_HiPerf (default), G_PWRMD_HiAccu, G_PWRMD_ODR_Trig, G_PWRMD_Sleep, G_PWRMD_LPWR
*/ 
uint8_t Ascale = AFS_2G, Gscale = GFS_250DPS, AODR = AODR_30Hz, GODR = GODR_30Hz, A_PwrMode = A_PWRMD_HiPerf, G_PwrMode = G_PWRMD_HiPerf;
uint8_t sflpODR = SFLP_ODR_15; // choices: SLFP_ODR_15, SLFP_ODR_30, SLFP_ODR_60, SFLP_ODR_120, SFLP_ODR_240 and SFLP_ODR_480, must be less than AODR
float aRes, gRes, gravRes = 2.0f/32768.0f; // scale resolutions per LSB for the accel and gyro sensors
float accelBias[3] = {0.0f, 0.0f, 0.0f}, gyroBias[3] = {0.0f, 0.0f, 0.0f}; // offset biases for the accel and gyro
int16_t LSM6DSVData[7];         // Stores the 16-bit signed sensor output
float   Gtemperature;           // Stores the real internal gyro temperature in degrees Celsius
float ax, ay, az, gx, gy, gz;   // variables to hold latest accel/gyro data values 
float a12, a22, a31, a32, a33;  // rotation matrix coefficients for Euler angles and gravity components
uint8_t LSM6DSVstatus;
volatile bool LSM6DSVint = false, LSM6DSVasleep = false;

// sleepMode choices are: SLPMODE0 (stationary), SLPMODE1 (accel in LP1), SLPMODE2 (accel in LP1 and gyro in sleep), 
// and SLPMODE3 (accel in LP1 and gyro powerdown)
// inactODR choices are: inactODR_1_875Hz, inactODR_15Hz (default), inactODR_30Hz, inactODR_60Hz
// inactive time to sleep is inactiveTime x 512/AODR. At 30 Hz, 1 x 512/30 ~ 17 seconds 
// inactive time to sleep is inactiveTime x 16/AODR when inactiveTime == 0, ~8 sec at AODR_1_875Hz
uint8_t inactiveTime = 0, inactAODR = inactAODR_1_875Hz, sleepMode = SLPMODE3; // select sleep behavior

const uint8_t fifoMode = FIFOMODE; // choices are BYPASS, FIFOMODE, CONTINUOUS, et al
const uint8_t wtm = 64;  // set watermark, max 255
// Choices are: FIFO_NOBATCH, FIFO_1_875Hz, FIFO_7_5Hz, FIFO_15Hz, FIFO_30Hz, FIFO_60Hz, FIFO_120Hz,
//              FIFO_240Hz, FIFO_480Hz, FIFO_960Hz, FIFO_1920Hz, FIFO_3840Hz, FIFO_7680Hz but must be less than AODR/GODR
uint8_t fifo_AODR = FIFOODR_15Hz, fifo_GODR = FIFOODR_15Hz; // must be less than AODR/GODR
uint16_t fifoLevel = 0; 
uint8_t fifoStatus = 0, FStatus[2] = {0, 0}, tagID, tagCount;
uint8_t fifoOut[7];                           // raw FIFO data bytes
int16_t fifoData[3] = {0, 0, 0};              // Stores the 16-bit signed sensor output from FIFO
float fax, fay, faz, fgx, fgy, fgz, gravx, gravy, gravz; // variables to hold accel/gyro and gravity data values from FIFO
uint16_t fifoHF[3] = {0, 0, 0};               // array for the half precision fifo data
float sflpq[4] = {1.0f, 0.0f, 0.0f, 0.0f};    // vector to hold sflp quaternion
float SFLPpitch, SFLPyaw, SFLProll;           // absolute SLFP orientation
float pi = 3.14159265f;

LSM6DSV LSM6DSV(&i2c_0); // instantiate LSM6DSV class


/* Specify sensor parameters (sample rate, averaging, mode) 
   Choices are:
       PODR = P_1Hz, P_4Hz, P_10Hz P_25 Hz, P_50Hz, P_75Hz, P_100Hz, and P_200Hz
       AVG = avg_4, avg_8, avg_16, avg_32, avg_64, avg_128, and avg_512
       LPF = lpf_odr/4, lpf_odr/9 (if EN_LPFP = 0 in REG2, LPF == ODR/2)
 */
// set pressure amd temperature output data rate, and data averaging 
uint8_t PODR = P_4Hz, AVG = avg_64, LPF = lpf_odr4;     
uint8_t LPS22DFstatus;
float Temperature, Pressure, altitude;
bool ONESHOT = false;
volatile bool LPS22DF_flag = false;

LPS22DF LPS22DF(&i2c_0);


// RTC set time using STM32L4 native RTC class
uint8_t seconds = 0, minutes = 0, hours = 0, day = 1, month = 1, year = 23;
uint8_t Seconds, Minutes, Hours, Day, Month, Year;
volatile bool alarmFlag = false; // for RTC alarm interrupt


void setup() {
  Serial.begin(115200);
  Serial.blockOnOverrun(false);
  delay(4000);

  // Configure led
  pinMode(myLed, OUTPUT);
  digitalWrite(myLed, LOW); // start with led off

  pinMode(LSM6DSV_intPin1, INPUT); // enable LSM6DSV interrupt

  Wire.begin();               // designate I2C pins for MCU master I2C bus
  Wire.setClock(400000);      // I2C frequency at 400 kHz  
  delay(1000);
 
  i2c_0.I2Cscan();           // scan for slave I2C devices on master MCU I2C bus

  LSM6DSV.passthruMode();    // allow direct access to slave sensors on LSM6DSV master I2C bus

  i2c_0.I2Cscan();           // scan for slave I2C devices on master MCU I2C bus, should see barometer now

  // Read the LSM6DSV Chip ID register, this is a good test of communication
  Serial.println("LSM6DSV accel/gyro...");
  byte LSM6DSV_ChipID = LSM6DSV.getChipID();  // Read CHIP_ID register for LSM6DSV
  Serial.print("LSM6DSV "); Serial.print("I AM "); Serial.print(LSM6DSV_ChipID, HEX); Serial.print(" I should be "); Serial.println(0x70, HEX);
  Serial.println(" ");

  Serial.println("LPS22DF barometer...");
  uint8_t LPS22DF_ChipID = LPS22DF.getChipID();
  Serial.print("LPS22DF "); Serial.print("I AM "); Serial.print(LPS22DF_ChipID, HEX); Serial.print(" I should be "); Serial.println(0xB4, HEX);
  delay(1000); 

  if(LSM6DSV_ChipID == 0x70 && LPS22DF_ChipID == 0xB4) // check if all I2C sensors have acknowledged
  {
   Serial.println("LSM6DSV and LPS22DF are online..."); Serial.println(" ");
      
   digitalWrite(myLed, HIGH);

   // get accel/gyro sensor resolutions, only need to do this once
   aRes = LSM6DSV.getAres(Ascale);
   gRes = LSM6DSV.getGres(Gscale);

   // reset LSM6DSV to start fresh
   LSM6DSV.reset();
   LSM6DSV.init(Ascale, Gscale, AODR, GODR, A_PwrMode, G_PwrMode); // configure LSM6DSV  
   LSM6DSV.selfTest();
   LSM6DSV.AGoffsetBias(gyroBias, accelBias);
   Serial.println("accel biases (mg)"); Serial.println(1000.0f * accelBias[0]); Serial.println(1000.0f * accelBias[1]); Serial.println(1000.0f * accelBias[2]);
   Serial.println(" ");
   Serial.println("gyro biases (dps)"); Serial.println(gyroBias[0]); Serial.println(gyroBias[1]); Serial.println(gyroBias[2]);
   Serial.println(" ");
   delay(1000); 

   LPS22DF.reset();
   delay(1);
   LPS22DF.Init(PODR, AVG, LPF);    // Initialize LPS22DF altimeter

   LPS22DF.powerDown();
   LSM6DSV.accelPowerDown(); // power down accel & gyro to configure embedded functions
   LSM6DSV.gyroPowerDown();

   // enable embedded activity detection when sensors are powered down
   LSM6DSV.TiltDetect();                          // allow detection of tilting by 35 degrees from starting position 
   inactiveTime = 1;                              // at 30 Hz inactive time to sleep is 1 x 512/30 ~ 17 seconds 
   LSM6DSV.ActivityDetect(inactiveTime, inactAODR, sleepMode); // allow sleep/wake detection, sleep after (inactiveTime x 512/AODR) seconds of inactivity  
   LSM6DSV.SFLP(sflpODR);                         // enable sensor fusion low power function 
  
   // enable Finite State Machine programs 
   /* Wrist tilt  detect on FSM1
   *  Shake       detect on FSM2
   *  Motion      detect on FSM3
   *  Glance      detect on FSM4
   */ 
   LSM6DSV.FSMprograms();   // start the finite-state machine programs

   LSM6DSV.accelPowerUp(AODR); // accel odr set in FSM functions
   LSM6DSV.gyroPowerUp(GODR);  // power up gyro
   LPS22DF.powerUp(PODR);      // power up baro

   LSM6DSV.masterMode();  // Let LSM6DSV manage the slave LPS22DF barometer

  digitalWrite(myLed, LOW); // turn off led when sensor configuration is finished
  }
  else 
  {
  if(LSM6DSV_ChipID != 0x70) Serial.println(" LSM6DSV not functioning!");
  if(LPS22DF_ChipID != 0xB4) Serial.println(" LPS22DF not functioning!");
  while(1){};
  }

 // Set the time
  SetDefaultRTC();
  
  /* Set up the RTC alarm interrupt */
  RTC.enableAlarm(RTC.MATCH_ANY); // alarm once a second
  
  RTC.attachInterrupt(alarmMatch); // interrupt every time the alarm sounds

  attachInterrupt(LSM6DSV_intPin1, myinthandler1, RISING);  // define data ready interrupt for intPin1  

  LSM6DSV.initFIFO(fifoMode, wtm, fifo_AODR, fifo_GODR);

// Clear interrupts before entering main loop
  LSM6DSVstatus = LSM6DSV.DRstatus();  // read data ready status
  LSM6DSVstatus = LSM6DSV.ACTstatus(); // read significant motion status
  LSM6DSVstatus = LSM6DSV.EMBstatus(); // read embedded function status
  LSM6DSVstatus = LSM6DSV.FSMstatus(); // read FSM status
}
/* End of setup */

void loop() {

   // when intPin1 goes high, check status registers to determine which event was triggered
   if(LSM6DSVint == true) {   // On interrupt, handle all INT sources
      LSM6DSVint = false;     // reset INT flag
      
 
      // Handle FSM interrupts
      LSM6DSVstatus = LSM6DSV.FSMstatus();
      if(LSM6DSVstatus & 0x01)
      {Serial.println(" "); Serial.print("Wrist tilt detected!"); Serial.println(" "); }  // FSM 1
      if(LSM6DSVstatus & 0x02)
      {Serial.println(" "); Serial.print("Shake detected!"); Serial.println(" "); }       // FSM 2
      if(LSM6DSVstatus & 0x03)
      {Serial.println(" "); Serial.print("Motion detected!"); Serial.println(" "); }      // FSM 3
      if(LSM6DSVstatus & 0x04)
      {Serial.println(" "); Serial.print("Glance detected!"); Serial.println(" "); }      // FSM 4
      

      // Check for FIFO watermark full
      LSM6DSV.FIFOStatus(FStatus); // read FIFO status registers
      fifoLevel  = (FStatus[1] & 0x01) << 8 | FStatus[0]; // number of samples (up to 256 sets of tag + six bytes) stored in FIFO
      fifoStatus = FStatus[1];
      if(fifoStatus & 0x40 && SerialDebug) {Serial.println(" "); Serial.println("FIFO is full and at least one sample in the FIFO has been overwritten!"); } // Simple error handling
      if(fifoStatus & 0x20) { // watermark reached on next ODR
        
//          for (uint8_t ii = 0; ii < fifoLevel; ii++) {
          for (uint8_t ii = 0; ii < 5; ii++) {  // process first 5 only for easier viewing on Serial Monitor
          LSM6DSV.FIFOOutput(fifoOut);
          tagID = (fifoOut[0] & 0xF8) >> 3;
          tagCount = (fifoOut[0] & 0x06) >> 1;
         
          fifoData[0] = (int16_t) fifoOut[2] << 8 | fifoOut[1];
          fifoData[1] = (int16_t) fifoOut[4] << 8 | fifoOut[3];
          fifoData[2] = (int16_t) fifoOut[6] << 8 | fifoOut[5];

          if(tagID == 0x01) { // uncompressed gyro data
                     fgx = (float)fifoData[0]*gRes - gyroBias[0];
                     fgy = (float)fifoData[1]*gRes - gyroBias[1];
                     fgz = (float)fifoData[2]*gRes - gyroBias[2];
                     if(SerialDebug) {
                       Serial.print(tagCount); Serial.print(",");
                       Serial.print("fgx = ");  Serial.print(fgx, 2); 
                       Serial.print(" fgy = "); Serial.print(fgy, 2); 
                       Serial.print(" fgz = "); Serial.print(fgz, 2); Serial.println(" deg/s"); }
                     }
          if(tagID == 0x02) { // uncompressed accel data
                     fax = (float)fifoData[0]*aRes - accelBias[0];
                     fay = (float)fifoData[1]*aRes - accelBias[1];
                     faz = (float)fifoData[2]*aRes - accelBias[2];
                     if(SerialDebug) {
                       Serial.print(tagCount);  Serial.print(",");
                       Serial.print("fax = ");  Serial.print((int)1000*fax);  
                       Serial.print(" fay = "); Serial.print((int)1000*fay); 
                       Serial.print(" faz = "); Serial.print((int)1000*faz); Serial.println(" mg"); }
                     }
          if(tagID == 0x13) { // uncompressed sflp quaternion data
                     fifoHF[0] = (uint16_t) fifoOut[2] << 8 | fifoOut[1]; // construct half-float quaternions
                     fifoHF[1] = (uint16_t) fifoOut[4] << 8 | fifoOut[3];
                     fifoHF[2] = (uint16_t) fifoOut[6] << 8 | fifoOut[5];
 
                     LSM6DSV.convertQ(fifoHF, sflpq); // convert half-float to float quaternions
                    
                    // construct rotation matrix                  
                     a12 =   2.0f * (sflpq[1] * sflpq[2] + sflpq[0] * sflpq[3]);
                     a22 =   sflpq[0] * sflpq[0] + sflpq[1] * sflpq[1] - sflpq[2] * sflpq[2] - sflpq[3] * sflpq[3];
                     a31 =   2.0f * (sflpq[0] * sflpq[1] + sflpq[2] * sflpq[3]);
                     a32 =   2.0f * (sflpq[1] * sflpq[3] - sflpq[0] * sflpq[2]);
                     a33 =   sflpq[0] * sflpq[0] - sflpq[1] * sflpq[1] - sflpq[2] * sflpq[2] + sflpq[3] * sflpq[3];

                     // derive YPR from rotation matrix
                     SFLPpitch = -asinf(a32);
                     SFLProll  = atan2f(a31, a33);
                     SFLPyaw   = atan2f(a12, a22);
                     SFLPpitch *= 180.0f / pi;
                     SFLPyaw   *= 180.0f / pi; 
                     if(SFLPyaw < 0) SFLPyaw  += 360.0f; // Ensure SFLP yaw stays between 0 and 360
                     SFLProll  *= 180.0f / pi;
    
                     if(SerialDebug) {
                      Serial.print(tagCount);  Serial.print(",");
                       Serial.print("sflp q0 = ");  Serial.print(sflpq[0], 2);
                       Serial.print(" sflp qx = "); Serial.print(sflpq[1], 2); 
                       Serial.print(" sflp qy = "); Serial.print(sflpq[2], 2); 
                       Serial.print(" sflp qz = "); Serial.println(sflpq[3], 2);                       
                       Serial.print("SFLP Yaw, Pitch, Roll: ");
                       Serial.print(SFLPyaw, 2);
                       Serial.print(", ");
                       Serial.print(SFLPpitch, 2);
                       Serial.print(", ");
                       Serial.println(SFLProll, 2);}
                     }
          if(tagID == 0x17) { // uncompressed gravity data
                     gravx = (float)fifoData[0]*gravRes;
                     gravy = (float)fifoData[1]*gravRes;
                     gravz = (float)fifoData[2]*gravRes;
                     if(SerialDebug) {
                       Serial.print(tagCount);  Serial.print(",");
                       Serial.print("gravx = ");  Serial.print((int)1000*gravx);  
                       Serial.print(" gravy = "); Serial.print((int)1000*gravy); 
                       Serial.print(" gravz = "); Serial.print((int)1000*gravz); Serial.println(" mg"); }
                     } 
          if(tagID == 0x0E) {  // check if barometer (SLV0) data in FIFO
                     Pressure = (float) ( (int32_t) ((int32_t) fifoOut[3] << 16 | (int32_t) fifoOut[2] << 8 | fifoOut[1]) );
                     Temperature = (float) ( (int16_t)((int16_t) fifoOut[5] << 8 | fifoOut[4]) );
                     Pressure /=  4096.0f;
                     Temperature /=  100.0f; 
                     altitude = 145366.45f*(1.0f - pow((Pressure/1013.25f), 0.190284f));     
                     if(SerialDebug) {
                       Serial.print(tagCount); Serial.print(",");
                       Serial.print("Alt T = "); Serial.print( Temperature, 2); Serial.print(" C");                    // temperature in degrees Celsius  
                       Serial.print(", Alt T = "); Serial.print(9.0f*Temperature/5.0f + 32.0f, 2); Serial.print(" F"); // temperature in degrees Fahrenheit
                       Serial.print(", Alt P = "); Serial.print(Pressure, 2);  Serial.print(" mbar");                     // pressure in millibar
                       Serial.print(", Alt = "); Serial.print(altitude, 2); Serial.println(" feet");}
                     }              
        }
          LSM6DSV.FIFOReset();                                                                // reset LSM6DSV FIFO
          LSM6DSV.initFIFO(fifoMode, wtm, fifo_AODR, fifo_GODR);                              // restart LSM6DSV FIFO
    }


     // Handle embedded function interrupts
     // Tilt detection
     LSM6DSVstatus = LSM6DSV.EMBstatus();
     if(LSM6DSVstatus & 0x10)
     {Serial.println(" "); Serial.println(" Tilt event detected!"); Serial.println(" ");}


     // Handle significant event interrupts
     LSM6DSVstatus = LSM6DSV.ACTstatus(); // read significant motion status

     if(LSM6DSVstatus & 0x20) { // check for sleep change event
      if(LSM6DSV.wakeSource() & 0x10) {
        Serial.println("LSM6DSV is inactive!");  Serial.println(" ");
        LSM6DSVasleep = true;

        LSM6DSV.FIFOReset();                                     // reset LSM6DSV FIFO
        LSM6DSV.idleLPS22DF();                                   // put baro to sleep
      }
      else {
        Serial.println("LSM6DSV is active!");  Serial.println(" ");
        LSM6DSVasleep = false;

        LSM6DSV.initFIFO(fifoMode, wtm, fifo_AODR, fifo_GODR);   // restart LSM6DSV FIFO
        LSM6DSV.resumeLPS22DF(PODR, AVG);                        // resume baro operation
      }
   }

   

}  // end INT1 interrupt handling


    if(alarmFlag) { // update RTC output (serial display) whenever the RTC alarm condition is achieved 
       alarmFlag = false;

    // Read RTC
   if(SerialDebug)
    {
    Serial.println(" "); Serial.println("RTC:");
    Day = RTC.getDay();
    Month = RTC.getMonth();
    Year = RTC.getYear();
    Seconds = RTC.getSeconds();
    Minutes = RTC.getMinutes();
    Hours   = RTC.getHours();     
    if(Hours < 10) {Serial.print("0"); Serial.print(Hours);} else Serial.print(Hours);
    Serial.print(":"); 
    if(Minutes < 10) {Serial.print("0"); Serial.print(Minutes);} else Serial.print(Minutes); 
    Serial.print(":"); 
    if(Seconds < 10) {Serial.print("0"); Serial.println(Seconds);} else Serial.println(Seconds);  

    Serial.print(Month); Serial.print("/"); Serial.print(Day); Serial.print("/"); Serial.println(Year);
    Serial.println(" ");
    }

    /* either poll at RTC alarm rate or use FIFO and external sensor interrupt for rate
    Pressure = (float)LSM6DSV.readBaroData()/4096.0f;
    Temperature = (float)LSM6DSV.readBaroTemp()/100.0f; 
    altitude = 145366.45f*(1.0f - pow((Pressure/1013.25f), 0.190284f)); 

    if(SerialDebug) {
    Serial.print("Altimeter temperature = "); Serial.print( Temperature, 2); Serial.println(" C");                  // temperature in degrees Celsius  
    Serial.print("Altimeter temperature = "); Serial.print(9.0f*Temperature/5.0f + 32.0f, 2); Serial.println(" F"); // temperature in degrees Fahrenheit
    Serial.print("Altimeter pressure = "); Serial.print(Pressure, 2);  Serial.println(" mbar");                     // pressure in millibar
    Serial.print("Altitude = "); Serial.print(altitude, 2); Serial.println(" feet");
    }
    */
    Gtemperature = ((float) LSM6DSVData[0]) / 256.0f + 25.0f; // Gyro chip temperature in degrees Centigrade
    // Print temperature in degrees Centigrade      
    if(SerialDebug && LSM6DSVasleep == false) {
      Serial.print("Gyro temperature is "); Serial.print(Gtemperature, 1); Serial.println(" degrees C"); Serial.println(" "); // Print T values to tenths of s degree C
    }

    // For plotting comma-delimited Euler angles in a spreadsheet
//     Serial.print(millis()/1000); Serial.print(",");
//     Serial.print(yaw, 2); Serial.print(","); Serial.print(pitch, 2); Serial.print(","); Serial.print(roll, 2); Serial.print(","); 
//     Serial.print(SFLPyaw, 2); Serial.print(","); Serial.print(SFLPpitch, 2); Serial.print(","); Serial.println(SFLProll, 2); 
     
    digitalWrite(myLed, HIGH); delay(1); digitalWrite(myLed, LOW);   
    } // end of RTC Alarm section

//    STM32.stop(); // sleep while waiting for an interrupt

} /*  End of main loop */


void myinthandler1()
{
  LSM6DSVint = true;
}


void alarmMatch()
{
  alarmFlag = true;
}


void SetDefaultRTC()  // Sets the RTC to the FW build date-time...
{
  char Build_mo[3];

  // Convert month string to integer

  Build_mo[0] = build_date[0];
  Build_mo[1] = build_date[1];
  Build_mo[2] = build_date[2];

  String build_mo = Build_mo;

  if(build_mo == "Jan")
  {
    month = 1;
  } else if(build_mo == "Feb")
  {
    month = 2;
  } else if(build_mo == "Mar")
  {
    month = 3;
  } else if(build_mo == "Apr")
  {
    month = 4;
  } else if(build_mo == "May")
  {
    month = 5;
  } else if(build_mo == "Jun")
  {
    month = 6;
  } else if(build_mo == "Jul")
  {
    month = 7;
  } else if(build_mo == "Aug")
  {
    month = 8;
  } else if(build_mo == "Sep")
  {
    month = 9;
  } else if(build_mo == "Oct")
  {
    month = 10;
  } else if(build_mo == "Nov")
  {
    month = 11;
  } else if(build_mo == "Dec")
  {
    month = 12;
  } else
  {
    month = 1;     // Default to January if something goes wrong...
  }

  // Convert ASCII strings to integers
  day     = (build_date[4] - 48)*10 + build_date[5] - 48;  // ASCII "0" = 48
  year    = (build_date[9] - 48)*10 + build_date[10] - 48;
  hours   = (build_time[0] - 48)*10 + build_time[1] - 48;
  minutes = (build_time[3] - 48)*10 + build_time[4] - 48;
  seconds = (build_time[6] - 48)*10 + build_time[7] - 48;

  // Set the date/time

  RTC.setDay(day);
  RTC.setMonth(month);
  RTC.setYear(year);
  RTC.setHours(hours);
  RTC.setMinutes(minutes);
  RTC.setSeconds(seconds);
}
  
