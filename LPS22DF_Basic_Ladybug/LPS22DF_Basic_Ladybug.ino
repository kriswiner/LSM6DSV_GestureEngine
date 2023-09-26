/* LPS22DF_Basic_Ladybug.ino
 *  
 *  Created by Kris WIner, 09/24/2023  
 *  
 *  Copyright 2023 Tlera Corporation.  All rights reserved.
 *  
 *  Sketch demonstrating the configuration and data output for ST's LPS22DF barometer
 * 
 *  Data configuration incldes data rate as well as low-pass
 *  filter and averaging selection. Data is output as absolute pressure in mBAR and temperature
 *  in degrees C as well as an estimate of altitude made from the pressure output.
 * 
 *  Sketch designed to run on an STM32L432KC (Ladybug) development board but should work on
 *  almost any MCU with a hardware I2C port.
 * 
 *  Sketch may be used by anyone without permission with proper attribution.
 */
 
#include "LPS22DF.h"
#include <RTC.h>

#define I2C_BUS          Wire                          // Define the I2C bus (Wire instance) you wish to use

I2Cdev                   i2c_0(&I2C_BUS);              // Instantiate the I2Cdev object and point to the desired I2C bus


#define SerialDebug false  // set to true to get Serial output for debugging
#define myLed 13
#define LPS22DF_intPin 8 // data ready/interrupt pin

const char        *build_date = __DATE__;   // 11 characters MMM DD YYYY
const char        *build_time = __TIME__;   // 8 characters HH:MM:SS

/* Specify sensor parameters (sample rate, averaging, mode) 
   Choices are:
       PODR = P_1Hz, P_4Hz, P_10Hz P_25 Hz, P_50Hz, P_75Hz, P_100Hz, and P_200Hz
       AVG = avg_4, avg_8, avg_16, avg_32, avg_64, avg_128, and avg_512
       LPF = lpf_odr/4, lpf_odr/9 (if EN_LPFP = 0 in REG2, LPF == ODR/2)
 */
// set pressure amd temperature output data rate, and data averaging 
uint8_t PODR = P_1Hz, AVG = avg_64, LPF = lpf_odr4;     
uint8_t LPS22DFstatus;
float Temperature, Pressure, altitude;
bool ONESHOT = false;
volatile bool LPS22DF_flag = false;

float LPS22DF_avgFIFOPressure = 0.0f;
const uint8_t fifoMode = FIFOMODE; // choices are BYPASS, FIFOMODE, CONTINUOUS
const uint8_t wtm = 10;  // set watermark, max 127
uint8_t fifoLevel = 0, fifoStatus = 0, FStatus[2] = {0, 0};
int32_t fifoSum = 0;

LPS22DF LPS22DF(&i2c_0);


// RTC set time using STM32L4 natve RTC class
uint8_t seconds, minutes, hours, day, month, year;
uint8_t Seconds, Minutes, Hours, Day, Month, Year;
volatile bool alarmFlag = false; // for RTC alarm interrupt


void setup() {
  Serial.begin(115200);
  Serial.blockOnOverrun(false);
  delay(4000);

  // Configure led
  pinMode(myLed, OUTPUT);
  digitalWrite(myLed, HIGH); // start with led on

  pinMode (LPS22DF_intPin, INPUT); // configure LPS22DF interrupt pin as input

  I2C_BUS.begin();          // set master mode 
  I2C_BUS.setClock(400000); // I2C frequency at 400 kHz  
  delay(100);
 
  i2c_0.I2Cscan();

  Serial.println("LPS22DF barometer...");
  uint8_t LPS22DF_ChipID = LPS22DF.getChipID();
  Serial.print("LPS22DF "); Serial.print("I AM "); Serial.print(LPS22DF_ChipID, HEX); Serial.print(" I should be "); Serial.println(0xB4, HEX);
  delay(1000); 
  

  if(LPS22DF_ChipID == 0xB4) // check if all I2C sensors have acknowledged
  {
   Serial.println("LPS22DF is online..."); Serial.println(" ");
   
   digitalWrite(myLed, LOW);

   LPS22DF.reset();
   delay(1);
   LPS22DF.Init(PODR, AVG, LPF);    // Initialize LPS22DF altimeter
   LPS22DF.initFIFO(fifoMode, wtm); // configure LPS22DF FIFO, works with ONESHOT mode too
   LPS22DF.powerDown();

   digitalWrite(myLed, LOW); // Signal sensor initialization complete
   
  }
  else 
  {
  if(LPS22DF_ChipID != 0xB4) Serial.println(" LPS22DF not functioning!");
  while(1){};
  }

  // Set the time
  SetDefaultRTC();
  
  /* Set up the RTC alarm interrupt */
  RTC.enableAlarm(RTC.MATCH_ANY);  // alarm once a second 
//  RTC.enableAlarm(RTC.MATCH_SS);  // alarm once a minute 
  RTC.attachInterrupt(alarmMatch); // interrupt every time the alarm sounds

  attachInterrupt(LPS22DF_intPin, LPS22DF_intHandler, RISING);  // define interrupt for intPin1 output of LPS22DF
 
  if(ONESHOT) {
    LPS22DF.oneShot();    // run in one shot mode
  }
  else {
    LPS22DF.powerUp(PODR); // or run in continuous mode at preset ODR
  }
}

/* End of setup */

void loop() {

  if(LPS22DF_flag) {
    LPS22DF_flag = false;

    LPS22DFstatus = LPS22DF.status();
    if(LPS22DFstatus & 0x10) {Serial.println("Pressure Overrun!");}
    if(LPS22DFstatus & 0x20) {Serial.println("Temperature Overrun!");}

    if(LPS22DFstatus & 0x01) { // if new pressure data available
    // get pressure  from the LPS22DF
    Pressure = (float) LPS22DF.Pressure()/4096.0f;
    }

    if(LPS22DFstatus & 0x02) { // if new temperature data available
    // get temperature  from the LPS22DF
    Temperature = (float) LPS22DF.Temperature()/100.0f; 
    }
    
    altitude = 145366.45f*(1.0f - powf((Pressure/1013.25f), 0.190284f)); 

    if(SerialDebug) {
      Serial.print("Altimeter temperature = "); Serial.print( Temperature, 2); Serial.println(" C"); // temperature in degrees Celsius  
      Serial.print("Altimeter temperature = "); Serial.print(9.0f*Temperature/5.0f + 32.0f, 2); Serial.println(" F"); // temperature in degrees Fahrenheit
      Serial.print("Altimeter pressure = "); Serial.print(Pressure, 2);  Serial.println(" mbar");// pressure in millibar
      Serial.print("Altitude = "); Serial.print(altitude, 2); Serial.println(" feet");
    }

     //For data plotting
     Serial.print(millis()/1000); Serial.print(",");
     Serial.print(Pressure, 2); Serial.print(","); Serial.print(Temperature, 2); Serial.print(","); Serial.println(altitude, 2);  
   
      // Check for FIFO interrupts
      LPS22DF.FIFOStatus(FStatus); // read FIFO status registers
      fifoLevel  = FStatus[0];
      fifoStatus = FStatus[1];
//      Serial.print("FIFO Status = "); Serial.println(fifoStatus);
//      Serial.print("FIFO Level = "); Serial.println(fifoLevel);
      if(fifoStatus & 0x40 && SerialDebug) Serial.println("FIFO is full and at least one sample in the FIFO has been overwritten!"); // Simple error handling
      if(fifoStatus & 0x20 && SerialDebug) Serial.println("FIFO is completely filled, no samples overwritten!");  
      if(fifoStatus & 0x80) {
          fifoSum = 0;
          for (uint8_t ii = 0; ii < fifoLevel; ii++) {
          int32_t temp = LPS22DF.FIFOPressure();
          if(ii > 0) fifoSum +=  temp; // discard first value if LPF is ODR/4, first two when LPF = ODR/9
          }
          if(fifoLevel >  1) LPS22DF_avgFIFOPressure = ((float) fifoSum / (float) (fifoLevel - 1)) /4096.0f;
          if (SerialDebug) {
            Serial.print("Average LPS22DF FIFO Pressure (hPa) over "); Serial.print(fifoLevel - 1); Serial.print(" samples = "); Serial.println(LPS22DF_avgFIFOPressure, 2);
            altitude = 145366.45f*(1.0f - powf((LPS22DF_avgFIFOPressure/1013.25f), 0.190284f)); 
            Serial.print("Average altitude = "); Serial.print(altitude, 2); Serial.println(" feet");
          }
          
          LPS22DF.FIFOReset();               // reset LPS22DF FIFO
          LPS22DF.initFIFO(fifoMode, wtm);   // restart LPS22DF FIFO
      }
      
  }  // end of LPS22DF interrupt handling


   if(alarmFlag) { // update RTC output (serial display) whenever the RTC alarm condition is achieved and the MPU9250 is awake
      alarmFlag = false;
      
   if(ONESHOT) LPS22DF.oneShot(); // run LPS22DF in one shot mode at rate of RTC alarm

    // Read RTC
   if(SerialDebug)
    {
    Serial.println("RTC:");
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
        
    digitalWrite(myLed, HIGH); delay(1); digitalWrite(myLed, LOW);   

 } // end of RTC Alarm handling

}  /*  End of main loop */


// Some useful functions
void alarmMatch()
{
  alarmFlag = true;
}


void SetDefaultRTC()  // Sets the RTC to the FW build date-time...Courtesy Greg Tomasch
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


 void LPS22DF_intHandler()
{
  LPS22DF_flag = true;
}
