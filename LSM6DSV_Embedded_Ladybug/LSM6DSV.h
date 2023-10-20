/* 09/24/2023 Copyright Tlera Corporation

    Created by Kris Winer

  This sketch uses SDA/SCL on pins 21/20 (Ladybugy default), respectively, and it uses the Ladybug (STM32L432KC) Breakout Board.

  https://www.tindie.com/products/tleracorp/ladybug-stm32l432-development-board/?pt=ac_prod_search
  
  The LSM6DSV is a combo accel and gyro, withsensor hub and and a finite state machine, with a lot of
  embedded functionality including a 6 DoF sensor fusion (gaming) engine.

  Library may be used freely and without limit with attribution.

*/

#ifndef LSM6DSV_h
#define LSM6DSV_h

#include "Arduino.h"
#include <Wire.h>
#include "I2Cdev.h"

/* LSM6DSV registers
 *  Datasheet:
 *  https://www.st.com/resource/en/datasheet/lsm6dsv.pdf
 *  Application note:
 *  https://www.st.com/resource/en/application_note/an5922-lsm6dsv-6axis-imu-with-embedded-sensor-fusion-i3c-oiseis-for-smart-applications-stmicroelectronics.pdf
*/
#define LSM6DSV_FUNC_CFG_ACCESS           0x01
#define LSM6DSV_PIN_CTRL                  0x02
#define LSM6DSV_IF_CFG                    0x03

#define LSM6DSV_ODR_TRIG_CFG              0x06
#define LSM6DSV_FIFO_CTRL1                0x07
#define LSM6DSV_FIFO_CTRL2                0x08
#define LSM6DSV_FIFO_CTRL3                0x09
#define LSM6DSV_FIFO_CTRL4                0x0A
#define LSM6DSV_COUNTER_BDR_REG1          0x0B
#define LSM6DSV_COUNTER_BDR_REG2          0x0C
#define LSM6DSV_INT1_CTRL                 0x0D
#define LSM6DSV_INT2_CTRL                 0x0E
#define LSM6DSV_WHO_AM_I                  0x0F  // should be 0x70
#define LSM6DSV_CTRL1                     0x10
#define LSM6DSV_CTRL2                     0x11
#define LSM6DSV_CTRL3                     0x12
#define LSM6DSV_CTRL4                     0x13
#define LSM6DSV_CTRL5                     0x14
#define LSM6DSV_CTRL6                     0x15
#define LSM6DSV_CTRL7                     0x16
#define LSM6DSV_CTRL8                     0x17
#define LSM6DSV_CTRL9                     0x18
#define LSM6DSV_CTRL10                    0x19
#define LSM6DSV_CTRL_STATUS               0x1A
#define LSM6DSV_FIFO_STATUS1              0x1B
#define LSM6DSV_FIFO_STATUS2              0x1C
#define LSM6DSV_ALL_INT_SRC               0x1D
#define LSM6DSV_STATUS_REG                0x1E

#define LSM6DSV_OUT_TEMP_L                0x20
#define LSM6DSV_OUT_TEMP_H                0x21
#define LSM6DSV_OUTX_L_G                  0x22
#define LSM6DSV_OUTX_H_G                  0x23
#define LSM6DSV_OUTY_L_G                  0x24
#define LSM6DSV_OUTY_H_G                  0x25
#define LSM6DSV_OUTZ_L_G                  0x26
#define LSM6DSV_OUTZ_H_G                  0x27
#define LSM6DSV_OUTX_L_A                  0x28
#define LSM6DSV_OUTX_H_A                  0x29
#define LSM6DSV_OUTY_L_A                  0x2A
#define LSM6DSV_OUTY_H_A                  0x2B
#define LSM6DSV_OUTZ_L_A                  0x2C
#define LSM6DSV_OUTZ_H_A                  0x2D
#define LSM6DSV_UI_OUTX_L_G_OIS_EIS       0x2E
#define LSM6DSV_UI_OUTX_H_G_OIS_EIS       0x2F
#define LSM6DSV_UI_OUTY_L_G_OIS_EIS       0x30
#define LSM6DSV_UI_OUTY_H_G_OIS_EIS       0x31
#define LSM6DSV_UI_OUTZ_L_G_OIS_EIS       0x32
#define LSM6DSV_UI_OUTZ_H_G_OIS_EIS       0x33
#define LSM6DSV_UI_OUTX_L_A_OIS_DUALC     0x34
#define LSM6DSV_UI_OUTX_H_A_OIS_DUALC     0x35
#define LSM6DSV_UI_OUTY_L_A_OIS_DUALC     0x36
#define LSM6DSV_UI_OUTY_H_A_OIS_DUALC     0x37
#define LSM6DSV_UI_OUTZ_L_A_OIS_DUALC     0x38
#define LSM6DSV_UI_OUTZ_H_A_OIS_DUALC     0x39

#define LSM6DSV_TIMESTAMP0                0x40
#define LSM6DSV_TIMESTAMP1                0x41
#define LSM6DSV_TIMESTAMP2                0x42
#define LSM6DSV_TIMESTAMP3                0x43
#define LSM6DSV_UI_STATUS_REG_OIS         0x44
#define LSM6DSV_WAKE_UP_SRC               0x45
#define LSM6DSV_TAP_SRC                   0x46
#define LSM6DSV_D6D_SRC                   0x47
#define LSM6DSV_STATUS_MASTER_MAINPAGE    0x48
#define LSM6DSV_EMB_FUNC_STATUS_MAINPAGE  0x49
#define LSM6DSV_FSM_STATUS_MAINPAGE       0x4A

#define LSM6DSV_INTERNAL_FREQ_FINE        0x4F
#define LSM6DSV_FUNCTIONS_ENABLE          0x50
#define LSM6DSV_DEN                       0x51

#define LSM6DSV_INACTIVITY_DUR            0x54
#define LSM6DSV_INACTIVITY_THS            0x55
#define LSM6DSV_TAP_CFG0                  0x56
#define LSM6DSV_TAP_CFG1                  0x57
#define LSM6DSV_TAP_CFG2                  0x58
#define LSM6DSV_TAP_THS_6D                0x59
#define LSM6DSV_TAP_DUR                   0x5A
#define LSM6DSV_WAKE_UP_THS               0x5B
#define LSM6DSV_WAKE_UP_DUR               0x5C
#define LSM6DSV_FREE_FALL                 0x5D
#define LSM6DSV_MD1_CFG                   0x5E
#define LSM6DSV_MD2_CFG                   0x5F

#define LSM6DSV_HAODR_CFG                 0x62
#define LSM6DSV_EMB_FUNC_CFG              0x63
#define LSM6DSV_UI_HANDSHAKE_CTRL         0x64
#define LSM6DSV_UI_SPI2_SHARED_0          0x65
#define LSM6DSV_UI_SPI2_SHARED_1          0x66
#define LSM6DSV_UI_SPI2_SHARED_2          0x67
#define LSM6DSV_UI_SPI2_SHARED_3          0x68
#define LSM6DSV_UI_SPI2_SHARED_4          0x69
#define LSM6DSV_UI_SPI2_SHARED_5          0x6A
#define LSM6DSV_CTRL_EIS                  0x6B

#define LSM6DSV_UI_INT_OIS                0x6F
#define LSM6DSV_UI_CTRL1_OIS              0x70
#define LSM6DSV_UI_CTRL2_OIS              0x71
#define LSM6DSV_UI_CTRL3_OIS              0x72
#define LSM6DSV_X_OFS_USR                 0x73
#define LSM6DSV_Y_OFS_USR                 0x74
#define LSM6DSV_Z_OFS_USR                 0x75

#define LSM6DSV_FIFO_DATA_OUT_TAG         0x78
#define LSM6DSV_FIFO_DATA_OUT_X_L         0x79
#define LSM6DSV_FIFO_DATA_OUT_X_H         0x7A
#define LSM6DSV_FIFO_DATA_OUT_Y_L         0x7B
#define LSM6DSV_FIFO_DATA_OUT_Y_H         0x7C
#define LSM6DSV_FIFO_DATA_OUT_Z_L         0x7D
#define LSM6DSV_FIFO_DATA_OUT_Z_H         0x7E

// Embedded functions
#define LSM6DSV_PAGE_SEL                  0x02

#define LSM6DSV_EMB_FUNC_EN_A             0x04
#define LSM6DSV_EMB_FUNC_EN_B             0x05

#define LSM6DSV_EMB_FUNC_EXEC_STATUS      0x07
#define LSM6DSV_PAGE_ADDR                 0x08
#define LSM6DSV_PAGE_VALUE                0x09
#define LSM6DSV_EMB_FUNC_INT1             0x0A
#define LSM6DSV_FSM_INT1                  0x0B
 
#define LSM6DSV_EMB_FUNC_INT2             0x0E
#define LSM6DSV_FSM_INT2                  0x0F
 
#define LSM6DSV_EMB_FUNC_STATUS           0x12
#define LSM6DSV_FSM_STATUS                0x13
 
#define LSM6DSV_PAGE_RW                   0x17

#define LSM6DSV_EMB_FUNC_FIFO_EN_A        0x44

#define LSM6DSV_FSM_ENABLE                0x46
 
#define LSM6DSV_FSM_LONG_COUNTER_L        0x48
#define LSM6DSV_FSM_LONG_COUNTER_H        0x49
 
#define LSM6DSV_INT_ACK_MASK              0x4B
#define LSM6DSV_FSM_OUTS1                 0x4C
#define LSM6DSV_FSM_OUTS2                 0x4D
#define LSM6DSV_FSM_OUTS3                 0x4E
#define LSM6DSV_FSM_OUTS4                 0x4F
#define LSM6DSV_FSM_OUTS5                 0x50
#define LSM6DSV_FSM_OUTS6                 0x51
#define LSM6DSV_FSM_OUTS7                 0x52
#define LSM6DSV_FSM_OUTS8                 0x53
 
#define LSM6DSV_SFLP_ODR                  0x5E
#define LSM6DSV_FSM_ODR                   0x5F
 
#define LSM6DSV_STEP_COUNTER_L            0x62
#define LSM6DSV_STEP_COUNTER_H            0x63
#define LSM6DSV_EMB_FUNC_SRC              0x64

#define LSM6DSV_EMB_FUNC_INIT_A           0x66
#define LSM6DSV_EMB_FUNC_INIT_B           0x67

// Embedded Features page 0
#define LSM6DSV_SFLP_GAME_GBIASX_L        0x6E
#define LSM6DSV_SFLP_GAME_GBIASX_H        0x6F
#define LSM6DSV_SFLP_GAME_GBIASY_L        0x70
#define LSM6DSV_SFLP_GAME_GBIASY_H        0x71
#define LSM6DSV_SFLP_GAME_GBIASZ_L        0x72
#define LSM6DSV_SFLP_GAME_GBIASZ_H        0x73

#define LSM6DSV_EXT_SENSITIVITY_L         0xBA
#define LSM6DSV_EXT_SENSITIVITY_H         0xBB
#define LSM6DSV_EXT_OFFX_L                0xC0
#define LSM6DSV_EXT_OFFX_H                0xC1
#define LSM6DSV_EXT_OFFY_L                0xC2
#define LSM6DSV_EXT_OFFY_H                0xC3
#define LSM6DSV_EXT_OFFZ_L                0xC4
#define LSM6DSV_EXT_OFFZ_H                0xC5
#define LSM6DSV_EXT_MATRIX_XX_L           0xC6
#define LSM6DSV_EXT_MATRIX_XX_H           0xC7
#define LSM6DSV_EXT_MATRIX_XY_L           0xC8
#define LSM6DSV_EXT_MATRIX_XY_H           0xC9
#define LSM6DSV_EXT_MATRIX_XZ_L           0xCA
#define LSM6DSV_EXT_MATRIX_XZ_H           0xCB
#define LSM6DSV_EXT_MATRIX_YY_L           0xCC
#define LSM6DSV_EXT_MATRIX_YY_H           0xCD
#define LSM6DSV_EXT_MATRIX_YZ_L           0xCE
#define LSM6DSV_EXT_MATRIX_YZ_H           0xCF
#define LSM6DSV_EXT_MATRIX_ZZ_L           0xD0
#define LSM6DSV_EXT_MATRIX_ZZ_H           0xD1
#define LSM6DSV_EXT_CFG_A                 0xD4
#define LSM6DSV_EXT_CFG_B                 0xD5

// Embedded Features page 1
#define LSM6DSV_FSM_LC_TIMEOUT_L          0x7A
#define LSM6DSV_FSM_LC_TIMEOUT_H          0x7B
#define LSM6DSV_FSM_PROGRAMS              0x7C
#define LSM6DSV_FSM_START_ADD_L           0x7E
#define LSM6DSV_FSM_START_ADD_H           0x7F
#define LSM6DSV_PEDO_CMD_REG              0x83
#define LSM6DSV_PEDO_DEB_STEPS_CONF       0x84
#define LSM6DSV_PEDO_SC_DELTAT_L          0xD0
#define LSM6DSV_PEDO_SC_DELTAT_H          0xD1

// Embedded Features page 2
#define LSM6DSV_EXT_FORMAT                0x00

#define LSM6DSV_EXT_3BYTE_SENSITIVITY_L   0x02
#define LSM6DSV_EXT_3BYTE_SENSITIVITY_H   0x03

#define LSM6DSV_EXT_3BYTE_OFFSET_XL       0x06
#define LSM6DSV_EXT_3BYTE_OFFSET_L        0x07
#define LSM6DSV_EXT_3BYTE_OFFSET_H        0x08

// SPI2 Register Map
#define LSM6DSV_SPI2_WHO_AM_I             0x0F

#define LSM6DSV_SPI2_STATUS_REG_OIS       0x1E

#define LSM6DSV_SPI2_OUT_TEMP_L           0x20
#define LSM6DSV_SPI2_OUT_TEMP_H           0x21
#define LSM6DSV_SPI2_OUTX_L_G_OIS         0x22
#define LSM6DSV_SPI2_OUTX_H_G_OIS         0x23
#define LSM6DSV_SPI2_OUTY_L_G_OIS         0x24
#define LSM6DSV_SPI2_OUTY_H_G_OIS         0x25
#define LSM6DSV_SPI2_OUTZ_L_G_OIS         0x26
#define LSM6DSV_SPI2_OUTZ_H_G_OIS         0x27
#define LSM6DSV_SPI2_OUTX_L_A_OIS         0x28
#define LSM6DSV_SPI2_OUTX_H_A_OIS         0x29
#define LSM6DSV_SPI2_OUTY_L_A_OIS         0x2A
#define LSM6DSV_SPI2_OUTY_H_A_OIS         0x2B
#define LSM6DSV_SPI2_OUTZ_L_A_OIS         0x2C
#define LSM6DSV_SPI2_OUTZ_H_A_OIS         0x2D

#define LSM6DSV_SPI2_HANDSHAKE_CTRL       0x6E
#define LSM6DSV_SPI2_INT_OIS              0x6F
#define LSM6DSV_SPI2_CTRL1_OIS            0x70
#define LSM6DSV_SPI2_CTRL2_OIS            0x70
#define LSM6DSV_SPI2_CTRL3_OIS            0x70

// Sensor Hub Registers
#define LSM6DSV_SENSOR_HUB_1              0x02
#define LSM6DSV_SENSOR_HUB_2              0x03
#define LSM6DSV_SENSOR_HUB_3              0x04
#define LSM6DSV_SENSOR_HUB_4              0x05
#define LSM6DSV_SENSOR_HUB_5              0x06
#define LSM6DSV_SENSOR_HUB_6              0x07
#define LSM6DSV_SENSOR_HUB_7              0x08
#define LSM6DSV_SENSOR_HUB_8              0x09
#define LSM6DSV_SENSOR_HUB_9              0x0A
#define LSM6DSV_SENSOR_HUB_10             0x0B
#define LSM6DSV_SENSOR_HUB_11             0x0C
#define LSM6DSV_SENSOR_HUB_12             0x0D
#define LSM6DSV_SENSOR_HUB_13             0x0E
#define LSM6DSV_SENSOR_HUB_14             0x0F
#define LSM6DSV_SENSOR_HUB_15             0x10
#define LSM6DSV_SENSOR_HUB_16             0x11
#define LSM6DSV_SENSOR_HUB_17             0x12
#define LSM6DSV_SENSOR_HUB_18             0x13
#define LSM6DSV_MASTER_CONFIG             0x14
#define LSM6DSV_SLV0_ADD                  0x15
#define LSM6DSV_SLV0_SUBADD               0x16
#define LSM6DSV_SLV0_CONFIG               0x17
#define LSM6DSV_SLV1_ADD                  0x18
#define LSM6DSV_SLV1_SUBADD               0x19
#define LSM6DSV_SLV1_CONFIG               0x1A
#define LSM6DSV_SLV2_ADD                  0x1B
#define LSM6DSV_SLV2_SUBADD               0x1C
#define LSM6DSV_SLV2_CONFIG               0x1D
#define LSM6DSV_SLV3_ADD                  0x1E
#define LSM6DSV_SLV3_SUBADD               0x1F
#define LSM6DSV_SLV3_CONFIG               0x20
#define LSM6DSV_DATAWRITE_SLV0            0x21
#define LSM6DSV_STATUS_MASTER             0x22

#define LSM6DSV_ADDRESS           0x6A   // Address of LSM6DSV accel/gyro when ADO = 0


#define AFS_2G  0x00
#define AFS_4G  0x01
#define AFS_8G  0x02
#define AFS_16G 0x03

#define GFS_125DPS  0x00
#define GFS_250DPS  0x01
#define GFS_500DPS  0x02
#define GFS_1000DPS 0x03
#define GFS_2000DPS 0x04
#define GFS_4000DPS 0x05

#define AODR_PwrDwn  0x00  // default
#define AODR_1_875Hz 0x01  // not in HA mode
#define AODR_7_5Hz   0x02  // not in HA mode
#define AODR_15Hz    0x03
#define AODR_30Hz    0x04
#define AODR_60Hz    0x05
#define AODR_120Hz   0x06
#define AODR_240Hz   0x07
#define AODR_480Hz   0x08 // not in LP modes
#define AODR_960Hz   0x09 // not in LP modes
#define AODR_1920Hz  0x0A // not in LP modes
#define AODR_3840Hz  0x0B // not in normal or LP modes
#define AODR_7680Hz  0x0C // not in normal or LP modes

#define GODR_PwrDwn  0x00 // default
#define GODR_7_5Hz   0x02 // not in HA mode
#define GODR_15Hz    0x03
#define GODR_30Hz    0x04
#define GODR_60Hz    0x05
#define GODR_120Hz   0x06
#define GODR_240Hz   0x07
#define GODR_480Hz   0x08 // not in LP mode
#define GODR_960Hz   0x09 // not in LP mode
#define GODR_1920Hz  0x0A // not in LP mode
#define GODR_3840Hz  0x0B // not in LP mode
#define GODR_7680Hz  0x0C // not in LP mode

// Accel power modes
#define A_PWRMD_HiPerf   0x00  // default, up to 7680 Hz ODR
#define A_PWRMD_HiAccu   0x01  // set ODR in HAODR_CFG, up to 8000 Hz
#define A_PWRMD_ODR_Trig 0x03
#define A_PWRMD_LPWR1    0x04 // two measurements averaged, 2300 Hz bandwidth
#define A_PWRMD_LPWR2    0x05 // four measurements averaged, 912 Hz bandwidth
#define A_PWRMD_LPWR3    0x06 // eight measurements averaged, 431 Hz bandwidth, Low Power Modes up to 240 Hz ODR
#define A_PWRMD_Normal   0x07 // up to 1920 Hz ODR

// Gyro power modes
#define G_PWRMD_HiPerf   0x00  // default, up to 7680 Hz ODR
#define G_PWRMD_HiAccu   0x01  // set ODR in HAODR_CFG, up to 8000 Hz
#define G_PWRMD_ODR_Trig 0x03
#define G_PWRMD_Sleep    0x04 //  Fast wake up
#define G_PWRMD_LPWR     0x05 //  Low power Mode up to 240 Hz ODR

#define SFLP_ODR_15   0x00   
#define SFLP_ODR_30   0x01  
#define SFLP_ODR_60   0x02   
#define SFLP_ODR_120  0x03  // default
#define SFLP_ODR_240  0x04   
#define SFLP_ODR_480  0x05 // must be less than the AODR

// define FIFO modes
#define BYPASS                  0x00
#define FIFOMODE                0x01
#define CONTINUOUSWTM_TO_FULL   0x02
#define CONTINUOUS_TO_FIFO      0x03
#define BYPASS_TO_CONTINUOUS    0x04
#define CONTINUOUS              0x06
#define BYPASS_TO_FIFO          0x07

#define FIFOODR_NOBATCH 0x00
#define FIFOODR_1_875Hz 0x01  
#define FIFOODR_7_5Hz   0x02   
#define FIFOODR_15Hz    0x03
#define FIFOODR_30Hz    0x04
#define FIFOODR_60Hz    0x05
#define FIFOODR_120Hz   0x06
#define FIFOODR_240Hz   0x07
#define FIFOODR_480Hz   0x08  
#define FIFOODR_960Hz   0x09  
#define FIFOODR_1920Hz  0x0A  
#define FIFOODR_3840Hz  0x0B  
#define FIFOODR_7680Hz  0x0C  

#define SLPMODE0 0x00  // stationary mode, no change in AODR or GODR upon inactivity
#define SLPMODE1 0x01  // accel in LPMode 1, gyro unchanged upon inactivity
#define SLPMODE2 0x02  // accel in LPMode 1, gyro in sleep mode (fast wake) upon inactivity
#define SLPMODE3 0x03  // accel in LPMode 1, gyro in power down (slow wake) upon inactivity


class LSM6DSV
{
  public:
  LSM6DSV(I2Cdev* i2c_bus);
  float getAres(uint8_t Ascale);
  float getGres(uint8_t Gscale);
  uint8_t getChipID();
  void accelPowerDown();
  void accelPowerUp(uint8_t AODR);
  void gyroPowerDown();
  void gyroPowerUp(uint8_t GODR);
  void gyroSleep();
  void gyroWake(uint8_t G_PwrMode);
  void init(uint8_t Ascale, uint8_t Gscale, uint8_t AODR, uint8_t GODR, uint8_t A_PwrMode, uint8_t G_PwrMode);
  void AGoffsetBias(float * dest1, float * dest2);
  void reset();
  void selfTest();
  void readAccelGyroData(int16_t * destination);
  uint8_t DRstatus();
  uint8_t ACTstatus();
  uint8_t EMBstatus();
  uint8_t FSMstatus();
  void ActivityDetect(uint8_t inactiveTime, uint8_t sleepMode);
  void singleTapDetect();
  void TiltDetect();
  void SFLP(uint8_t SFLPODR);
  void initFIFO(uint8_t fmode, uint8_t wtm, uint8_t FIFO_AODR, uint8_t FIFO_GODR);
  void FIFOStatus(uint8_t * dest);
  void FIFOReset();
  void FIFOOutput(uint8_t * dest);
  void D6DOrientationDetect();
  uint8_t wakeSource();
  uint8_t TapSource();
  uint8_t D6DSource();
  uint16_t FloattoHalf(float f);
  float HalftoFloat(uint16_t n);
  void convertQ(uint16_t * fifoHF, float * sflpq);
  void FSMprograms();
  void FSMWristTiltDetect();
  void FSMMotionDetect();
  void FSMShakeDetect();
  void FSMGlanceDetect();
  private:
  float _aRes, _gRes;
  I2Cdev* _i2c_bus;
};

#endif
