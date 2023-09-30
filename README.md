# GestureEngine
High-accuracy accel/gyro and barometer configured as gesture detector/tracker

The idea here is to make use of two new ST sensors (LSM6DSV and LPS22DF) which, in combination, should allow ultra-low-power gesture recognition and detection for applications such as head or limb tracking, ergonometrics, UAVs and general navigation.

I designed a breakout board for the [LSM6DSV](https://www.st.com/resource/en/datasheet/lsm6dsv.pdf) combination accel/gyro with embedded finite-state machine and sensor fusion which I hope will allow six DoF orientation estimation without having to run a fusion filter on the host MCU. I added the [LPS22DF](https://www.st.com/resource/en/datasheet/lps22df.pdf) barometer which allows altitude estimation with ~10 cm accuracy. Both of these sensors can perform these feats while consuming just a few uA of power.

![GestureEngine](https://user-images.githubusercontent.com/6698410/270500414-0c61126a-d074-4be5-9fb3-47e5183ae3b9.jpg)

I am using a hand-built [Ladybug](https://www.tindie.com/products/tleracorp/ladybug-stm32l432-development-board/?pt=ac_prod_search) (STM32L432KC) development board (see [here](https://oshpark.com/shared_projects/Yi34KlP5) for pcb design) which has an excellent [Arduino core](https://github.com/GrumpyOldPizza/arduino-STM32L4) making it easy to get the most out of these sensors. The Ladybug uses only 2.4 uA in stop mode so is an excellent ultra-low-power development board for this application. The resulting sketches can be adapted to other Arduino-based MCUs like the ESP32 or Teensy, etc with a bit of effort and a lot more power usage.

The plan is to create standalone Arduino sketches for each sensor separately showing basic sensor operations and then combine into a single set of "firmware" that can track gestures. My target is hand gestures and a slightly different approach might be required for head tracking or vehicle tracking, etc. But the basic building blocks should provide a good place to start for other applications.

**First up is the LSP22DF barometer.** The basic sketch shows how to configure the sensor data rate, averaging, and low pass filter, set up the interrupt, set up and configure the FIFO, read the barometer raw data and convert to properly scaled pressure and temperature. The scaled pressure is used to convert to an altitude estimate. 

The sketch does not show how to use the barometer in differential mode although this should be straightforward. One particularly interesting use case for differential pressure would be to set the current pressure as a reference, then set up an interrupt for when the change in pressure exceeds some value (like 3 or 5 mBar) and trigger the FIFO to capture data on either side of this transition. This would be useful to monitor "man-down" applications for firefighters, etc. As you can see in the following demo, this can also be done in the main loop just by selecting the proper amount of data averaging and detecting meaningful changes in altitude.

Here are typical results running the sensor at 1 Hz with low pass filter of ODR/4 and two settings for the averaging, either 4-sample or 64-sample averaging. The difference in power usage (according to the data sheet) is 2.5 uA vs 6.3 uA, respectively. It costs more power to average more data, of course. But for this little bit of extra power, the data jitter drops by a factor of ~3:

![Altitude test](https://user-images.githubusercontent.com/6698410/270502358-2e9ddef7-a1be-41c0-8efd-57bd10e0dd88.jpg)

The blue triangles are altitude estimated from scaled pressure data with averaging at 4x with the breadboard flat on a table. At 55 seconds, I lifted the breadboard over my head and held it. The ~three feet of elevation change is easy to detect despite the data jitter. With averaging set to 64x the data jitter is dramatically reduced and the discrimination (or signal-to-noise) in elevation change is significantly improved. This improvement comes at a cost of ~4 uA, so there is a simple trade between elevation jitter and power consumption. In either case, the power usage is well within the ultra-low power regime. Further reductions in pressure (and therefore, altitude) jitter are possible as are higher data rates to track dynamic motion all at increased power usage.

**Next is the LSM6DSV combination accel/gyro.** This is a rather complicated sensor with a lot of features; the **LSM6DSV_Basic_Ladybug** sketch demonstrates only a few of them. The basic sketch shows how to configure the accel/gyro full-scale, odr, and bandwidth settings as well as set up the data ready interrupts, perform the self tests, calibrate the accel and gyro for offset biases, read the raw data, properly scale the data and display this on the serial monitor. The 6 DoF Madgwick sensor fusion algorithm running on the STM32L4 MCU is used to generate yaw, pitch, and roll estimates from the scaled data. The sketch demonstrates operation of the FIFO to collect uncompressed accel and gyro data, read the raw data from the FIFO, and reconstruct properly-scaled data therefrom. Lastly, three of the simpler hardware functions of the LSM6DSV are implemented: activity detection, rotation change detection and single tap detection.

Running both the gyro and accel at 240 Hz produces a steady yaw estimate with surprisingly low drift:

![Madgwick yaw test](https://user-images.githubusercontent.com/6698410/271437726-b179273b-bcc6-4766-9d93-19d1ea4e99de.jpg)

In this test,  the yaw, pitch and roll are plotted vs time every second while the breadboard containing the LSM6DSV sensor (see above) is manually held flat and stationary along a table edge. Every minute or so the orientation was changed by 90 degrees (i.e., the 90-degree table turning test), completing a 360-degree turn back to the start. This is a manual test with a USB cable so not super high precision nor high accuracy. It is good enough to detect jitter and drift, especially yaw drift, which is expected for any 6DoF AHRS solution. The absolute accuracy of the measured turning angle is poor; mostly +/- 10 degrees of what should be 90 degrees. While the breadboard is stationary, there is very little (~1 degree or less over ~1 minute) yaw drift. Remarkable. Absolute orientation requires use of a magnetometer with its more complicated calibration requirements, larger suceptibility to environmental interference, and higher BOM cost and complexity. A super-low-jitter 6DoF solution (sans mag) is very attractive for a lot of use cases (especially dynamic gesturing and limb tracking, etc). The next step is to move the sensor fusion processing from the MCU to the LSM6DSV itself and see what the tradeoff is between accuracy and power usage....

It was surprisingly easy to make the SFLP (**sensor fusion low power**) embedded function work (**LSM6DSV_SFLP_Ladybug** sketch). I did have to make some changes from the basic sketch. Firstly, I changed the FIFO handling to be interrupt based and moved FIFO handling out of the RTC Alarm handler as a separate (and only) INT1 activity; no need to read the individual sensors via data ready since all data collection can and should be managed by FIFO reads for highest efficiency. Next, I discovered that the LSM6DSV uses the ENU orientation convention rather than the NED I usually use, so I changed the Madgwick fusion filter accordingly to allow comparison, but this doesn't matter since I dropped the Madgwick fusion filter from this sketch anyway. The FIFO data comes tagged to allow identifying the type of data and also the order (time association) of the data in the FIFO. This is typical serial monitor output including accel, gyro, gravity, and quaternions as well as yaw, pitch and roll which is derived from the quaternions.

![typicaloutput](https://user-images.githubusercontent.com/6698410/271735999-61fbd0fe-bedb-439f-8859-13c893a3495e.jpg)

I am plotting just the first two data sets to make for easier reading. Since I am using 15 Hz data rate for both the accel/gyro ODR and SFLP ODR and a FIFO watermark of 60, I expect to get a FIFO full interrupt every second with 15 sets of accel data, gyro data, gravity data and quaternion data each for a total of 60 seven-byte (tag + six data bytes) FIFO entries.  

In an application, the righter way to manage this is to configure the 256-byte FIFO such that the interrupt triggers when this is full and use the MCU to burst read the FIFO and immediately log the FIFO data via page write (256 bytes) to SPI NOR flash, for example. All the processing would be done off-board, later. This would be the most power-efficient way to collect gesture data.

I haven't taken a look at power usage yet, but I did look at orientation accuracy, which was quite surprising.

![90degreeturntestcomparison](https://user-images.githubusercontent.com/6698410/271735946-8e0f5686-421e-46dc-b806-4ea425c4f02b.jpg)

I am comparing the yaw from the Madgwick sensor fusion solution with the yaw from the SFLP solution again using the simple 90-degree table turning test. Both solutions show excellent stability (not surprising since they are using the same data after all!) but the Madgwick solution is systematically too low with typical turn angles of ~80 degrees whereas the SFLP is showing 90 degrees within a degree or two. In fact, the SFLP is probably more accurate than I am in placing the breadboard against the table edge. Remember, these are not absolute orientation estimates wrt True North like one might get by using a magnetometer and 9 DoF sensor fusion. Both  6DoF solutions here start off at or near 0 degrees from whatever orientation they happen to be in upon power on. So they provide a relative orientation estimation. But for a lot of application, even relative orientation estimation with this kind of low jitter and relative accuracy is plenty good enough.

The last bits of the project are making use of the embedded finite-state machine to manage some gesture recognition functions and then, finally, tying all of this capability together into a prototype device for gesture data logging (which will likely require another pcb spin or two).

