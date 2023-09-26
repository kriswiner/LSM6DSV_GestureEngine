# GestureEngine
High-accuracy accel/gyro and barometer configured as gesture detector/tracker

The idea here is to make use of two new ST sensors (LSM6DSV and LPS22DF) which, in combination, should allow ultra-low-power gesture recognition and detection for applications such as head or limb tracking, ergonometrics, UAVs and general navigation.

I designed a breakout board for the [LSM6DSV](https://www.st.com/resource/en/datasheet/lsm6dsv.pdf) combination accel/gyro with embedded finite-state machine and sensor fusion which I hope will allow six DoF orientation estimation without having to run a fusion filter on the host MCU. I added the [LPS22DF](https://www.st.com/resource/en/datasheet/lps22df.pdf) barometer which allows altitude estimation with ~10 cm accuracy. Both of these sensors can perform these feats while consuming just a few uA of power.

![GestureEngine](https://user-images.githubusercontent.com/6698410/270500414-0c61126a-d074-4be5-9fb3-47e5183ae3b9.jpg)

I am using a hand-built [Ladybug](https://www.tindie.com/products/tleracorp/ladybug-stm32l432-development-board/?pt=ac_prod_search) (STM32L432KC) development board (see [here](https://oshpark.com/shared_projects/Yi34KlP5) for pcb design) which has an excellent [Arduino core](https://github.com/GrumpyOldPizza/arduino-STM32L4) making it easy to get the most out of these sensors. The Ladybug uses only 2.4 uA in stop mode so is an excellent ultra-low-power development board for this application. The resulting sketches can be adapted to other Arduino-based MCUs like the ESP32 or Teensy, etc with a bit of effort and a lot more power usage.

The plan is to create standalone Arduino sketches for each sensor separately showing basic sensor operations and then combine into a single set of "firmware" that can track gestures. My target is hand gestures and a slightly different approach might be required for head tracking or vehicle tracking, etc. But the basic building blocks should provide a good place to start for other applications.

First up is the LSP22DF barometer. The basic sketch shows how to configure the sensor data rate, averaging, and low pass filter, set up the interrupt, set up and configure the FIFO, read the barometer raw data and convert to properly scaled pressure and temperature. The scaled pressure is used to convert to an altitude estimate. 

The sketch does not show how to use the barometer in differential mode although this should be straightforward. One particularly interesting use case for differential pressure would be to set the current pressure as a reference, then set up an interrupt for when the change in pressure exceeds some value (like 3 or 5 mBar) and trigger the FIFO to capture data on either side of this transition. This would be useful to monitor "man-down" applications for firefighters, etc. As you can see in the following demo, this can also be done in the main loop just by selecting the proper amount of data averaging and detecting meaningful changes in altitude.

Here are typical results running the sensor at 1 Hz with low pass filter of ODR/4 and two settings for the averaging, either 4-sample or 64-sample averaging. The difference in power usage (according to the data sheet) is 2.5 uA vs 6.3 uA, respectively. It costs more power to average more data, of course. But for this little bit of extra power, the data jitter drops by a factor of ~3:

![Altitude test](https://user-images.githubusercontent.com/6698410/270502358-2e9ddef7-a1be-41c0-8efd-57bd10e0dd88.jpg)

The blue triangles are altitude estimated from scaled pressure data with averaging at 4x with the breadboard flat on a table. At 55 seconds, I lifted the breadboard over my head and held it. The ~three feet of elevation change is easy to detect despite the data jitter. With averaging set to 64x the data jitter is dramatically reduced and the discrimination (or signal-to-noise) in elevation change is significantly improved. This improvement comes at a cost of ~4 uA, so there is a simple trade between elevation jitter and power consumption. In either case, the power usage is well within the ultra-low power regime. Further reductions in pressure (and therefore, altitude) jitter are possible as are higher data rates to track dynamic motion all at increased power usage.


