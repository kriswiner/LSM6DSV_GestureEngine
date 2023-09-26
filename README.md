# GestureEngine
High-accuracy accel/gyro and barometer configured as gesture detector

The idea here is to make use of two new ST sensors (LSM6DSV and LPS22DF) which, in combination, should allow ultra-low-power gesture recognition and detection for applications such as head or limb tracking, ergonometrics, UAVs and general navigation.

I designed a breakout board for the [LSM6DSV](https://www.st.com/resource/en/datasheet/lsm6dsv.pdf) combination accel/gyro with embedded finite-state machine and sensor fusion which I hope will allow six DoF orientation estimation without having to run a fusion filter on the host MCU. I added the [LPS22DF](https://www.st.com/resource/en/datasheet/lps22df.pdf) barometer which allows altitude estimation with ~10 cm accuracy. Both of these sensors can perform these feats while consuming just a few uA of power.

![GestureEngine](https://user-images.githubusercontent.com/6698410/270500414-0c61126a-d074-4be5-9fb3-47e5183ae3b9.jpg)

The plan is to create standalone Arduino sketches for each sensor separately showing basic sensor operations and then combine into a single set of "firmware" that can track gestures. My target is hand gestures and a slightly different approach might be required for head tracking or vehicle tracking, etc. But the basic building blocks should provide a good place to start for other applications.

First up is the LSP22DF barometer. The basic sketch shows how to configure the sensor data rate,d averaging, and low pass filter, set up the interrupt, set up and configure the FIFO, read the barometer raw data and convert to properly scaled pressure and temperature. The scaled pressure is used to convert to an altitude estimate. 

The sketch does not show how to use the barometer in differential mode although this should be straightforward. One particularly interesting use case for differential pressure would be to set the current pressure as a reference, then set up an interrupt for when the change in pressure exceeds some value (like 3 or 5 mBar) and trigger the FIFO to capture data on either side of this transition. This would be useful to monitor "man-down" applications for firefighter, etc. As you can see in the next demo, this can also be done in the main loop just by selecting the proper amount of data averaging and detecting meaningful changes in altitude.

Here are typical results running the sensor at 1 Hz with low pass filter of ODR/4 and two settings for the averaging, either 4 sample or 64 sample averaging. The difference in power usage (according to the data sheet) is 2.5 uA vs 6.3 uA, respectively. It costs more power to average more data, of course. But for this little bit of extra power, the data jitter drops by a factor of ~3:


