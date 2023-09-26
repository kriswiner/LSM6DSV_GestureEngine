# GestureEngine
High-accuracy accel/gyro and barometer configured as gesture detector

The idea here is to make use of two new ST sensors which, in combination, should allow ultra-low-power gesture recognition and detection for applications such as head or limb tracking, ergonometrics, UAVs and general navigation.

I designed a breakout board for the LSM6DSV combination accel/gyro with embedded finite-state machine and sensor fusion which I hope will allow six DoF orientation estimation without having to run a fusion filter on the host MCU. I added the LSP22DF barometer which allows altitude estimation with ~10 cm accuracy. Both of these sensors can perfrom these feats while consuming a few uA of power.
