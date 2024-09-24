#bno055

## Issue: BUS_OVER_RUN_ERROR
Ref to this link
https://community.bosch-sensortec.com/t5/MEMS-sensors-forum/BNO055-0x07-error-over-UART/td-p/14740

* adding work around
* Accel + gyro data rate is set also to 100Hz
* magnetic data is 20Hz means every 50ms
* NDOF mode needs magnetic sensor,  and runs under 100Hz
