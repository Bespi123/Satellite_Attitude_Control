# MPU-9250-for-TM4C123G
MPU-9250 library for interfacing TI TM4C123G via SPI
- Use PD0 - PD3 of TM4C123G as SPI ports; change it in `MPU_init()` if needed
- Bus frequency set to 80MHz; change it if needed
- IMU algorithm is included in `AHRS.c`. You can get real-time euler angles with it.
