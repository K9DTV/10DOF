# Changelog

## 1.00.00

First public release of the 10DOF Pico HUD.

- SSD1306 wireframe airplane with altitude in feet, g-load, temperature in °F, and heading
- MPU-9250-class accelerometer and gyro at 0x68, HMC5883L at 0x1E, FXPQ3115 (BB3115) barometer at 0x60
- Startup gyro and barometer calibration, plus a magnetometer figure-8 stored in `mag_cal.json`
- Host pytest suites for SI conversions and HUD math
- Raspberry Pi Pico or Pico 2
