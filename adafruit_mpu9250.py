# The MIT License (MIT)
#
# Copyright (c) 2019 wallarug for Robotics Masters
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.
"""
`adafruit_mpu9250`
====================================================
CircuitPython driver from MPU9250 IMU with Gyroscope, Accelerometer and Magenetometer sensors
* Author(s): wallarug

Implementation Notes
--------------------

**Hardware:**

* Sparkfun `9-DOF Accel/Mag/Gyro+Temp Breakout Board - MPU9250
  <https://www.sparkfun.com/products/13762>`_ (Product ID: 13762)

**Software and Dependencies:**

* Adafruit CircuitPython firmware for the ESP8622 and M0-based boards:
  https://github.com/adafruit/circuitpython/releases
* Adafruit's Bus Device library: https://github.com/adafruit/Adafruit_CircuitPython_BusDevice
"""

__version__ = "0.0.0-auto.0"
__repo__ = "https://github.com/robotics-masters/RoboticsMasters_CircuitPython_MPU9250.git"

import time
try:
    import struct
except ImportError:
    import ustruct as struct
    
import adafruit_bus_device.i2c_device as i2c_device
import adafruit_bus_device.spi_device as spi_device
from micropython import const

# Internal constants and register values:
# pylint: disable=bad-whitespace
_MPU9250_ADDRESS_ACCELGYRO       = const(0x68) #corrected
_MPU9250_ADDRESS_MAG             = const(0x69) #corrected
_MPU9250_XG_ID                   = const(0b01101000)
_MPU9250_MAG_ID                  = const(0b00111101)
_MPU9250_ACCEL_MG_LSB_2G         = 0.061
_MPU9250_ACCEL_MG_LSB_4G         = 0.122
_MPU9250_ACCEL_MG_LSB_8G         = 0.244
_MPU9250_ACCEL_MG_LSB_16G        = 0.732
_MPU9250_MAG_MGAUSS_4GAUSS       = 0.14
_MPU9250_MAG_MGAUSS_8GAUSS       = 0.29
_MPU9250_MAG_MGAUSS_12GAUSS      = 0.43
_MPU9250_MAG_MGAUSS_16GAUSS      = 0.58
_MPU9250_GYRO_DPS_DIGIT_245DPS   = 0.00875
_MPU9250_GYRO_DPS_DIGIT_500DPS   = 0.01750
_MPU9250_GYRO_DPS_DIGIT_2000DPS  = 0.07000
_MPU9250_TEMP_LSB_DEGREE_CELSIUS = 8 # 1°C = 8, 25° = 200, etc.
_MPU9250_REGISTER_WHO_AM_I_XG    = const(0x75) #corrected
_MPU9250_REGISTER_CTRL_REG1_G    = const(0x10)
_MPU9250_REGISTER_CTRL_REG2_G    = const(0x11)
_MPU9250_REGISTER_CTRL_REG3_G    = const(0x12)
_MPU9250_REGISTER_TEMP_OUT_L     = const(0x42) #corrected
_MPU9250_REGISTER_TEMP_OUT_H     = const(0x41) #corrected
_MPU9250_REGISTER_STATUS_REG     = const(0x17)
_MPU9250_REGISTER_OUT_X_L_G      = const(0x78) #corrected
_MPU9250_REGISTER_OUT_X_H_G      = const(0x77) #corrected
_MPU9250_REGISTER_OUT_Y_L_G      = const(0x7B) #corrected
_MPU9250_REGISTER_OUT_Y_H_G      = const(0x7A) #corrected
_MPU9250_REGISTER_OUT_Z_L_G      = const(0x7E) #corrected
_MPU9250_REGISTER_OUT_Z_H_G      = const(0x7D) #corrected
_MPU9250_REGISTER_CTRL_REG4      = const(0x1E)
_MPU9250_REGISTER_CTRL_REG5_XL   = const(0x1F)
_MPU9250_REGISTER_CTRL_REG6_XL   = const(0x20)
_MPU9250_REGISTER_CTRL_REG7_XL   = const(0x21)
_MPU9250_REGISTER_CTRL_REG8      = const(0x22)
_MPU9250_REGISTER_CTRL_REG9      = const(0x23)
_MPU9250_REGISTER_CTRL_REG10     = const(0x24)
_MPU9250_REGISTER_OUT_X_L_XL     = const(0x28)
_MPU9250_REGISTER_OUT_X_H_XL     = const(0x29)
_MPU9250_REGISTER_OUT_Y_L_XL     = const(0x2A)
_MPU9250_REGISTER_OUT_Y_H_XL     = const(0x2B)
_MPU9250_REGISTER_OUT_Z_L_XL     = const(0x2C)
_MPU9250_REGISTER_OUT_Z_H_XL     = const(0x2D)
_MPU9250_REGISTER_WHO_AM_I_M     = const(0x0F)
_MPU9250_REGISTER_CTRL_REG1_M    = const(0x02) #corrected
_MPU9250_REGISTER_CTRL_REG2_M    = const(0x09) #corrected
_MPU9250_REGISTER_CTRL_REG3_M    = const(0x22) #notexist
_MPU9250_REGISTER_CTRL_REG4_M    = const(0x23) #notexist
_MPU9250_REGISTER_CTRL_REG5_M    = const(0x24) #notexist
_MPU9250_REGISTER_STATUS_REG_M   = const(0x27)
_MPU9250_REGISTER_OUT_X_L_M      = const(0x28)
_MPU9250_REGISTER_OUT_X_H_M      = const(0x29)
_MPU9250_REGISTER_OUT_Y_L_M      = const(0x2A)
_MPU9250_REGISTER_OUT_Y_H_M      = const(0x2B)
_MPU9250_REGISTER_OUT_Z_L_M      = const(0x2C)
_MPU9250_REGISTER_OUT_Z_H_M      = const(0x2D)
_MPU9250_REGISTER_CFG_M          = const(0x30)
_MPU9250_REGISTER_INT_SRC_M      = const(0x31)
_MAGTYPE                         = True
_XGTYPE                          = False
_SENSORS_GRAVITY_STANDARD        = 9.80665
