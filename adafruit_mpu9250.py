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
_MPU9250_ADDRESS_MAG             = const(0x0C) #corrected - bypass
_MPU9250_XG_ID                   = const(0b01110001) #corrected (0x71)
_MPU9250_MAG_ID                  = const(0b01001000) #corrected (0x48) 0 1 0 0   1 0 0 0

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


## MPU6500 - Accel and Gyro
_MPU9250_REGISTER_WHO_AM_I_XG    = const(0x75) #reports 0x71

_MPU9250_SELF_TEST_X_GYRO       = const(0x00)
_MPU9250_SELF_TEST_Y_GYRO       = const(0x01)
_MPU9250_SELF_TEST_Z_GYRO       = const(0x02)
_MPU9250_SELF_TEST_X_ACCEL       = const(0x0D)
_MPU9250_SELF_TEST_Y_ACCEL       = const(0x0E)
_MPU9250_SELF_TEST_Z_ACCEL       = const(0x0F)

_MPU9250_XG_OFFSET_H            = const(0x13)
_MPU9250_XG_OFFSET_L            = const(0x14)
_MPU9250_YG_OFFSET_H            = const(0x15)
_MPU9250_YG_OFFSET_L            = const(0x16)
_MPU9250_ZG_OFFSET_H            = const(0x17)
_MPU9250_ZG_OFFSET_L            = const(0x18)
_MPU9250_XA_OFFSET_H        = const(0x77)
_MPU9250_XA_OFFSET_L        = const(0x78)
_MPU9250_YA_OFFSET_H        = const(0x7A)
_MPU9250_YA_OFFSET_L        = const(0x7B)
_MPU9250_ZA_OFFSET_H        = const(0x7D)
_MPU9250_ZA_OFFSET_L        = const(0x7E)

_MPU9250_SMPLRT_DIV             = const(0x19)
_MPU9250_CONFIG                 = const(0x1A)
_MPU9250_GYRO_CONFIG            = const(0x1B)
_MPU9250_ACCEL_CONFIG           = const(0x1C)   
_MPU9250_ACCEL_CONFIG2          = const(0x1D)
#_MPU9250_LP_ACCEL_ODR           = const(0x1E)
#_MPU9250_WOM_THR                = const(0x1F)

_MPU9250_FIFO_EN                = const(0x23)
_MPU9250_I2C_MST_CTRL           = const(0x24)
_MPU9250_I2C_SLV0_ADDR          = const(0x25)
_MPU9250_I2C_SLV0_REG           = const(0x26)
_MPU9250_I2C_SLV0_CTRL          = const(0x27)
#_MPU9250_I2C_MST_STATUS         = const(0x36)

_MPU9250_INT_PIN_CFG            = const(0x37) # for bypass
_MPU9250_INT_ENABLE             = const(0x38) # for bypass

_MPU9250_REGISTER_ACCEL_XOUT_H     = const(0x3B)
_MPU9250_REGISTER_ACCEL_XOUT_L     = const(0x3C)
_MPU9250_REGISTER_ACCEL_YOUT_H     = const(0x3D)
_MPU9250_REGISTER_ACCEL_YOUT_L     = const(0x3E)
_MPU9250_REGISTER_ACCEL_ZOUT_H     = const(0x3F)
_MPU9250_REGISTER_ACCEL_ZOUT_L     = const(0x40)
_MPU9250_REGISTER_TEMP_OUT_H     = const(0x41)
_MPU9250_REGISTER_TEMP_OUT_L     = const(0x42)
_MPU9250_REGISTER_GYRO_XOUT_H      = const(0x43)
_MPU9250_REGISTER_GYRO_XOUT_L      = const(0x44)
_MPU9250_REGISTER_GYRO_YOUT_H      = const(0x45)
_MPU9250_REGISTER_GYRO_YOUT_L      = const(0x46)
_MPU9250_REGISTER_GYRO_ZOUT_H      = const(0x47)
_MPU9250_REGISTER_GYRO_ZOUT_L      = const(0x48)

_MPU9250_USER_CTRL      = const(0x6A)
_MPU9250_PWR_MGMT_1     = const(0x6B)
_MPU9250_PWR_MGMT_2     = const(0x6C)

_MPU9250_FIFO_COUNT_H     = const(0x72)
_MPU9250_FIFO_COUNT_L     = const(0x73)
_MPU9250_FIFO_R_W         = const(0x74)


## AK8963 - Magenetometer
_MPU9250_REGISTER_WHO_AM_I_M     = const(0x00) #reports 0x48
_MPU9250_REGISTER_STATUS_REG1_M   = const(0x02) # ST1
_MPU9250_REGISTER_MAG_XOUT_L      = const(0x03)
_MPU9250_REGISTER_MAG_XOUT_H      = const(0x04)
_MPU9250_REGISTER_MAG_YOUT_L      = const(0x05)
_MPU9250_REGISTER_MAG_YOUT_H      = const(0x06)
_MPU9250_REGISTER_MAG_ZOUT_L      = const(0x07)
_MPU9250_REGISTER_MAG_ZOUT_H      = const(0x08)
_MPU9250_REGISTER_STATUS_REG2_M   = const(0x09) # ST2
_MPU9250_REGISTER_ASTC_M          = const(0x0C) # self-test
_MPU9250_REGISTER_MAG_ASAX        = const(0x10)
_MPU9250_REGISTER_MAG_ASAY        = const(0x11)
_MPU9250_REGISTER_MAG_ASAZ        = const(0x12)


##  See Page 51 of Documentation for information about this register.
##   Link: https://www.invensense.com/wp-content/uploads/2015/02/RM-MPU-9250A-00-v1.6.pdf
_MPU9250_REGISTER_CNTL_M    = const(0x0A) #corrected - RW (CNTL)


_MAGTYPE                         = True
_XGTYPE                          = False
_SENSORS_GRAVITY_STANDARD        = 9.80665


# User facing constants/module globals.
ACCELRANGE_2G                = (0b00 << 3)
ACCELRANGE_16G               = (0b01 << 3)
ACCELRANGE_4G                = (0b10 << 3)
ACCELRANGE_8G                = (0b11 << 3)
MAGGAIN_4GAUSS               = (0b00 << 5)  # +/- 4 gauss
MAGGAIN_8GAUSS               = (0b01 << 5)  # +/- 8 gauss
MAGGAIN_12GAUSS              = (0b10 << 5)  # +/- 12 gauss
MAGGAIN_16GAUSS              = (0b11 << 5)  # +/- 16 gauss
GYROSCALE_245DPS             = (0b00 << 3)  # +/- 245 degrees/s rotation
GYROSCALE_500DPS             = (0b01 << 3)  # +/- 500 degrees/s rotation
GYROSCALE_2000DPS            = (0b11 << 3)  # +/- 2000 degrees/s rotation
# pylint: enable=bad-whitespace


def _twos_comp(val, bits):
    # Convert an unsigned integer in 2's compliment form of the specified bit
    # length to its signed integer value and return it.
    if val & (1 << (bits - 1)) != 0:
        return val - (1 << bits)
    return val


class MPU9250:
    """Driver for the MPU9250 accelerometer, magnetometer, gyroscope."""
    
    # Class-level buffer for reading and writing data with the sensor.
    # This reduces memory allocations but means the code is not re-entrant or
    # thread safe!
    _BUFFER = bytearray(6)

    
    # TODO:  Fix this for the MPU9250 - it has different registers and methods.
    def __init__(self, address=_MPU9250_ADDRESS_ACCELGYRO):
        ### ACCEL and GYRO SETUP
        # Check ID register for accel/gyro.
        if self._read_u8(_XGTYPE, _MPU9250_REGISTER_WHO_AM_I_XG) != _MPU9250_XG_ID:
            raise RuntimeError('Could not find MPU9250, check wiring!')
        
        # wake up device - clear sleep mode bit (6), enable all sensors
        self._write_u8(_XGTYPE, _MPU9250_PWR_MGMT_1, 0x00)
        time.sleep(0.1)
    
        # get stable time source - 
        self._write_u8(_XGTYPE, _MPU9250_PWR_MGMT_1, 0x01)
        time.sleep(0.2)

        # configure gyro and themometer
        # disable Fsync and set above to 41 and 42 Hz respectively;
        self._write_u8(_XGTYPE, _MPU9250_CONFIG, 0x03)

        # set sample rate = gyroscope output rate/(1 + SMPLRT_DIV)
        self._write_u8(_XGTYPE, _MPU9250_SMPLRT_DIV, 0x04)

        ## Set accelerometer sample rate configuration
        self._write_u8(_XGTYPE, _MPU9250_ACCEL_CONFIG2, 0x03)
        
        ## Set I2C By-Pass
        self._write_u8(_XGTYPE, _MPU9250_INT_PIN_CFG, 0x22) # could also be 0x02 or 0x12
        self._write_u8(_XGTYPE, _MPU9250_INT_ENABLE, 0x01)
        time.sleep(0.1)


        ### MAGNETOMETER SETUP
        # Check ID register for mag.
        if self._read_u8(_MAGTYPE, _MPU9250_REGISTER_WHO_AM_I_M) != _MPU9250_MAG_ID:
            raise RuntimeError('Could not find MPU9250 Magnetometer, enable I2C bypass!')
        print("successfully found Magenetometer")
        # cont mode 1 - 16 Bit 8Hz
        #self._write_u8(_MAGTYPE, _MPU9250_REGISTER_STATUS_REG1_M, ((0x01 << 4) | 0x02)) # 16bit|8hz
        #self._write_u8(_MAGTYPE, _MPU9250_REGISTER_ASTC_M, 0x00)
    
        # Sensitivity Ajustment Values
        self._write_u8(_MAGTYPE, _MPU9250_REGISTER_CNTL_M, 0x0F)
        asax = self._read_u8(_MAGTYPE, _MPU9250_REGISTER_MAG_ASAX)
        asay = self._read_u8(_MAGTYPE, _MPU9250_REGISTER_MAG_ASAY)
        asaz = self._read_u8(_MAGTYPE, _MPU9250_REGISTER_MAG_ASAZ)

        self._adjustments = (
            (0.5 * (asax -128)) / 128 + 1,
            (0.5 * (asax -128)) / 128 + 1,
            (0.5 * (asax -128)) / 128 + 1
        )
        
        # power on
        self._write_u8(_MAGTYPE, _MPU9250_REGISTER_STATUS_REG1_M, (0x02 | 0x10)) # 8hz 16bit

        ### Default ranges for various sensor
        self._accel_mg_lsb = None
        self._mag_mgauss_lsb = 4800.0 / 32760
        self._gyro_dps_digit = None
        self.accel_range = ACCELRANGE_2G
        #self.mag_gain = MAGGAIN_4GAUSS
        self.gyro_scale = GYROSCALE_245DPS

    
    @property
    def accel_range(self):
        """The accelerometer range.  Must be a value of:
          - ACCELRANGE_2G
          - ACCELRANGE_4G
          - ACCELRANGE_8G
          - ACCELRANGE_16G
        """
        reg = self._read_u8(_XGTYPE, _MPU9250_ACCEL_CONFIG) # corrected.
        return (reg & 0b00011000) & 0xFF
    
    @accel_range.setter
    def accel_range(self, val):
        assert val in (ACCELRANGE_2G, ACCELRANGE_4G, ACCELRANGE_8G,
                       ACCELRANGE_16G)
        reg = self._read_u8(_XGTYPE, _MPU9250_ACCEL_CONFIG)
        reg = (reg & ~(0b00011000)) & 0xFF
        reg |= val
        self._write_u8(_XGTYPE, _MPU9250_ACCEL_CONFIG, reg)
        if val == ACCELRANGE_2G:
            self._accel_mg_lsb = _MPU9250_ACCEL_MG_LSB_2G
        elif val == ACCELRANGE_4G:
            self._accel_mg_lsb = _MPU9250_ACCEL_MG_LSB_4G
        elif val == ACCELRANGE_8G:
            self._accel_mg_lsb = _MPU9250_ACCEL_MG_LSB_8G
        elif val == ACCELRANGE_16G:
            self._accel_mg_lsb = _MPU9250_ACCEL_MG_LSB_16G

    # TODO: Patch these two methods up
    @property
    def accel_rate(self):
        """The accelerometer sample rate.  Must be a value of:
          - UNKNOWN
        """
        reg = self._read_u8(_XGTYPE, _MPU9250_ACCEL_CONFIG2) # corrected.
        return (reg & 0b00011000) & 0xFF
    
    @accel_rate.setter
    def accel_rate(self, val):
        assert val in (ACCELRANGE_2G, ACCELRANGE_4G, ACCELRANGE_8G,
                       ACCELRANGE_16G)
        reg = self._read_u8(_XGTYPE, _MPU9250_ACCEL_CONFIG2)
        reg = reg & ~0x0F
        reg |= 0x03
        self._write_u8(_XGTYPE, _MPU9250_ACCEL_CONFIG2, reg)

    @property
    def gyro_scale(self):
        """The gyroscope scale.  Must be a value of:
          - GYROSCALE_245DPS
          - GYROSCALE_500DPS
          - GYROSCALE_2000DPS
        """
        reg = self._read_u8(_XGTYPE, _MPU9250_GYRO_CONFIG) # Corrected.
        return (reg & 0b00011000) & 0xFF

    @gyro_scale.setter
    def gyro_scale(self, val):
        assert val in (GYROSCALE_245DPS, GYROSCALE_500DPS, GYROSCALE_2000DPS)

        reg = self._read_u8(_XGTYPE, _MPU9250_GYRO_CONFIG)
        reg = (reg & ~(0b00011000)) & 0xFF
        reg |= val
        self._write_u8(_XGTYPE, _MPU9250_GYRO_CONFIG, reg)
        if val == GYROSCALE_245DPS:
            self._gyro_dps_digit = _MPU9250_GYRO_DPS_DIGIT_245DPS
        elif val == GYROSCALE_500DPS:
            self._gyro_dps_digit = _MPU9250_GYRO_DPS_DIGIT_500DPS
        elif val == GYROSCALE_2000DPS:
            self._gyro_dps_digit = _MPU9250_GYRO_DPS_DIGIT_2000DPS
    
    def read_accel_raw(self):
        """Read the raw accelerometer sensor values and return it as a
        3-tuple of X, Y, Z axis values that are 16-bit unsigned values.  If you
        want the acceleration in nice units you probably want to use the
        accelerometer property!
        """
        # Read the accelerometer
        self._read_bytes(_XGTYPE, 0x80 | _MPU9250_REGISTER_ACCEL_XOUT_L, 6,
                         self._BUFFER)
        raw_x, raw_y, raw_z = struct.unpack_from('<hhh', self._BUFFER[0:6])
        return (raw_x, raw_y, raw_z)

    @property
    def acceleration(self):
        """The accelerometer X, Y, Z axis values as a 3-tuple of
        m/s^2 values.
        """
        raw = self.read_accel_raw()
        return map(lambda x: x * self._accel_mg_lsb / 1000.0 * _SENSORS_GRAVITY_STANDARD,
                   raw)

    def read_mag_raw(self):
        """Read the raw magnetometer sensor values and return it as a
        3-tuple of X, Y, Z axis values that are 16-bit unsigned values.  If you
        want the magnetometer in nice units you probably want to use the
        magnetometer property!
        """
        # Read the magnetometer
        self._read_bytes(_MAGTYPE, _MPU9250_REGISTER_MAG_XOUT_L, 6,
                         self._BUFFER)
        self._read_u8(_MAGTYPE, _MPU9250_REGISTER_STATUS_REG2_M)
        raw_x, raw_y, raw_z = struct.unpack_from('<hhh', self._BUFFER[0:6])
        return (raw_x, raw_y, raw_z)

    @property
    def magnetic(self):
        """The magnetometer X, Y, Z axis values as a 3-tuple of
        gauss values.
        """
        raw = self.read_mag_raw()
        return map(lambda x: x * self._mag_mgauss_lsb / 1000.0, raw)

    def read_gyro_raw(self):
        """Read the raw gyroscope sensor values and return it as a
        3-tuple of X, Y, Z axis values that are 16-bit unsigned values.  If you
        want the gyroscope in nice units you probably want to use the
        gyroscope property!
        """
        # Read the gyroscope
        self._read_bytes(_XGTYPE, 0x80 | _MPU9250_REGISTER_GYRO_XOUT_L, 6,
                         self._BUFFER)
        raw_x, raw_y, raw_z = struct.unpack_from('<hhh', self._BUFFER[0:6])
        return (raw_x, raw_y, raw_z)

    @property
    def gyro(self):
        """The gyroscope X, Y, Z axis values as a 3-tuple of
        degrees/second values.
        """
        raw = self.read_gyro_raw()
        return map(lambda x: x * self._gyro_dps_digit, raw)
    
    def read_temp_raw(self):
        """Read the raw temperature sensor value and return it as a 12-bit
        signed value.  If you want the temperature in nice units you probably
        want to use the temperature property!
        """
        # Read temp sensor
        self._read_bytes(_XGTYPE, 0x80 | _MPU9250_REGISTER_TEMP_OUT_L, 2,
                         self._BUFFER)
        temp = ((self._BUFFER[1] << 8) | self._BUFFER[0]) >> 4
        return _twos_comp(temp, 12)

    @property
    def temperature(self):
        """The temperature of the sensor in degrees Celsius."""
        # This is just a guess since the starting point (21C here) isn't documented :(
        # See discussion from:
        #  https://github.com/kriswiner/LSM9DS1/issues/3
        temp = self.read_temp_raw()
        temp = 27.5 + temp/16
        return temp

    def _read_u8(self, sensor_type, address):
        # Read an 8-bit unsigned value from the specified 8-bit address.
        # The sensor_type boolean should be _MAGTYPE when talking to the
        # magnetometer, or _XGTYPE when talking to the accel or gyro.
        # MUST be implemented by subclasses!
        raise NotImplementedError()

    def _read_bytes(self, sensor_type, address, count, buf):
        # Read a count number of bytes into buffer from the provided 8-bit
        # register address.  The sensor_type boolean should be _MAGTYPE when
        # talking to the magnetometer, or _XGTYPE when talking to the accel or
        # gyro.  MUST be implemented by subclasses!
        raise NotImplementedError()

    def _write_u8(self, sensor_type, address, val):
        # Write an 8-bit unsigned value to the specified 8-bit address.
        # The sensor_type boolean should be _MAGTYPE when talking to the
        # magnetometer, or _XGTYPE when talking to the accel or gyro.
        # MUST be implemented by subclasses!
        raise NotImplementedError()
        

# TODO: Test if working (copied from Adafruit/LSM9DS1)
class MPU9250_I2C(MPU9250):
    """Driver for the MPU9250 connect over I2C."""

    def __init__(self, i2c):
        self._mag_device = i2c_device.I2CDevice(i2c, _MPU9250_ADDRESS_MAG)
        self._xg_device = i2c_device.I2CDevice(i2c, _MPU9250_ADDRESS_ACCELGYRO)
        super().__init__()

    def _read_u8(self, sensor_type, address):
        if sensor_type == _MAGTYPE:
            device = self._mag_device
        else:
            device = self._xg_device
        with device as i2c:
            self._BUFFER[0] = address & 0xFF
            i2c.write(self._BUFFER, end=1, stop=False)
            i2c.readinto(self._BUFFER, end=1)
        return self._BUFFER[0]

    def _read_bytes(self, sensor_type, address, count, buf):
        if sensor_type == _MAGTYPE:
            device = self._mag_device
        else:
            device = self._xg_device
        with device as i2c:
            buf[0] = address & 0xFF
            i2c.write(buf, end=1, stop=False)
            i2c.readinto(buf, end=count)

    def _write_u8(self, sensor_type, address, val):
        if sensor_type == _MAGTYPE:
            device = self._mag_device
        else:
            device = self._xg_device
        with device as i2c:
            self._BUFFER[0] = address & 0xFF
            self._BUFFER[1] = val & 0xFF
            i2c.write(self._BUFFER, end=2)
        
