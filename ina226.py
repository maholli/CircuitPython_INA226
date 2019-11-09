"""
`ina226`
====================================================

CircuitPython driver for the INA220 power and current sensor.
Heavily adapted from the adafruit_INA226 driver: https://www.adafruit.com/product/904

* Author(s): Max Holliday

Implementation Notes
--------------------

"""

from micropython import const
from adafruit_bus_device.i2c_device import I2CDevice

from adafruit_register.i2c_struct import ROUnaryStruct, UnaryStruct
from adafruit_register.i2c_bits import ROBits, RWBits
from adafruit_register.i2c_bit import ROBit

__version__ = "3.2.1"
__repo__ = "https://github.com/adafruit/Adafruit_CircuitPython_INA226.git"

# Bits
# pylint: disable=bad-whitespace
# pylint: disable=too-few-public-methods

# Config Register (R/W)
_REG_CONFIG             = const(0x00)

# number of samples collected and averaged
AVG_1                   = const(0x00) 
AVG_4                   = const(0x01) 
AVG_16                  = const(0x02) 
AVG_64                  = const(0x03) 
AVG_128                 = const(0x04) 
AVG_256                 = const(0x05) 
AVG_512                 = const(0x06) 
AVG_1024                = const(0x07) 

# bus voltage conversion time
VBUSCT_140US            = const(0x00)
VBUSCT_204US            = const(0x01)
VBUSCT_332US            = const(0x02)
VBUSCT_588US            = const(0x03)
VBUSCT_1MS              = const(0x04)
VBUSCT_2MS              = const(0x05)
VBUSCT_4MS              = const(0x06)
VBUSCT_8MS              = const(0x07)

# shunt voltage conversion time
VSHCT_140US             = const(0x00)
VSHCT_204US             = const(0x01)
VSHCT_332US             = const(0x02)
VSHCT_588US             = const(0x03)
VSHCT_1MS               = const(0x04)
VSHCT_2MS               = const(0x05)
VSHCT_4MS               = const(0x06)
VSHCT_8MS               = const(0x07)

# operating mode
POWERDOW                = const(0x00)  # power down
SVOLT_TRIGGERED         = const(0x01)  # shunt voltage triggered
BVOLT_TRIGGERED         = const(0x02)  # bus voltage triggered
SANDBVOLT_TRIGGERED     = const(0x03)  # shunt and bus voltage triggered
ADCOFF                  = const(0x04)  # ADC off
SVOLT_CONTINUOUS        = const(0x05)  # shunt voltage continuous
BVOLT_CONTINUOUS        = const(0x06)  # bus voltage continuous
SANDBVOLT_CONTINUOUS    = const(0x07)  # shunt and bus voltage continuous

# SHUNT VOLTAGE REGISTER (R)
_REG_SHUNTVOLTAGE           = const(0x01)

# BUS VOLTAGE REGISTER (R)
_REG_BUSVOLTAGE             = const(0x02)

# POWER REGISTER (R)
_REG_POWER                  = const(0x03)

# CURRENT REGISTER (R)
_REG_CURRENT                = const(0x04)

# CALIBRATION REGISTER (R/W)
_REG_CALIBRATION            = const(0x05)
# pylint: enable=too-few-public-methods


def _to_signed(num):
    if num > 0x7FFF:
        num -= 0x10000
    return num

class INA226:
    """Driver for the INA226 current sensor"""

    # Basic API:

    # INA226( i2c_bus, i2c_addr)  Create instance of INA226 sensor
    #    :param i2c_bus          The I2C bus the INA226 is connected to
    #    :param i2c_addr (0x40)  Address of the INA226 on the bus (default 0x40)

    # shunt_voltage               RO : shunt voltage scaled to Volts
    # bus_voltage                 RO : bus voltage (V- to GND) scaled to volts (==load voltage)
    # current                     RO : current through shunt, scaled to mA
    # power                       RO : power consumption of the load, scaled to Watt

    # raw_shunt_voltage           RO : Shunt Voltage register (not scaled)
    # raw_bus_voltage             RO : Bus Voltage field in Bus Voltage register (not scaled)
    # conversion_ready            RO : Conversion Ready bit in Bus Voltage register
    # overflow                    RO : Math Overflow bit in Bus Voltage register
    # raw_power                   RO : Power register (not scaled)
    # raw_current                 RO : Current register (not scaled)
    # calibration                 RW : calibration register (note: value is cached)

    def __init__(self, i2c_bus, addr=0x40):
        self.i2c_device = I2CDevice(i2c_bus, addr)
        self.i2c_addr = addr

        # Set chip to known config values to start
        self._cal_value = 0
        self._current_lsb = 0
        self._power_lsb = 0
        self.num_averages    = AVG_256
        self.bus_conv_time   = VBUSCT_2MS
        self.shunt_conv_time = VSHCT_2MS

    # config register break-up
    reset                   = RWBits( 1, _REG_CONFIG, 15, 2, False)
    num_averages            = RWBits( 3, _REG_CONFIG,  9, 2, False)
    bus_conv_time           = RWBits( 3, _REG_CONFIG,  6, 2, False)
    shunt_conv_time         = RWBits( 3, _REG_CONFIG,  3, 2, False)
    mode                    = RWBits( 3, _REG_CONFIG,  0, 2, False)


    # shunt voltage register
    raw_shunt_voltage       = ROUnaryStruct(_REG_SHUNTVOLTAGE, ">h")

    #bus voltage register
    raw_bus_voltage         = ROUnaryStruct(_REG_BUSVOLTAGE, ">h")

    # power and current registers
    raw_power               = ROUnaryStruct(_REG_POWER, ">H")
    raw_current             = ROUnaryStruct(_REG_CURRENT, ">h")

    # calibration register
    _raw_calibration        = UnaryStruct(_REG_CALIBRATION, ">H")



    @property
    def calibration(self):
        """Calibration register (cached value)"""
        return self._cal_value # return cached value

    @calibration.setter
    def calibration(self, cal_value):
        self._cal_value = cal_value # value is cached for ``current`` and ``power`` properties
        self._raw_calibration = self._cal_value

    @property
    def shunt_voltage(self):
        """The shunt voltage (between V+ and V-) in Volts (so +-.327V)"""
        # The least signficant bit is 2.5uV which is 0.0000025 volts
        return self.raw_shunt_voltage * 0.0000025

    @property
    def bus_voltage(self):
        """The bus voltage (between V- and GND) in Volts"""
        # Shift to the right 3 to drop CNVR and OVF and multiply by LSB
        # Each least signficant bit is 4mV
        return self.raw_bus_voltage * 0.00125

    @property
    def current(self):
        """The current through the shunt resistor in milliamps."""
        # Sometimes a sharp load will reset the INA226, which will
        # reset the cal register, meaning CURRENT and POWER will
        # not be available ... always setting a cal
        # value even if it's an unfortunate extra step
        self._raw_calibration = self._cal_value
        # Now we can safely read the CURRENT register!
        return self.raw_current * self._current_lsb

    @property
    def power(self):
        """The power through the load in Watt."""
        # Sometimes a sharp load will reset the INA226, which will
        # reset the cal register, meaning CURRENT and POWER will
        # not be available ... always setting a cal
        # value even if it's an unfortunate extra step
        self._raw_calibration = self._cal_value
        # Now we can safely read the CURRENT register!
        return self.raw_power * self._power_lsb