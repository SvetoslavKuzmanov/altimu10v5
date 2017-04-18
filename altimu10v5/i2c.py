# -*- coding: utf-8 -*-

"""Python I2C library module.
This class has helper methods for I2C SMBus access on a Raspberry PI.
"""

from smbus import SMBus


class I2C(object):
    """ Class to set up and access I2C devices.
    """

    def __init__(self, bus_id=1):
        """ Initialize the I2C bus. """
        self._i2c = SMBus(bus_id)

    def __del__(self):
        """ Clean up. """
        try:
            del(self._i2c)
        except:
            pass

    def write_register(self, address, register, value):
        """ Write a single byte to a I2C register. Return the value the
            register had before the write.
        """
        value_old = self.read_register(address, register)
        self._i2c.write_byte_data(address, register, value)
        return value_old

    def read_register(self, address, register):
        """ Read a single I2C register. """
        return self._i2c.read_byte_data(address, register)

    def combine_lo_hi(self, lo_byte, hi_byte):
        """ Combine low and high bytes to an unsigned 16 bit value. """
        return (hi_byte << 8) | lo_byte

    def combine_signed_lo_hi(self, lo_byte, hi_byte):
        """ Combine low and high bytes to a signed 16 bit value. """
        combined = self.combine_lo_hi(lo_byte, hi_byte)
        return combined if combined < 32768 else (combined - 65536)

    def combine_xlo_lo_hi(self, xlo_byte, lo_byte, hi_byte):
        """ Combine extra low, low, and high bytes to an unsigned
            24 bit value.
        """
        return (xlo_byte | lo_byte << 8 | hi_byte << 16)

    def combine_signed_xlo_lo_hi(self, xlo_byte, lo_byte, hi_byte):
        """ Combine extra low, low, and high bytes to a signed 24 bit value. """
        combined = self.combine_xlo_lo_hi(xlo_byte, lo_byte, hi_byte)
        return combined if combined < 8388608 else (combined - 16777216)
