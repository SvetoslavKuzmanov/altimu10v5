# -*- coding: utf-8 -*-

"""Python I2C library module.
This class has helper methods for I2C SMBus access on a Raspberry PI.
"""

from smbus import SMBus


class I2C(object):
    """ Class to set up and access I2C devices.
    """

    def __init__(self, busId=1):
        """ Initialize the I2C bus. """
        self._i2c = SMBus(busId)

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

    def combine_signed_lo_hi(self, loByte, hiByte):
        """ Combine low and high bytes to a signed 16 bit value. """
        combined = self.combine_lo_hi(loByte, hiByte)
        return combined if combined < 32768 else (combined - 65536)
