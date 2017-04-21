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

    def read_1d_sensor(self, address, registers):
        """ Return a vector with the combined raw signed 24 bit values
            of the output registers of a 1d sensor.
        """

        xlo_byte = self.read_register(address, registers[0])
        lo_byte = self.read_register(address, registers[1])
        hi_byte = self.read_register(address, registers[2])

        return self.combine_signed_xlo_lo_hi(xlo_byte, lo_byte, hi_byte)

    def read_3d_sensor(self, address, registers):
        """ Return a vector with the combined raw signed 16 bit values
            of the output registers of a 3d sensor.
        """

        # Read register outputs and combine low and high byte values
        x_low = self.read_register(address, registers[0])
        x_hi = self.read_register(address, registers[1])
        y_low = self.read_register(address, registers[2])
        y_hi = self.read_register(address, registers[3])
        z_low = self.read_register(address, registers[4])
        z_hi = self.read_register(address, registers[5])

        x_val = self.combine_signed_lo_hi(x_low, x_hi)
        y_val = self.combine_signed_lo_hi(y_low, y_hi)
        z_val = self.combine_signed_lo_hi(z_low, z_hi)

        return [x_val, y_val, z_val]
