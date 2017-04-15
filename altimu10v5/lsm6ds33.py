# -*- coding: utf-8 -*-

"""Python library module for LSM6DS33 accelerometer and gyroscope.
This module for the Raspberry Pi computer helps interface the LSM6DS33
accelerometer and gyro.The library makes it easy to read
the raw accelerometer and gyro data through I²C interface and it also provides
methods for getting angular velocity and g forces.

The datasheet for the LSM6DS33 is available at
[https://www.pololu.com/file/download/LSM6DS33.pdf?file_id=0J1087]
"""

from i2c import I2C
from time import sleep
from constants import *


class LSM6DS33(I2C):
    """ Set up and access LSM6DS33 accelerometer and gyroscope.
    """

    # Output registers used by the gyroscope
    gyro_registers = [
        LSM6DS33_OUTX_L_G,  # low byte of X value
        LSM6DS33_OUTX_H_G,  # high byte of X value
        LSM6DS33_OUTY_L_G,  # low byte of Y value
        LSM6DS33_OUTY_H_G,  # high byte of Y value
        LSM6DS33_OUTZ_L_G,  # low byte of Z value
        LSM6DS33_OUTZ_H_G,  # high byte of Z value
    ]

    # Output registers used by the accelerometer
    accel_registers = [
        LSM6DS33_OUTX_L_XL,  # low byte of X value
        LSM6DS33_OUTX_H_XL,  # high byte of X value
        LSM6DS33_OUTY_L_XL,  # low byte of Y value
        LSM6DS33_OUTY_H_XL,  # high byte of Y value
        LSM6DS33_OUTZ_L_XL,  # low byte of Z value
        LSM6DS33_OUTZ_H_XL,  # high byte of Z value
    ]

    def __init__(self, busId=1):
        """ Set up I2C connection and initialize some flags and values.
        """

        super(LSM6DS33, self).__init__(busId)
        self.is_accel_enabled = False
        self.is_gyro_enabled = False

        self.is_gyro_calibrated = False
        self.gyro_cal = [0, 0, 0]

    def __del__(self):
        """ Clean up."""
        try:
            # Power down accelerometer and gyro
            self.writeRegister(LSM6DS33_ADDR, LSM6DS33_CTRL1_XL, 0x00)
            self.writeRegister(LSM6DS33_ADDR, LSM6DS33_CTRL2_G, 0x00)
            super(LSM6DS33, self).__del__()
            print('Destroying')
        except:
            pass

    def enable(self, accelerometer=True, gyroscope=True, calibration=True):
        """ Enable and set up the given sensors in the IMU."""
        if accelerometer:
            # 1.66 kHz (high performance) / +/- 4g
            # binary value -> 0b01011000, hex value -> 0x58
            self.write_register(LSM6DS33_ADDR, LSM6DS33_CTRL1_XL, 0x58)
            self.is_accel_enabled = True
        if gyroscope:
            # 208 Hz (high performance) / 1000 dps
            # binary value -> 0b01011000, hex value -> 0x58
            self.write_register(LSM6DS33_ADDR, LSM6DS33_CTRL2_G, 0x58)
            self.is_gyro_enabled = True
        if calibration:
            self.calibrate()
            self.is_gyro_calibrated = True
            self.is_accel_calibrated = True

    def calibrate(self, iterations=2000):
        """ Calibrate the gyro's raw values."""
        print('Calibrating...')

        for i in range(iterations):
            gyro_raw = self.get_gyroscope_raw()
            self.gyro_cal[0] += gyro_raw[0]
            self.gyro_cal[1] += gyro_raw[1]
            self.gyro_cal[2] += gyro_raw[2]
            sleep(0.004)

        self.gyro_cal[0] /= iterations
        self.gyro_cal[1] /= iterations
        self.gyro_cal[2] /= iterations

        print('Calibration Done')

    def get_gyroscope_raw(self):
        """ Return a 3D vector of raw gyro data.
        """
        # Check if gyroscope has been enabled
        if not self.is_gyro_enabled:
            raise(Exception('Gyroscope is not enabled!'))

        # Read register outputs and combine low and high byte values
        x_low = self.read_register(LSM6DS33_ADDR, self.gyro_registers[0])
        x_hi = self.read_register(LSM6DS33_ADDR, self.gyro_registers[1])
        y_low = self.read_register(LSM6DS33_ADDR, self.gyro_registers[2])
        y_hi = self.read_register(LSM6DS33_ADDR, self.gyro_registers[3])
        z_low = self.read_register(LSM6DS33_ADDR, self.gyro_registers[4])
        z_hi = self.read_register(LSM6DS33_ADDR, self.gyro_registers[5])

        x_val = self.combine_signed_lo_hi(x_low, x_hi)
        y_val = self.combine_signed_lo_hi(y_low, y_hi)
        z_val = self.combine_signed_lo_hi(z_low, z_hi)

        sensor_data = [x_val, y_val, z_val]

        # Return the vector
        if self.is_gyro_calibrated:
            calibrated_gyro_data = sensor_data
            calibrated_gyro_data[0] -= self.gyro_cal[0]
            calibrated_gyro_data[1] -= self.gyro_cal[1]
            calibrated_gyro_data[2] -= self.gyro_cal[2]
            return calibrated_gyro_data
        else:
            return sensor_data

    def get_gyro_angular_velocity(self):
        """ Return a 3D vector of the angular velocity measured by the gyro
            in degrees/second.
        """
        # Check if gyroscope has been enabled
        if not self.is_gyro_enabled:
            raise(Exception('Gyroscope is not enabled!'))

        # Check if gyroscope has been calibrated
        if not self.is_gyro_calibrated:
            raise(Exception('Gyroscope is not calibrated!'))

        gyro_data = self.get_gyroscope_raw()

        gyro_data[0] = (gyro_data[0] * GYRO_GAIN) / 1000
        gyro_data[1] = (gyro_data[1] * GYRO_GAIN) / 1000
        gyro_data[2] = (gyro_data[2] * GYRO_GAIN) / 1000

        return gyro_data

    def get_accelerometer_raw(self):
        """ Return a 3D vector of raw accelerometer data.
        """

        # Check if accelerometer has been enabled
        if not self.is_accel_enabled:
            raise(Exception('Accelerometer is not enabled!'))

        # Read register outputs and combine low and high byte values
        x_low = self.read_register(LSM6DS33_ADDR, self.accel_registers[0])
        x_hi = self.read_register(LSM6DS33_ADDR, self.accel_registers[1])
        y_low = self.read_register(LSM6DS33_ADDR, self.accel_registers[2])
        y_hi = self.read_register(LSM6DS33_ADDR, self.accel_registers[3])
        z_low = self.read_register(LSM6DS33_ADDR, self.accel_registers[4])
        z_hi = self.read_register(LSM6DS33_ADDR, self.accel_registers[5])

        x_val = self.combine_signed_lo_hi(x_low, x_hi)
        y_val = self.combine_signed_lo_hi(y_low, y_hi)
        z_val = self.combine_signed_lo_hi(z_low, z_hi)

        # Return the vector
        return [x_val, y_val, z_val]

    def get_accelerometer_g_forces(self):
        """ Return a 3D vector of the g forces measured by the accelerometer"""
        [x_val, y_val, z_val] = self.get_accelerometer_raw()

        x_val = (x_val * ACCEL_CONVERSION_FACTOR) / 1000
        y_val = (y_val * ACCEL_CONVERSION_FACTOR) / 1000
        z_val = (z_val * ACCEL_CONVERSION_FACTOR) / 1000

        return [x_val, y_val, z_val]
