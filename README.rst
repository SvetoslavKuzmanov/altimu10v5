==================================================================
AltIMU-10 v5 library for Raspberry Pi
==================================================================

TODO: Modify the whole file as necessary.

Installation
------------

The easiest way to install most Python packages is via ``easy_install`` or ``pip``::

    $ easy_install altimu10v5

Usage
-----

.. code-block:: python

    from altimu10v5.lsm6ds33 import LSM6DS33
    from time import sleep

    lsm6ds33 = LSM6DS33()
    lsm6ds33.enable()

    while True:
        print(lsm6ds33.get_accelerometer_g_forces())
        print(lsm6ds33.get_gyro_angular_velocity())
        sleep(1)


When the package is installed via ``easy_install`` or ``pip`` this function will be bound to the ``altimu10v5`` executable in the Python installation's ``bin`` directory.
