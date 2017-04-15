'''
AltIMU-10 v5 library for Raspberry Pi.

Copyright 2017, Svetoslav Kuzmanov.
Licensed under MIT.
'''
import sys
from setuptools import setup

version = "0.1"

setup(name="altimu10v5",
      version=version,
      description="AltIMU-10 v5 library for Raspberry Pi",
      long_description=open("README.rst").read(),
      classifiers=[ # Get strings from http://pypi.python.org/pypi?%3Aaction=list_classifiers
        'Development Status :: 2 - Pre-Alpha',
        'Programming Language :: Python'
      ],
      keywords="altimu10v5 raspberry", # Separate with spaces
      author="Svetoslav Kuzmanov",
      author_email="svetoslav.kuzmanov@gmail.com",
      url="",
      license="MIT",
      packages=['altimu10v5'],
      include_package_data=True,
      zip_safe=False,
      install_requires=['smbus']
)
