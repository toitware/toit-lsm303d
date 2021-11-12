// Copyright (C) 2021 Toitware ApS. All rights reserved.
// Use of this source code is governed by a Zero-Clause BSD license that can
// be found in the EXAMPLES_LICENSE file.

import gpio
import i2c
import lsm303d show *

/**
Example program for reading acceleration and magnetometer values
  from the LSM303D.
*/

main:
  bus := i2c.Bus
    --sda=gpio.Pin 21
    --scl=gpio.Pin 22

  device := bus.device Lsm303d.I2C_ADDRESS
  lsm303d := Lsm303d device

  lsm303d.enable
  100.repeat:
    acceleration := lsm303d.accelerometer.read
    print "Acceleration (in m/s²): $acceleration"
    // The temperature sensor does not provide absolute values, but
    //   could be used to measure temperature swings.
    temp := lsm303d.magnetometer.read_temperature
    print "Temperature: $temp"

    field := lsm303d.magnetometer.read
    print "Magnetic field (in micro-tesla): $field"
    sleep --ms=2000
