// Copyright (C) 2021 Toitware ApS.
// Use of this source code is governed by a Zero-Clause BSD license that can
// be found in the EXAMPLES_LICENSE file.

import i2c
import lsm303d show *
import system.storage

/**
Example program to show how to calibrate the magnetometer of the LSM303D.

While running the program move the sensor in a figure 8 shape. Ideally,
  the sensor should be in almost every angle/orientation possible.

The goal of the calibration is to find the min/max values the sensor measures.

The calibration settings are stored in the flash with the key
  "lsm303d-mag-calibration". This way, any other app on the device can take
  advantage of the calibration settings.
*/

main:
  bucket := storage.Bucket.open --flash "toitware/toit-lsm303d"
  bus := i2c.Bus
    --sda=21
    --scl=22

  device := bus.device Lsm303d.I2C-ADDRESS
  lsm303d := Lsm303d device

  min-x := 0x3FFF_FFFF
  min-y := 0x3FFF_FFFF
  min-z := 0x3FFF_FFFF
  max-x := -(0x3FFF_FFFF)
  max-y := -(0x3FFF_FFFF)
  max-z := -(0x3FFF_FFFF)

  old-calibration := null
  lsm303d.magnetometer.enable
  counter := 0
  while true:
    field := lsm303d.magnetometer.read --raw
    x := field[0]
    y := field[1]
    z := field[2]
    if x < min-x: min-x = x
    if x > max-x: max-x = x
    if y < min-y: min-y = y
    if y > max-y: max-y = y
    if z < min-z: min-z = z
    if z > max-z: max-z = z
    counter++
    // Update the store every 32 values.
    if counter & 0x1F == 0 and
        min-x < max-x and min-y < max-y and min-z < max-z:
      calibration := [min-x, min-y, min-z, max-x, max-y, max-z]
      if calibration != old-calibration:
        old-calibration = calibration
        bucket["lsm303d-mag-calibration"] = calibration
        print "New calibration: $calibration"
    sleep --ms=50
