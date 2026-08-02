// Copyright (C) 2021 Toitware ApS.
// Use of this source code is governed by a Zero-Clause BSD license that can
// be found in the EXAMPLES_LICENSE file.

import i2c
import math
import lsm303d show *
import system.storage

/**
Example program demonstrating the 'heading' function of the LSM303D.
*/

main:
  bus := i2c.Bus
    --sda=21
    --scl=22

  device := bus.device Lsm303d.I2C-ADDRESS
  bucket := storage.Bucket.open --flash "toitware/toit-lsm303d"
  // If it doesn't exist, then the default value is used.
  mag-calibration := bucket.get "lsm303d-mag-calibration"
  if mag-calibration:
    print "Using calibration: $mag-calibration"

  lsm303d := Lsm303d device --mag-calibration=mag-calibration
  lsm303d.enable

  top := math.Point3f 1.0 0.0 0.0
  while true:
    print (lsm303d.heading top)
    sleep --ms=100
