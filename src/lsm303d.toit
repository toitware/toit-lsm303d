// Copyright (C) 2021 Toitware ApS. All rights reserved.
// Use of this source code is governed by an MIT-style license that can be found
// in the LICENSE file.

import .accelerometer
import .magnetometer
import math
import serial.device as serial

class Lsm303d:
  static I2C-ADDRESS ::= 0b11101  // 6.1.1.
  static I2C-ADDRESS-ALT ::= 0b11110

  accelerometer /Accelerometer
  magnetometer  /Magnetometer

  constructor device/serial.Device --mag-calibration/List?=null:
    accelerometer = Accelerometer device
    magnetometer  = Magnetometer device --calibration=mag-calibration

  /**
  Enables the accelerometer and magnetometer with default values.

  Use $Accelerometer.enable and $Magnetometer.enable to use different values.
  */
  enable:
    accelerometer.enable
    magnetometer.enable

  /**
  Returns the heading of the device with respect to the given base vector.

  Uses the acceleration to find "down". If the device is moved, the heading
    thus might be off.

  Returns the heading in degrees.
  */
  heading base-vector/math.Point3f -> float:
    a-vector := accelerometer.read
    mag-vector := magnetometer.read

    e-vector := vector-cross_ mag-vector a-vector
    e-normalized := vector-normalize_ e-vector
    n-vector := vector-cross_ a-vector e-normalized
    n-normalized := vector-normalize_ n-vector

    heading-rads := math.atan2
        vector-dot_ e-normalized base-vector
        vector-dot_ n-normalized base-vector
    heading := heading-rads * 180 / math.PI
    return heading

vector-cross_ v1/math.Point3f v2/math.Point3f -> math.Point3f:
  return math.Point3f
      v1.y * v2.z - v1.z * v2.y
      v1.z * v2.x - v1.x * v2.z
      v1.x * v2.y - v1.y * v2.x

vector-dot_ v1/math.Point3f v2/math.Point3f -> float:
  return (v1.x * v2.x) + (v1.y * v2.y) + (v1.z * v2.z)

vector-normalize_ v/math.Point3f -> math.Point3f:
  len := math.sqrt (vector-dot_ v v)
  return v / len
