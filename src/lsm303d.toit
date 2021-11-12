// Copyright (C) 2021 Toitware ApS. All rights reserved.
// Use of this source code is governed by a MIT-style license that can be found
// in the LICENSE file.

import .accelerometer
import .magnetometer
import math
import serial.device as serial

class Lsm303d:
  static I2C_ADDRESS ::= 0b11101  // 6.1.1.
  static I2C_ADDRESS_ALT ::= 0b11110

  accelerometer /Accelerometer
  magnetometer  /Magnetometer


  constructor device/serial.Device --mag_calibration/List?=null:
    accelerometer = Accelerometer device
    magnetometer  = Magnetometer device --calibration=mag_calibration

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
  heading base_vector/math.Point3f -> float:
    a_vector := accelerometer.read
    mag_vector := magnetometer.read

    e_vector := vector_cross_ mag_vector a_vector
    e_normalized := vector_normalize_ e_vector
    n_vector := vector_cross_ a_vector e_normalized
    n_normalized := vector_normalize_ n_vector

    heading_rads := math.atan2
        vector_dot_ e_normalized base_vector
        vector_dot_ n_normalized base_vector
    heading := heading_rads * 180 / math.PI
    return heading

vector_cross_ v1/math.Point3f v2/math.Point3f -> math.Point3f:
  return math.Point3f
      v1.y * v2.z - v1.z * v2.y
      v1.x * v2.z - v1.z * v2.x
      v1.x * v2.y - v1.y * v2.x

vector_dot_ v1/math.Point3f v2/math.Point3f -> float:
  return (v1.x * v2.x) + (v1.y * v2.y) + (v1.z * v2.z)

vector_normalize_ v/math.Point3f -> math.Point3f:
  len := math.sqrt (vector_dot_ v v)
  return v / len
