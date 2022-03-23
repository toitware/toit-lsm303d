// Copyright (C) 2021 Toitware ApS. All rights reserved.
// Use of this source code is governed by an MIT-style license that can be found
// in the LICENSE file.

import serial.device as serial
import serial.registers as serial
import math

/**
Driver for the magnetometer of the LSM303D module.
*/
class Magnetometer:
  static I2C_ADDRESS ::= 0b11101  // 6.1.1.
  static I2C_ADDRESS_ALT ::= 0b11110

  // Section 7. Table 16. Register mapping.
  static WHO_AM_I_ ::= 0x0F
  static TEMP_OUT_L_ ::= 0x05
  static TEMP_OUT_H_ ::= 0x06
  static OUT_X_L_M_ ::= 0x08
  static OUT_X_H_M_ ::= 0x09
  static OUT_Y_L_M_ ::= 0x0A
  static OUT_Y_H_M_ ::= 0x0B
  static OUT_Z_L_M_ ::= 0x0C
  static OUT_Z_H_M_ ::= 0x0D
  static CTRL5_ ::= 0x24
  static CTRL6_ ::= 0x25
  static CTRL7_ ::= 0x26

  // Section 8.21, Table 47.
  static RATE_3_125HZ ::= 0
  static RATE_6_25HZ  ::= 1
  static RATE_12_5HZ  ::= 2
  static RATE_25HZ    ::= 3
  static RATE_50HZ    ::= 4
  /** Only available if the accelerometer has a rate of more than 50Hz. */
  static RATE_100HZ   ::= 5

  // Section 8.22. Table 50.
  // "G" refers to "Gauss".
  static RANGE_2G  ::= 0
  static RANGE_4G  ::= 1
  static RANGE_8G  ::= 2
  static RANGE_12G ::= 3

  static GAUSS_TO_MICROTESLA_ ::= 100.0

  reg_ /serial.Registers
  calibration_ /List
  range_ /int := 0


  /**
  Constructs a new Magnetometer.

  The $calibration should be a 3-element list, containing the
    the calibration value of each axis. The calibration value is
    simply the average value of min and max raw values (see $(read --raw)).
    Typically, the user moves the sensor in a figure 8, while the
    calibration program is collecting all seen values.
  */
  constructor dev/serial.Device --calibration=[0, 0, 0]:
    reg_ = dev.registers

    calibration_ = calibration

    id := reg_.read_u8 WHO_AM_I_
    // Section 8.6, Table 19.
    if id != 0x49: throw "INVALID_CHIP"

  enable -> none
      --rate  /int = RATE_6_25HZ
      --range /int = RANGE_2G:
    if not RATE_3_125HZ <= rate <= RATE_100HZ: throw "INVALID_RATE"
    if not RANGE_2G <= range <= RANGE_12G: throw "INVALID_RANGE"

    // Section 8.21.
    // Enable the temperature sensor, and apply the rate.
    // The temperature sensor is disabled by default, but I can't see
    //   any reason why one would want to disable it.
    // Also set the high-resolution bit for the magnetometer.
    ctrl5 := 0b1110_0000
    // Apply the rate.
    ctrl5 |= rate << 2
    reg_.write_u8 CTRL5_ ctrl5

    // Section 8.22.
    // Set the range.
    ctrl6 := range << 5
    range_ = range
    reg_.write_u8 CTRL6_ ctrl6

    // Section 8.23. Table 54.
    // High-pass filter. Default 0.
    // Filtered acceleration data selection. Default 0.
    // Temperature sensor only. Default 0. (Temperature needs magnetometer to be active).
    // Low-power data mode. Default 0.
    // Set to continuous-conversion mode: 0. Default  0b10.
    ctrl7 := 0x0
    reg_.write_u8 CTRL7_ ctrl7

  disable -> none:
    // Section 8.23. Table 54.
    // Power down mode.
    reg_.write_u8 CTRL7_ 0b10

  /**
  Reads the temperature.
  Returns the result in Celsius.
  */
  read_temperature -> float:
    // 7.2.9, Table 86.
    // 12 bit signed integer shifted by 4. In other words: a 16 bit integer with
    //   the least significant 4 bits not used.
    // 8 steps per degree. This means that there are 3 fractional bits.
    // If we just wanted to return an integer temperature value we could
    //   return `value >> 7`.
    // TODO(florian): check that the least 4 significant bits are equal to 0.
    //   Shouldn't matter too much if they aren't.
    low := reg_.read_u8 TEMP_OUT_L_
    high := reg_.read_u8 TEMP_OUT_H_
    // value := reg_.read_i16_le TEMP_OUT_L_
    value := (high << 8) | low
    // TODO(florian): see whether the division by 8 is correct.
    // Also: we seem to get very few digits.
    return value * (1.0 / 8.0) + 25.0  // Let the compiler constant-fold the division.

  /**
  Reads the magnetic field.
  The returned values are in microtesla.
  If a value is out of range, +-$float.INFINITY is used. In this case
    changing the range (see $enable) might be an option to allow the
    sensor to measure the magnetic field.
  */
  read -> math.Point3f:
    AUTO_INCREMENT_BIT ::= 0b1000_0000
    x := reg_.read_i16_le (OUT_X_L_M_ | AUTO_INCREMENT_BIT)
    z := reg_.read_i16_le (OUT_Z_L_M_ | AUTO_INCREMENT_BIT)
    y := reg_.read_i16_le (OUT_Y_L_M_ | AUTO_INCREMENT_BIT)

    gain := ?
    if range_ == RANGE_2G: gain = 0.080
    else if range_ == RANGE_4G: gain = 0.160
    else if range_ == RANGE_8G: gain = 0.320
    else:
      assert: range_ == RANGE_12G
      // Note that this is not a multiple of 0.080 which would have made
      // things easier.
      gain = 0.479

    x_calibrated := x - calibration_[0]
    y_calibrated := y - calibration_[1]
    z_calibrated := z - calibration_[2]

    x_converted := x_calibrated * gain * (GAUSS_TO_MICROTESLA_ / 1000.0)
    y_converted := y_calibrated * gain * (GAUSS_TO_MICROTESLA_ / 1000.0)
    z_converted := z_calibrated * gain * (GAUSS_TO_MICROTESLA_ / 1000.0)

    // Check for saturation.
    if not -(0x7FF0) < x < 0x7FF0: x_converted = x.sign * float.INFINITY
    if not -(0x7FF0) < y < 0x7FF0: y_converted = y.sign * float.INFINITY
    if not -(0x7FF0) < z < 0x7FF0: z_converted = z.sign * float.INFINITY

    return math.Point3f
        x_converted
        y_converted
        z_converted

  read_range -> int:
    // Section 8.22.
    // Read the range.
    ctrl6 := reg_.read_u8 CTRL6_
    // The bit-and shouldn't be necessary, but doesn't hurt.
    return (ctrl6 >> 5) & 0b11

  /**
  Reads the raw magnetic field values.
  These can be used for calibration.
  */
  read --raw/bool -> List:
    if not raw: throw "INVALID_ARGUMENT"

    AUTO_INCREMENT_BIT ::= 0b1000_0000
    x := reg_.read_i16_le (OUT_X_H_M_ | AUTO_INCREMENT_BIT)
    z := reg_.read_i16_le (OUT_Z_H_M_ | AUTO_INCREMENT_BIT)
    y := reg_.read_i16_le (OUT_Y_H_M_ | AUTO_INCREMENT_BIT)

    return [x, y, z]
