// Copyright (C) 2021 Toitware ApS. All rights reserved.
// Use of this source code is governed by an MIT-style license that can be found
// in the LICENSE file.

import serial.device as serial
import serial.registers as serial
import io
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
  static CTRL1_ ::= 0x20
  static CTRL5_ ::= 0x24
  static CTRL6_ ::= 0x25
  static CTRL7_ ::= 0x26

  static BDU_BIT_ ::= 1 << 3
  static AUTO_INCREMENT_BIT_ ::= 1 << 7

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
  calibration_offsets_ /List := [0, 0, 0]
  calibration_scales_ /List := [1.0, 1.0, 1.0]
  range_ /int := 0


  /**
  Constructs a new Magnetometer.

  The $calibration may be a 6-element list containing the minimum values for
    X, Y, and Z followed by the maximum values for X, Y, and Z. These values
    correct both the offset and relative scale of each axis. Typically, the
    user moves the sensor in a figure 8 while the calibration program collects
    all seen values.

  For backwards compatibility, a 3-element list is interpreted as an offset
    for each axis.
  */
  constructor dev/serial.Device --calibration=[0, 0, 0]:
    reg_ = dev.registers

    set_calibration_ calibration

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

    // BDU is in CTRL1 and applies to both acceleration and magnetic data.
    ctrl1 := reg_.read_u8 CTRL1_
    reg_.write_u8 CTRL1_ (ctrl1 | BDU_BIT_)

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
    // Section 4.2.
    // Unlike the LSM303DLHC, the LSM303D stores temperature right-justified.
    // The value is a right-justified, 12-bit two's complement integer.
    // 8 steps per degree. This means that there are 3 fractional bits.
    // If we just wanted to return an integer temperature value we could
    //   return `value >> 3`.
    bytes := reg_.read_bytes (TEMP_OUT_L_ | AUTO_INCREMENT_BIT_) 2
    value := io.LITTLE_ENDIAN.uint16 bytes 0
    value &= 0x0fff
    if value & 0x0800 != 0: value -= 0x1000
    return value * (1.0 / 8.0) + 25.0  // Let the compiler constant-fold the division.

  /**
  Reads the magnetic field.
  The returned values are in microtesla.
  If a value is out of range, +-$float.INFINITY is used. In this case
    changing the range (see $enable) might be an option to allow the
    sensor to measure the magnetic field.
  */
  read -> math.Point3f:
    raw := read_raw_
    x := raw[0]
    y := raw[1]
    z := raw[2]

    gain := ?
    if range_ == RANGE_2G: gain = 0.080
    else if range_ == RANGE_4G: gain = 0.160
    else if range_ == RANGE_8G: gain = 0.320
    else:
      assert: range_ == RANGE_12G
      // Note that this is not a multiple of 0.080 which would have made
      // things easier.
      gain = 0.479

    x_calibrated := (x - calibration_offsets_[0]) * calibration_scales_[0]
    y_calibrated := (y - calibration_offsets_[1]) * calibration_scales_[1]
    z_calibrated := (z - calibration_offsets_[2]) * calibration_scales_[2]

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

    return read_raw_

  read_raw_ -> List:
    bytes := reg_.read_bytes (OUT_X_L_M_ | AUTO_INCREMENT_BIT_) 6
    return [
      io.LITTLE_ENDIAN.int16 bytes 0,
      io.LITTLE_ENDIAN.int16 bytes 2,
      io.LITTLE_ENDIAN.int16 bytes 4,
    ]

  set_calibration_ calibration/List -> none:
    if calibration.size == 3:
      calibration_offsets_ = calibration
      calibration_scales_ = [1.0, 1.0, 1.0]
      return

    if calibration.size != 6: throw "INVALID_CALIBRATION"
    calibration.do:
      if it is not num: throw "INVALID_CALIBRATION"

    ranges := [
      calibration[3] - calibration[0],
      calibration[4] - calibration[1],
      calibration[5] - calibration[2],
    ]
    ranges.do:
      if it <= 0: throw "INVALID_CALIBRATION"

    average_range := (ranges[0] + ranges[1] + ranges[2]) / 3.0
    calibration_offsets_ = [
      (calibration[3] + calibration[0]) / 2.0,
      (calibration[4] + calibration[1]) / 2.0,
      (calibration[5] + calibration[2]) / 2.0,
    ]
    calibration_scales_ = [
      average_range / ranges[0],
      average_range / ranges[1],
      average_range / ranges[2],
    ]
