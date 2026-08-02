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
  static I2C-ADDRESS ::= 0b11101  // 6.1.1.
  static I2C-ADDRESS-ALT ::= 0b11110

  // Section 7. Table 16. Register mapping.
  static WHO-AM-I_ ::= 0x0F
  static TEMP-OUT-L_ ::= 0x05
  static TEMP-OUT-H_ ::= 0x06
  static OUT-X-L-M_ ::= 0x08
  static OUT-X-H-M_ ::= 0x09
  static OUT-Y-L-M_ ::= 0x0A
  static OUT-Y-H-M_ ::= 0x0B
  static OUT-Z-L-M_ ::= 0x0C
  static OUT-Z-H-M_ ::= 0x0D
  static CTRL1_ ::= 0x20
  static CTRL5_ ::= 0x24
  static CTRL6_ ::= 0x25
  static CTRL7_ ::= 0x26

  static BDU-BIT_ ::= 1 << 3
  static AUTO-INCREMENT-BIT_ ::= 1 << 7

  // Section 8.21, Table 47.
  static RATE-3-125HZ ::= 0
  static RATE-6-25HZ  ::= 1
  static RATE-12-5HZ  ::= 2
  static RATE-25HZ    ::= 3
  static RATE-50HZ    ::= 4
  /** Only available if the accelerometer has a rate of more than 50Hz. */
  static RATE-100HZ   ::= 5

  // Section 8.22. Table 50.
  // "G" refers to "Gauss".
  static RANGE-2G  ::= 0
  static RANGE-4G  ::= 1
  static RANGE-8G  ::= 2
  static RANGE-12G ::= 3

  static GAUSS-TO-MICROTESLA_ ::= 100.0

  reg_ /serial.Registers
  calibration-offsets_ /List := [0, 0, 0]
  calibration-scales_ /List := [1.0, 1.0, 1.0]
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

    set-calibration_ calibration

    id := reg_.read-u8 WHO-AM-I_
    // Section 8.6, Table 19.
    if id != 0x49: throw "INVALID_CHIP"

  enable -> none
      --rate  /int = RATE-6-25HZ
      --range /int = RANGE-2G:
    if not RATE-3-125HZ <= rate <= RATE-100HZ: throw "INVALID_RATE"
    if not RANGE-2G <= range <= RANGE-12G: throw "INVALID_RANGE"

    // Section 8.21.
    // Enable the temperature sensor, and apply the rate.
    // The temperature sensor is disabled by default, but I can't see
    //   any reason why one would want to disable it.
    // Also set the high-resolution bit for the magnetometer.
    ctrl5 := 0b1110_0000
    // Apply the rate.
    ctrl5 |= rate << 2
    reg_.write-u8 CTRL5_ ctrl5

    // Section 8.22.
    // Set the range.
    ctrl6 := range << 5
    range_ = range
    reg_.write-u8 CTRL6_ ctrl6

    // BDU is in CTRL1 and applies to both acceleration and magnetic data.
    ctrl1 := reg_.read-u8 CTRL1_
    reg_.write-u8 CTRL1_ (ctrl1 | BDU-BIT_)

    // Section 8.23. Table 54.
    // High-pass filter. Default 0.
    // Filtered acceleration data selection. Default 0.
    // Temperature sensor only. Default 0. (Temperature needs magnetometer to be active).
    // Low-power data mode. Default 0.
    // Set to continuous-conversion mode: 0. Default  0b10.
    ctrl7 := 0x0
    reg_.write-u8 CTRL7_ ctrl7

  disable -> none:
    // Section 8.23. Table 54.
    // Power down mode.
    reg_.write-u8 CTRL7_ 0b10

  /**
  Reads the temperature.
  Returns the result in Celsius.
  */
  read-temperature -> float:
    // Section 4.2.
    // Unlike the LSM303DLHC, the LSM303D stores temperature right-justified.
    // The value is a right-justified, 12-bit two's complement integer.
    // 8 steps per degree. This means that there are 3 fractional bits.
    // If we just wanted to return an integer temperature value we could
    //   return `value >> 3`.
    bytes := reg_.read-bytes (TEMP-OUT-L_ | AUTO-INCREMENT-BIT_) 2
    value := io.LITTLE-ENDIAN.uint16 bytes 0
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
    raw := read-raw_
    x := raw[0]
    y := raw[1]
    z := raw[2]

    gain := ?
    if range_ == RANGE-2G: gain = 0.080
    else if range_ == RANGE-4G: gain = 0.160
    else if range_ == RANGE-8G: gain = 0.320
    else:
      assert: range_ == RANGE-12G
      // Note that this is not a multiple of 0.080 which would have made
      // things easier.
      gain = 0.479

    x-calibrated := (x - calibration-offsets_[0]) * calibration-scales_[0]
    y-calibrated := (y - calibration-offsets_[1]) * calibration-scales_[1]
    z-calibrated := (z - calibration-offsets_[2]) * calibration-scales_[2]

    x-converted := x-calibrated * gain * (GAUSS-TO-MICROTESLA_ / 1000.0)
    y-converted := y-calibrated * gain * (GAUSS-TO-MICROTESLA_ / 1000.0)
    z-converted := z-calibrated * gain * (GAUSS-TO-MICROTESLA_ / 1000.0)

    // Check for saturation.
    if not -(0x7FF0) < x < 0x7FF0: x-converted = x.sign * float.INFINITY
    if not -(0x7FF0) < y < 0x7FF0: y-converted = y.sign * float.INFINITY
    if not -(0x7FF0) < z < 0x7FF0: z-converted = z.sign * float.INFINITY

    return math.Point3f
        x-converted
        y-converted
        z-converted

  read-range -> int:
    // Section 8.22.
    // Read the range.
    ctrl6 := reg_.read-u8 CTRL6_
    // The bit-and shouldn't be necessary, but doesn't hurt.
    return (ctrl6 >> 5) & 0b11

  /**
  Reads the raw magnetic field values.
  These can be used for calibration.
  */
  read --raw/bool -> List:
    if not raw: throw "INVALID_ARGUMENT"

    return read-raw_

  read-raw_ -> List:
    bytes := reg_.read-bytes (OUT-X-L-M_ | AUTO-INCREMENT-BIT_) 6
    return [
      io.LITTLE-ENDIAN.int16 bytes 0,
      io.LITTLE-ENDIAN.int16 bytes 2,
      io.LITTLE-ENDIAN.int16 bytes 4,
    ]

  set-calibration_ calibration/List -> none:
    if calibration.size == 3:
      calibration-offsets_ = calibration
      calibration-scales_ = [1.0, 1.0, 1.0]
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

    average-range := (ranges[0] + ranges[1] + ranges[2]) / 3.0
    calibration-offsets_ = [
      (calibration[3] + calibration[0]) / 2.0,
      (calibration[4] + calibration[1]) / 2.0,
      (calibration[5] + calibration[2]) / 2.0,
    ]
    calibration-scales_ = [
      average-range / ranges[0],
      average-range / ranges[1],
      average-range / ranges[2],
    ]
