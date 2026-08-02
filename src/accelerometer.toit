// Copyright (C) 2021 Toitware ApS. All rights reserved.
// Use of this source code is governed by an MIT-style license that can be found
// in the LICENSE file.

import serial.device as serial
import serial.registers as serial
import io
import math

/**
Driver for the accelerometer of the LSM303D module.
*/
class Accelerometer:
  static I2C-ADDRESS ::= 0b11101  // 6.1.1.
  static I2C-ADDRESS-ALT ::= 0b11110

  // Sampling Rates.
  // Section 8.17. Table 36.
  static RATE-3-125HZ ::= 1
  static RATE-6-25HZ  ::= 2
  static RATE-12-5HZ  ::= 3
  static RATE-25HZ    ::= 4
  static RATE-50HZ    ::= 5
  static RATE-100HZ   ::= 6
  static RATE-200HZ   ::= 7
  static RATE-400HZ   ::= 8
  static RATE-800HZ   ::= 9
  static RATE-1600HZ  ::= 10

  // Ranges.
  // Section 8.18. Table 40.
  static RANGE-2G  ::= 0
  static RANGE-4G  ::= 1
  static RANGE-6G  ::= 2
  static RANGE-8G  ::= 3
  static RANGE-16G ::= 4

  // Section 7. Table 16. Register mapping.
  static WHO-AM-I_ ::= 0x0F
  static CTRL1_ ::= 0x20
  static CTRL2_ ::= 0x21
  static OUT-X-L-A_ ::= 0x28
  static OUT-X-H-A_ ::= 0x29
  static OUT-Y-L-A_ ::= 0x2A
  static OUT-Y-H-A_ ::= 0x2B
  static OUT-Z-L-A_ ::= 0x2C
  static OUT-Z-H-A_ ::= 0x2D

  static BDU-BIT_ ::= 1 << 3
  static AUTO-INCREMENT-BIT_ ::= 1 << 7

  /**
  Standard acceleration due to gravity.
  In m/s².
  */
  static GRAVITY-STANDARD_ ::= 9.80665

  reg_ /serial.Registers
  range_ /int := 0

  constructor dev/serial.Device:
    reg_ = dev.registers

    id := reg_.read-u8 WHO-AM-I_
    // Section 8.6, Table 19.
    if id != 0x49: throw "INVALID_CHIP"


  /**
  Enables the sensor.

  The $rate parameter defines the frequency at which measurements are taken.
  Valid values for $rate are:
  - $RATE-3-125HZ
  - $RATE-6-25HZ
  - $RATE-12-5HZ
  - $RATE-25HZ
  - $RATE-50HZ
  - $RATE-100HZ
  - $RATE-200HZ
  - $RATE-400HZ
  - $RATE-800HZ
  - $RATE-1600HZ

  The $range parameter defines the measured acceleration range.
  Valid values for $range are:
  - $RANGE-2G: +-2G (19.61 m/s²)
  - $RANGE-4G: +-4G (39.23 m/s²)
  - $RANGE-6G: +-6G (58.84 m/s²)
  - $RANGE-8G: +-8G (78.45 m/s²)
  - $RANGE-16G: +-16G (156.9 m/s²)
  */
  enable -> none
      --rate  /int = RATE-100HZ
      --range /int = RANGE-2G:

    if not RATE-3-125HZ <= rate <= RATE-1600HZ: throw "INVALID_RANGE"
    // 8.17. CTRL1.
    rate-bits := rate << 4

    // We always enable all three axes.
    axes-bits := 0b111

    ctrl1 := rate-bits | axes-bits
    // Prevent an update while output bytes are being read.
    ctrl1 |= BDU-BIT_

    // 8.18. CTRL2.
    // Anti-alias filter bandwidth set to default (0).
    // Acceleration scaling (range).
    // Acceleration self-test: disabled. (0)
    // SPI disabled. (0)
    if not 0 <= range <= 4: throw "INVALID_RANGE"
    range_ = range
    ctrl2 := range << 3

    reg_.write-u8 CTRL1_ ctrl1
    reg_.write-u8 CTRL2_ ctrl2

    sleep --ms=10


  /**
  Disables the accelerometer.
  Initiates a power-down of the peripheral. It is safe to call $enable
    to restart the accelerometer.
  */
  disable:
    // Fundamentally we only care for the rate-bits: as long as they
    // are 0, the device is disabled.
    // Keep BDU enabled because it also applies to magnetic data.
    reg_.write-u8 CTRL1_ (BDU-BIT_ | 0b111)

  /**
  Reads the x, y and z axis.
  The returned values are in in m/s².
  */
  read -> math.Point3f:
    raw := read-raw_
    x := raw[0]
    y := raw[1]
    z := raw[2]

    // Section 2.1, table3:
    // The linear acceleration sensitivity depends on the range:
    // - RANGE_2G:   0.061mg/LSB
    // - RANGE_4G:   0.122mg/LSB
    // - RANGE_6G:   0.183mg/LSB
    // - RANGE_8G:   0.244mg/LSB
    // - RANGE_16G:  0.732mg/LSB   // <- Note that the 16G sensitivity is not 0.488mg/LSB as expected.
    SENSITIVITIES ::= #[1, 2, 3, 4, 12]  // As factors of 0.061.
    sensitivity := SENSITIVITIES[range_]
    x *= sensitivity
    y *= sensitivity
    z *= sensitivity

    factor := GRAVITY-STANDARD_ * 0.061 / 1000.0  // Constant folded because it's one expression.
    return math.Point3f
        x * factor
        y * factor
        z * factor

  read --raw/bool -> List:
    if not raw: throw "INVALID_ARGUMENT"

    return read-raw_

  /**
  Reads the current range setting of the sensor.
  Returns $RANGE-2G, $RANGE-4G, $RANGE-8G or $RANGE-16G.
  */
  read-range -> int:
    reg4 := reg_.read-u8 CTRL2_
    return (reg4 >> 3) & 0b111

  read-raw_ -> List:
    bytes := reg_.read-bytes (OUT-X-L-A_ | AUTO-INCREMENT-BIT_) 6
    return [
      io.LITTLE-ENDIAN.int16 bytes 0,
      io.LITTLE-ENDIAN.int16 bytes 2,
      io.LITTLE-ENDIAN.int16 bytes 4,
    ]
