// Copyright (C) 2021 Toitware ApS. All rights reserved.
// Use of this source code is governed by an MIT-style license that can be found
// in the LICENSE file.

import serial.device as serial
import serial.registers as serial
import math

/**
Driver for the accelerometer of the LSM303D module.
*/
class Accelerometer:
  static I2C_ADDRESS ::= 0b11101  // 6.1.1.
  static I2C_ADDRESS_ALT ::= 0b11110

  // Sampling Rates.
  // Section 8.17. Table 36.
  static RATE_3_125HZ ::= 1
  static RATE_6_25HZ  ::= 2
  static RATE_12_5HZ  ::= 3
  static RATE_25HZ    ::= 4
  static RATE_50HZ    ::= 5
  static RATE_100HZ   ::= 6
  static RATE_200HZ   ::= 7
  static RATE_400HZ   ::= 8
  static RATE_800HZ   ::= 9
  static RATE_1600HZ  ::= 10

  // Ranges.
  // Section 8.18. Table 40.
  static RANGE_2G  ::= 0
  static RANGE_4G  ::= 1
  static RANGE_6G  ::= 2
  static RANGE_8G  ::= 3
  static RANGE_16G ::= 4

  // Section 7. Table 16. Register mapping.
  static WHO_AM_I_ ::= 0x0F
  static CTRL1_ ::= 0x20
  static CTRL2_ ::= 0x21
  static OUT_X_L_A_ ::= 0x28
  static OUT_X_H_A_ ::= 0x29
  static OUT_Y_L_A_ ::= 0x2A
  static OUT_Y_H_A_ ::= 0x2B
  static OUT_Z_L_A_ ::= 0x2C
  static OUT_Z_H_A_ ::= 0x2D

  /**
  Standard acceleration due to gravity.
  In m/s².
  */
  static GRAVITY_STANDARD_ ::= 9.80665

  reg_ /serial.Registers
  range_ /int := 0

  constructor dev/serial.Device:
    reg_ = dev.registers

    id := reg_.read_u8 WHO_AM_I_
    // Section 8.6, Table 19.
    if id != 0x49: throw "INVALID_CHIP"


  /**
  Enables the sensor.

  The $rate parameter defines the frequency at which measurements are taken.
  Valid values for $rate are:
  - $RATE_3_125HZ
  - $RATE_6_25HZ
  - $RATE_12_5HZ
  - $RATE_25HZ
  - $RATE_50HZ
  - $RATE_100HZ
  - $RATE_200HZ
  - $RATE_400HZ
  - $RATE_800HZ
  - $RATE_1600HZ

  The $range parameter defines the measured acceleration range.
  Valid values for $range are:
  - $RANGE_2G: +-2G (19.61 m/s²)
  - $RANGE_4G: +-4G (39.23 m/s²)
  - $RANGE_6G: +-6G (58.84 m/s²)
  - $RANGE_8G: +-8G (78.45 m/s²)
  - $RANGE_16G: +-16G (156.9 m/s²)
  */
  enable -> none
      --rate  /int = RATE_100HZ
      --range /int = RANGE_2G:

    if not RATE_3_125HZ <= rate <= RATE_1600HZ: throw "INVALID_RANGE"
    // 8.17. CTRL1.
    rate_bits := rate << 4

    // We always enable all three axes.
    axes_bits := 0b111

    ctrl1 := rate_bits | axes_bits

    // 8.18. CTRL2.
    // Anti-alias filter bandwith set to default (0).
    // Acceleration scaling (range).
    // Acceleration self-test: disabled. (0)
    // SPI disabled. (0)
    if not 0 <= range <= 4: throw "INVALID_RANGE"
    range_ = range
    ctrl2 := range << 3

    reg_.write_u8 CTRL1_ ctrl1
    reg_.write_u8 CTRL2_ ctrl2

    sleep --ms=10


  /**
  Disables the accelerometer.
  Initiates a power-down of the peripheral. It is safe to call $enable
    to restart the accelerometer.
  */
  disable:
    // Fundamentally we only care for the rate-bits: as long as they
    // are 0, the device is disabled.
    // It's safe to change the other bits as well.
    reg_.write_u8 CTRL1_ 0x00

  /**
  Reads the x, y and z axis.
  The returned values are in in m/s².
  */
  read -> math.Point3f:
    AUTO_INCREMENT_BIT ::= 0b1000_0000
    x := reg_.read_i16_le (OUT_X_L_A_ | AUTO_INCREMENT_BIT)
    y := reg_.read_i16_le (OUT_Y_L_A_ | AUTO_INCREMENT_BIT)
    z := reg_.read_i16_le (OUT_Z_L_A_ | AUTO_INCREMENT_BIT)

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

    factor := GRAVITY_STANDARD_ * 0.061 / 1000.0  // Constant folded because it's one expression.
    return math.Point3f
        x * factor
        y * factor
        z * factor

  read --raw/bool -> List:
    if not raw: throw "INVALID_ARGUMENT"

    AUTO_INCREMENT_BIT ::= 0b1000_0000
    x := reg_.read_i16_le (OUT_X_L_A_ | AUTO_INCREMENT_BIT)
    y := reg_.read_i16_le (OUT_Y_L_A_ | AUTO_INCREMENT_BIT)
    z := reg_.read_i16_le (OUT_Z_L_A_ | AUTO_INCREMENT_BIT)

    return [x, y, z]

  /**
  Reads the current range setting of the sensor.
  Returns $RANGE_2G, $RANGE_4G, $RANGE_8G or $RANGE_16G.
  */
  read_range -> int:
    reg4 := reg_.read_u8 CTRL2_
    return (reg4 >> 3) & 0b111

