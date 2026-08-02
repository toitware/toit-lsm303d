// Copyright (C) 2026 Toit contributors.
// Use of this source code is governed by an MIT-style license that can be found
// in the LICENSE file.

import expect show *
import io
import math
import serial.device show Device
import serial.registers show Registers

import lsm303d show Lsm303d
import lsm303d.accelerometer show Accelerometer
import lsm303d.magnetometer show Magnetometer

WHO-AM-I ::= 0x0f
CTRL1 ::= 0x20
OUT-X-L-M ::= 0x08
OUT-Y-L-M ::= 0x0a
OUT-Z-L-M ::= 0x0c
OUT-X-L-A ::= 0x28
OUT-Y-L-A ::= 0x2a
OUT-Z-L-A ::= 0x2c
TEMP-OUT-L ::= 0x05

main:
  test-cross-product
  test-raw-magnetometer-read
  test-bdu
  test-temperature
  test-calibration

test-cross-product:
  registers := test-registers
  registers.set-i16 OUT-X-L-A 0
  registers.set-i16 OUT-Y-L-A 0
  registers.set-i16 OUT-Z-L-A 16_384
  registers.set-i16 OUT-X-L-M 1_000
  registers.set-i16 OUT-Y-L-M 1_000
  registers.set-i16 OUT-Z-L-M 0

  sensor := Lsm303d (FakeDevice registers)
  sensor.enable
  heading := sensor.heading (math.Point3f 1 0 0)
  expect-close 45.0 heading

test-raw-magnetometer-read:
  registers := test-registers
  magnetometer := Magnetometer (FakeDevice registers)
  registers.set-i16 OUT-X-L-M 0x1234
  registers.set-i16 OUT-Y-L-M -1_234
  registers.set-i16 OUT-Z-L-M 0x2345

  expect-list-equals [0x1234, -1_234, 0x2345]
    magnetometer.read --raw

test-bdu:
  registers := test-registers
  accelerometer := Accelerometer (FakeDevice registers)
  accelerometer.enable
  expect-equals 0x6f registers[CTRL1]

  // Magnetometer-only use also enables BDU while preserving CTRL1.
  registers[CTRL1] = 0x67
  magnetometer := Magnetometer (FakeDevice registers)
  magnetometer.enable
  expect-equals 0x6f registers[CTRL1]

  accelerometer.disable
  expect-equals 0x0f registers[CTRL1]

test-temperature:
  registers := test-registers
  magnetometer := Magnetometer (FakeDevice registers)

  // +8 LSB is one degree above the nominal 25 °C reference.
  registers[TEMP-OUT-L] = 0x08
  registers[TEMP-OUT-L + 1] = 0x00
  expect-close 26.0 magnetometer.read-temperature

  // 0xff8 is -8 in right-justified 12-bit two's complement.
  registers[TEMP-OUT-L] = 0xf8
  registers[TEMP-OUT-L + 1] = 0x0f
  expect-close 24.0 magnetometer.read-temperature

test-calibration:
  registers := test-registers
  calibration := [-1_000, -500, -2_000, 1_000, 500, 2_000]
  magnetometer := Magnetometer (FakeDevice registers) --calibration=calibration

  // These values are at the same relative position on all three axes.
  registers.set-i16 OUT-X-L-M 500
  registers.set-i16 OUT-Y-L-M 250
  registers.set-i16 OUT-Z-L-M 1_000
  field := magnetometer.read
  expect-close field.x field.y
  expect-close field.x field.z

  // Existing three-offset calibrations remain supported.
  old-calibration := Magnetometer (FakeDevice registers) --calibration=[100, 200, 300]
  registers.set-i16 OUT-X-L-M 200
  registers.set-i16 OUT-Y-L-M 300
  registers.set-i16 OUT-Z-L-M 400
  old-field := old-calibration.read
  expect-close old-field.x old-field.y
  expect-close old-field.x old-field.z

expect-close expected/num actual/num:
  expect (expected - actual).abs < 0.000_001

test-registers -> FakeRegisters:
  result := FakeRegisters
  result[WHO-AM-I] = 0x49
  return result

class FakeDevice implements Device:
  registers_/Registers

  constructor .registers_:

  registers -> Registers:
    return registers_

  read amount/int -> ByteArray:
    throw "UNIMPLEMENTED"

  write bytes/ByteArray -> none:
    throw "UNIMPLEMENTED"

class FakeRegisters extends Registers:
  bytes_/ByteArray := ByteArray 0x80

  read-bytes register/int count/int -> ByteArray:
    address := register & 0x7f
    result := ByteArray count
    count.repeat:
      result[it] = bytes_[address + it]
    return result

  write-bytes register/int data/ByteArray -> none:
    address := register & 0x7f
    data.size.repeat:
      bytes_[address + it] = data[it]

  operator [] index/int -> int:
    return bytes_[index]

  operator []= index/int value/int -> none:
    bytes_[index] = value

  set-i16 register/int value/int -> none:
    io.LITTLE-ENDIAN.put-int16 bytes_ register value
