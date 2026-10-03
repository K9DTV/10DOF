# Pico 2 + China "MPU-9250" / mpu-92.65 clone (WHO_AM_I often 0x75)
# Accel + gyro + temp only. Mag usually absent on these boards.

from machine import I2C, Pin
import struct
import time
import math

SDA, SCL = 0, 1          # GP0 / GP1 — change to match your wiring
ADDR = 0x68
I2C_ID = 0
FREQ = 100_000

# Full-scale: ±2g, ±250 dps (matches default after reset)
ACCEL_LSB = 16384.0      # LSB/g @ ±2g
GYRO_LSB = 131.0         # LSB/(°/s) @ ±250 dps
G = 9.80665

REG_SMPLRT_DIV = 0x19
REG_CONFIG = 0x1A
REG_GYRO_CONFIG = 0x1B
REG_ACCEL_CONFIG = 0x1C
REG_ACCEL_CONFIG2 = 0x1D
REG_INT_PIN_CFG = 0x37
REG_ACCEL_XOUT_H = 0x3B
REG_PWR_MGMT_1 = 0x6B
REG_PWR_MGMT_2 = 0x6C
REG_WHO_AM_I = 0x75


class Mpu6dof:
    def __init__(self, i2c, addr=ADDR):
        self.i2c = i2c
        self.addr = addr
        self.gx0 = self.gy0 = self.gz0 = 0.0
        self._init()

    def _w(self, reg, val):
        self.i2c.writeto_mem(self.addr, reg, bytes([val & 0xFF]))

    def _r(self, reg, n=1):
        return self.i2c.readfrom_mem(self.addr, reg, n)

    def whoami(self):
        return self._r(REG_WHO_AM_I)[0]

    def _init(self):
        who = self.whoami()
        # Accept real and common clone IDs
        if who not in (0x70, 0x71, 0x73, 0x75, 0x68):
            print("WARN: unexpected WHO_AM_I", hex(who), "- continuing anyway")

        self._w(REG_PWR_MGMT_1, 0x80)   # reset
        time.sleep_ms(100)
        self._w(REG_PWR_MGMT_1, 0x01)   # clock = PLL gyro X
        time.sleep_ms(50)
        self._w(REG_PWR_MGMT_2, 0x00)   # all axes on
        self._w(REG_CONFIG, 0x03)       # DLPF ~44 Hz
        self._w(REG_SMPLRT_DIV, 0x04)   # ~200 Hz sample if internal 1 kHz
        self._w(REG_GYRO_CONFIG, 0x00)  # ±250 dps
        self._w(REG_ACCEL_CONFIG, 0x00) # ±2g
        try:
            self._w(REG_ACCEL_CONFIG2, 0x03)  # MPU6500-style; ignore if NACK
        except OSError:
            pass
        self._w(REG_INT_PIN_CFG, 0x02)  # bypass (harmless if no mag)
        time.sleep_ms(50)

    def read_raw(self):
        b = self._r(REG_ACCEL_XOUT_H, 14)
        return struct.unpack(">hhhhhhh", b)

    def read(self):
        ax, ay, az, temp_raw, gx, gy, gz = self.read_raw()
        accel = (
            ax / ACCEL_LSB * G,
            ay / ACCEL_LSB * G,
            az / ACCEL_LSB * G,
        )
        gyro = (
            gx / GYRO_LSB - self.gx0,
            gy / GYRO_LSB - self.gy0,
            gz / GYRO_LSB - self.gz0,
        )
        temp_c = (temp_raw / 333.87) + 21.0  # MPU6500-ish scale
        return accel, gyro, temp_c

    def calibrate_gyro(self, samples=200, delay_ms=5):
        print("Hold still — gyro bias…")
        sx = sy = sz = 0.0
        for _ in range(samples):
            _, _, _, _, gx, gy, gz = self.read_raw()
            sx += gx / GYRO_LSB
            sy += gy / GYRO_LSB
            sz += gz / GYRO_LSB
            time.sleep_ms(delay_ms)
        self.gx0, self.gy0, self.gz0 = sx / samples, sy / samples, sz / samples
        print("gyro bias °/s: {:.2f}, {:.2f}, {:.2f}".format(self.gx0, self.gy0, self.gz0))


def pitch_roll(ax, ay, az):
    # degrees; works when mostly still
    roll = math.degrees(math.atan2(ay, az))
    pitch = math.degrees(math.atan2(-ax, math.sqrt(ay * ay + az * az)))
    return pitch, roll


def main():
    i2c = I2C(I2C_ID, sda=Pin(SDA), scl=Pin(SCL), freq=FREQ)
    imu = Mpu6dof(i2c)
    print("WHO_AM_I", hex(imu.whoami()), "scan", [hex(a) for a in i2c.scan()])
    imu.calibrate_gyro()

    print("{:>7} {:>7} {:>7} | {:>7} {:>7} {:>7} | {:>6} {:>6} | {:>5}".format(
        "ax", "ay", "az", "gx", "gy", "gz", "pitch", "roll", "°C"))
    while True:
        (ax, ay, az), (gx, gy, gz), t = imu.read()
        pitch, roll = pitch_roll(ax, ay, az)
        print("{:7.2f} {:7.2f} {:7.2f} | {:7.1f} {:7.1f} {:7.1f} | {:6.1f} {:6.1f} | {:5.1f}".format(
            ax, ay, az, gx, gy, gz, pitch, roll, t))
        time.sleep_ms(100)


if __name__ == "__main__":
    main()