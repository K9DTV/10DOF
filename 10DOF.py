# Pico 2: MPU + HMC5883L + FXPQ3115 -> SSD1306
# SDA=GP0 SCL=GP1 | 0x68  0x1E  0x60  OLED 0x3C
# temp top-right, Pa bottom-left, mag bottom-right

from machine import I2C, Pin
import ssd1306
import struct, math, time

i2c = I2C(0, sda=Pin(0), scl=Pin(1), freq=400_000)
oled = ssd1306.SSD1306_I2C(128, 64, i2c, addr=0x3C)
MPU, HMC, BARO = 0x68, 0x1E, 0x60
ACCEL_LSB, GYRO_LSB = 16384.0, 131.0

def mw(r, v):
    i2c.writeto_mem(MPU, r, bytes([v]))
def hw(r, v):
    i2c.writeto_mem(HMC, r, bytes([v]))

def line(oled, x0, y0, x1, y1, c=1):
    dx = abs(x1 - x0); sx = 1 if x0 < x1 else -1
    dy = -abs(y1 - y0); sy = 1 if y0 < y1 else -1
    err = dx + dy
    while True:
        if 0 <= x0 < 128 and 0 <= y0 < 64:
            oled.pixel(x0, y0, c)
        if x0 == x1 and y0 == y1:
            break
        e2 = 2 * err
        if e2 >= dy:
            err += dy; x0 += sx
        if e2 <= dx:
            err += dx; y0 += sy

def read_baro():
    b = i2c.readfrom_mem(BARO, 0x01, 5)
    pa = ((b[0] << 16) | (b[1] << 8) | b[2]) >> 6
    tc = struct.unpack(">h", b[3:5])[0] / 256.0
    return pa, tc

mw(0x6B, 0x80); time.sleep_ms(100)
mw(0x6B, 0x01); time.sleep_ms(50)
mw(0x6C, 0x00); mw(0x1A, 0x03); mw(0x1B, 0x00); mw(0x1C, 0x00)
hw(0x00, 0x70); hw(0x01, 0x20); hw(0x02, 0x00)
i2c.writeto_mem(BARO, 0x26, b"\x38")
time.sleep_ms(10)
i2c.writeto_mem(BARO, 0x26, b"\x39")
time.sleep_ms(200)

print("hold still…")
sx = sy = sz = 0.0
for _ in range(200):
    *_, gx, gy, gz = struct.unpack(">hhhhhhh", i2c.readfrom_mem(MPU, 0x3B, 14))
    sx += gx; sy += gy; sz += gz
    time.sleep_ms(5)
bx, by, bz = sx/200/GYRO_LSB, sy/200/GYRO_LSB, sz/200/GYRO_LSB
p0 = 0
for _ in range(10):
    p0, _ = read_baro()
    if p0:
        break
    time.sleep_ms(100)
print("go", p0)

V = [(-1,-1,-1),(1,-1,-1),(1,1,-1),(-1,1,-1),
     (-1,-1,1),(1,-1,1),(1,1,1),(-1,1,1)]
EDGES = [(0,1),(1,2),(2,3),(3,0),(4,5),(5,6),(6,7),(7,4),(0,4),(1,5),(2,6),(3,7)]

def rot(x, y, z, yaw, pitch, roll):
    cy, sy_ = math.cos(yaw), math.sin(yaw)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cr, sr = math.cos(roll), math.sin(roll)
    x, y = x*cy - y*sy_, x*sy_ + y*cy
    y, z = y*cp - z*sp, y*sp + z*cp
    x, z = x*cr + z*sr, -x*sr + z*cr
    return x, y, z

roll = pitch = yaw = 0.0
tprev = time.ticks_ms()

while True:
    now = time.ticks_ms()
    dt = time.ticks_diff(now, tprev) / 1000.0
    tprev = now
    if dt <= 0 or dt > 0.2:
        dt = 0.05

    ax, ay, az, _, gx, gy, gz = struct.unpack(">hhhhhhh", i2c.readfrom_mem(MPU, 0x3B, 14))
    amag = math.sqrt(ax*ax + ay*ay + az*az) / ACCEL_LSB
    gx = math.radians(gx/GYRO_LSB - bx)
    gy = math.radians(gy/GYRO_LSB - by)
    gz = math.radians(gz/GYRO_LSB - bz)
    mx, mz, my = struct.unpack(">hhh", i2c.readfrom_mem(HMC, 0x03, 6))
    pascals, temp_c = read_baro()

    roll += gx * dt
    pitch += gy * dt
    yaw += gz * dt
    roll = 0.98*roll + 0.02*math.atan2(ay, az)
    pitch = 0.98*pitch + 0.02*math.atan2(-ax, math.sqrt(ay*ay + az*az))
    sp, cp = math.sin(pitch), math.cos(pitch)
    sr, cr = math.sin(roll), math.cos(roll)
    mxh = mx*cp + mz*sp
    myh = mx*sr*sp + my*cr - mz*sr*cp
    yaw_m = math.atan2(myh, mxh)
    yaw += 0.05 * ((yaw_m - yaw + math.pi) % (2*math.pi) - math.pi)

    s = 14 + (amag - 1) * 8 + (pascals - p0) * 1.0
    if s < 6:
        s = 6
    if s > 30:
        s = 30
    pts = []
    for x, y, z in V:
        xr, yr, zr = rot(x, y, z, yaw, pitch, roll)
        pts.append((int(64 + xr*s), int(28 - zr*s)))

    oled.fill(0)
    for a, b in EDGES:
        line(oled, pts[a][0], pts[a][1], pts[b][0], pts[b][1], 1)
    oled.text("{:.0f}F".format(temp_c * 9 / 5 + 32), 98, 0, 1)
    oled.text("{:d}Pa".format(pascals), 0, 56, 1)
    oled.text("{:3.0f}".format(math.degrees(yaw) % 360), 104, 56, 1)
    oled.show()