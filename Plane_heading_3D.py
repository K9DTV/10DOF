# Pico 2: MPU + HMC5883L + FXPQ3115 -> SSD1306
# gyro+baro first, then mag; if mag cal loaded, yaw starts at 0 from compass

from machine import I2C, Pin
import framebuf
import struct, math, time, json

#import os
#os.remove("mag_cal.json")# Pico 2: MPU + HMC5883L + FXPQ3115 -> SSD1306

SDA, SCL = 0, 1
OLED = 0x3C

def i2c_bus_recover():
    scl = Pin(SCL, Pin.OUT, value=1)
    sda = Pin(SDA, Pin.IN, Pin.PULL_UP)
    for _ in range(9):
        scl.value(0); time.sleep_us(5)
        scl.value(1); time.sleep_us(5)
        if sda.value():
            break
    sda = Pin(SDA, Pin.OUT, value=0)
    time.sleep_us(5); scl.value(1); time.sleep_us(5); sda.value(1)
    time.sleep_ms(20)

def make_i2c():
    return I2C(0, sda=Pin(SDA), scl=Pin(SCL), freq=100_000)

i2c_bus_recover()
i2c = make_i2c()
time.sleep_ms(50)
print("scan", [hex(a) for a in i2c.scan()])

def oled_cmd(c):
    i2c.writeto(OLED, bytes([0x00, c]))

def oled_cmds(seq):
    for c in seq:
        try:
            oled_cmd(c)
        except OSError:
            i2c_bus_recover()
            globals()["i2c"] = make_i2c()
            oled_cmd(c)
        time.sleep_ms(1)

oled_cmds([
    0xAE, 0xD5, 0x80, 0xA8, 0x3F, 0xD3, 0x00, 0x40,
    0x8D, 0x14, 0x20, 0x00, 0xA1, 0xC8, 0xDA, 0x12,
    0x81, 0xCF, 0xD9, 0xF1, 0xDB, 0x40, 0xA4, 0xA6, 0xAF,
])
time.sleep_ms(50)

buf = bytearray(128 * 64 // 8)
fb = framebuf.FrameBuffer(buf, 128, 64, framebuf.MONO_VLSB)

def oled_show():
    try:
        for page in range(8):
            oled_cmd(0xB0 | page)
            oled_cmd(0x00)
            oled_cmd(0x10)
            start = page * 128
            i2c.writeto(OLED, b"\x40" + buf[start:start + 128])
            time.sleep_ms(1)
    except OSError:
        i2c_bus_recover()
        globals()["i2c"] = make_i2c()

def oled_text(s, x, y):
    fb.text(s, x, y, 1)

def oled_fill(c=0):
    fb.fill(c)

def line(x0, y0, x1, y1, c=1):
    dx = abs(x1 - x0); sx = 1 if x0 < x1 else -1
    dy = -abs(y1 - y0); sy = 1 if y0 < y1 else -1
    err = dx + dy
    while True:
        if 0 <= x0 < 128 and 0 <= y0 < 64:
            fb.pixel(x0, y0, c)
        if x0 == x1 and y0 == y1:
            break
        e2 = 2 * err
        if e2 >= dy:
            err += dy; x0 += sx
        if e2 <= dx:
            err += dx; y0 += sy

MPU, HMC, BARO = 0x68, 0x1E, 0x60
ACCEL_LSB, GYRO_LSB = 16384.0, 131.0
HOME_ALT_FT = 450.0
MAG_CAL_PATH = "mag_cal.json"
MAG_CAL_SECS = 25
MAG_MIN_SPAN = 80
YAW0 = 3 * math.pi / 2
MAG_BLEND_MAX = math.radians(12)

def i2c_retry(fn, tries=5):
    global i2c
    last = None
    for _ in range(tries):
        try:
            return fn()
        except OSError as e:
            last = e
            i2c_bus_recover()
            i2c = make_i2c()
            time.sleep_ms(20)
    raise last

def mw(r, v):
    i2c_retry(lambda: i2c.writeto_mem(MPU, r, bytes([v])))

def hw(r, v):
    i2c_retry(lambda: i2c.writeto_mem(HMC, r, bytes([v])))

def read_baro():
    def _():
        b = i2c.readfrom_mem(BARO, 0x01, 5)
        pa = ((b[0] << 16) | (b[1] << 8) | b[2]) >> 6
        tc = struct.unpack(">h", b[3:5])[0] / 256.0
        return pa, tc
    return i2c_retry(_)

def read_mag_raw():
    def _():
        mx, mz, my = struct.unpack(">hhh", i2c.readfrom_mem(HMC, 0x03, 6))
        return mx, my, mz
    return i2c_retry(_)

def read_mpu14():
    return i2c_retry(lambda: struct.unpack(">hhhhhhh", i2c.readfrom_mem(MPU, 0x3B, 14)))

def pa_to_ft(pa, p0):
    if pa <= 0 or p0 <= 0:
        return HOME_ALT_FT
    return HOME_ALT_FT + 145366.45 * (1.0 - (pa / p0) ** 0.190284)

def hud(msg1, msg2="", msg3=""):
    oled_fill(0)
    oled_text(msg1[:16], 0, 0)
    if msg2:
        oled_text(msg2[:16], 0, 20)
    if msg3:
        oled_text(msg3[:16], 0, 40)
    oled_show()

def mag_cal_ok(c):
    try:
        return (
            c["maxx"] - c["minx"] >= MAG_MIN_SPAN
            and c["maxy"] - c["miny"] >= MAG_MIN_SPAN
            and c["maxz"] - c["minz"] >= MAG_MIN_SPAN
        )
    except Exception:
        return False

def load_mag_cal():
    try:
        with open(MAG_CAL_PATH, "r") as f:
            c = json.load(f)
        if mag_cal_ok(c):
            return c
    except Exception:
        pass
    return None

def save_mag_cal(c):
    with open(MAG_CAL_PATH, "w") as f:
        json.dump(c, f)

def run_mag_cal():
    mn = [32767, 32767, 32767]
    mxv = [-32768, -32768, -32768]
    t0 = time.ticks_ms()
    last_draw = 0
    while True:
        try:
            mx, my, mz = read_mag_raw()
        except OSError:
            time.sleep_ms(20)
            continue
        for i, v in enumerate((mx, my, mz)):
            if v < mn[i]:
                mn[i] = v
            if v > mxv[i]:
                mxv[i] = v
        sx = mxv[0] - mn[0]
        sy = mxv[1] - mn[1]
        sz = mxv[2] - mn[2]
        elapsed = time.ticks_diff(time.ticks_ms(), t0) / 1000.0
        ok = sx >= MAG_MIN_SPAN and sy >= MAG_MIN_SPAN and sz >= MAG_MIN_SPAN
        now = time.ticks_ms()
        if time.ticks_diff(now, last_draw) > 400:
            last_draw = now
            left = max(0, int(MAG_CAL_SECS - elapsed))
            hud("MAG CAL %ds" % left, "spin+fig8", "X%d Y%d Z%d" % (sx, sy, sz))
        if ok and elapsed >= MAG_CAL_SECS:
            break
        time.sleep_ms(30)
    while not (
        mxv[0] - mn[0] >= MAG_MIN_SPAN
        and mxv[1] - mn[1] >= MAG_MIN_SPAN
        and mxv[2] - mn[2] >= MAG_MIN_SPAN
    ):
        try:
            mx, my, mz = read_mag_raw()
        except OSError:
            time.sleep_ms(20)
            continue
        for i, v in enumerate((mx, my, mz)):
            if v < mn[i]:
                mn[i] = v
            if v > mxv[i]:
                mxv[i] = v
        hud("need more", "keep moving", "X%d Y%d Z%d" % (mxv[0]-mn[0], mxv[1]-mn[1], mxv[2]-mn[2]))
        time.sleep_ms(50)
    c = {
        "minx": mn[0], "maxx": mxv[0],
        "miny": mn[1], "maxy": mxv[1],
        "minz": mn[2], "maxz": mxv[2],
    }
    save_mag_cal(c)
    hud("mag saved", "ok", "")
    time.sleep_ms(800)
    return c

def apply_mag(mx, my, mz, cal):
    ox = 0.5 * (cal["minx"] + cal["maxx"])
    oy = 0.5 * (cal["miny"] + cal["maxy"])
    oz = 0.5 * (cal["minz"] + cal["maxz"])
    sx = 0.5 * (cal["maxx"] - cal["minx"]) or 1.0
    sy = 0.5 * (cal["maxy"] - cal["miny"]) or 1.0
    sz = 0.5 * (cal["maxz"] - cal["minz"]) or 1.0
    return (mx - ox) / sx, (my - oy) / sy, (mz - oz) / sz

def tilt_heading(mx, my, mz, pitch, roll):
    sp, cp = math.sin(pitch), math.cos(pitch)
    sr, cr = math.sin(roll), math.cos(roll)
    mxh = mx * cp + my * sr * sp + mz * cr * sp
    myh = my * cr - mz * sr
    return math.atan2(-myh, mxh)

mw(0x6B, 0x80); time.sleep_ms(100)
mw(0x6B, 0x01); time.sleep_ms(50)
mw(0x6C, 0x00); mw(0x1A, 0x03); mw(0x1B, 0x00); mw(0x1C, 0x00)
hw(0x00, 0x70); hw(0x01, 0x20); hw(0x02, 0x00)
i2c_retry(lambda: i2c.writeto_mem(BARO, 0x26, b"\x38"))
time.sleep_ms(10)
i2c_retry(lambda: i2c.writeto_mem(BARO, 0x26, b"\x39"))
time.sleep_ms(200)

print("hold still — gyro + baro cal…")
hud("gyro+baro", "{:.0f} ft MSL".format(HOME_ALT_FT), "hold still")

sx = sy = sz = 0.0
ax0 = ay0 = az0 = 0.0
n_ok = 0
while n_ok < 200:
    try:
        ax, ay, az, _, gx, gy, gz = read_mpu14()
    except OSError:
        time.sleep_ms(20)
        continue
    sx += gx; sy += gy; sz += gz
    ax0 += ax; ay0 += ay; az0 += az
    n_ok += 1
    time.sleep_ms(5)
bx, by, bz = sx / 200 / GYRO_LSB, sy / 200 / GYRO_LSB, sz / 200 / GYRO_LSB
g_rest = (ax0 / 200 / ACCEL_LSB, ay0 / 200 / ACCEL_LSB, az0 / 200 / ACCEL_LSB)
g_mag0 = math.sqrt(g_rest[0] ** 2 + g_rest[1] ** 2 + g_rest[2] ** 2) or 1.0

p_sum = 0.0
n_p = 0
for _ in range(40):
    try:
        pa, _ = read_baro()
    except OSError:
        continue
    if pa > 0:
        p_sum += pa
        n_p += 1
    time.sleep_ms(50)
p0 = p_sum / n_p if n_p else 101325.0
alt_ft = HOME_ALT_FT
alt_ema = HOME_ALT_FT
vz = 0.0
print("gyro+baro done p0=", int(p0))

roll0 = math.atan2(-ax0 / 200, math.sqrt((ay0 / 200) ** 2 + (az0 / 200) ** 2))
pitch0 = math.atan2(ay0 / 200, az0 / 200)

mag_cal = load_mag_cal()
if mag_cal is None:
    print("no mag cal — run figure-8")
    mag_cal = run_mag_cal()
else:
    print("mag cal loaded")
    hud("mag cal", "loaded", "hdg=0")
    time.sleep_ms(600)

mx_s = my_s = mz_s = 0.0
n_m = 0
for _ in range(30):
    try:
        mx_r, my_r, mz_r = read_mag_raw()
        mx, my, mz = apply_mag(mx_r, my_r, mz_r, mag_cal)
        mx_s += mx; my_s += my; mz_s += mz
        n_m += 1
    except OSError:
        pass
    time.sleep_ms(20)
yaw_ref = tilt_heading(mx_s / n_m, my_s / n_m, mz_s / n_m, pitch0, roll0) if n_m else 0.0

print("go yaw_ref=", math.degrees(yaw_ref))
hud("go", "hdg 0", "")
time.sleep_ms(400)

V = [
    (-2.6, 0.00, 0.05), (-2.2, 0.00, 0.22), (-2.2, 0.00, -0.18), (-1.7, 0.00, 0.38),
    (-0.3, 0.45, 0.00), (-0.3, -0.45, 0.00), (0.6, 2.40, -0.05), (0.6, -2.40, -0.05),
    (0.2, 1.40, -0.02), (0.2, -1.40, -0.02), (0.0, 0.85, -0.28), (0.0, -0.85, -0.28),
    (0.25, 1.70, -0.22), (0.25, -1.70, -0.22), (2.0, 0.00, 0.05), (1.85, 0.00, 0.95),
    (1.75, 0.95, 0.15), (1.75, -0.95, 0.15), (1.2, 0.00, 0.28),
]
EDGES = [
    (0, 1), (0, 2), (1, 3), (1, 4), (1, 5), (2, 4), (2, 5),
    (3, 18), (4, 18), (5, 18), (18, 14), (4, 14), (5, 14), (2, 14),
    (4, 8), (8, 6), (5, 9), (9, 7), (4, 6), (5, 7),
    (8, 10), (8, 12), (9, 11), (9, 13), (10, 4), (11, 5), (12, 6), (13, 7),
    (14, 15), (14, 16), (14, 17), (16, 17), (15, 18),
]

def rot(x, y, z, yaw, pitch, roll):
    cy, sy_ = math.cos(yaw), math.sin(yaw)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cr, sr = math.cos(roll), math.sin(roll)
    x, y = x * cy - y * sy_, x * sy_ + y * cy
    y, z = y * cp - z * sp, y * sp + z * cp
    x, z = x * cr + z * sr, -x * sr + z * cr
    return x, y, z

roll, pitch = roll0, pitch0
yaw = 0.0
tprev = time.ticks_ms()

while True:
    now = time.ticks_ms()
    dt = time.ticks_diff(now, tprev) / 1000.0
    tprev = now
    if dt <= 0 or dt > 0.2:
        dt = 0.05
    try:
        ax, ay, az, _, gx, gy, gz = read_mpu14()
        mx_r, my_r, mz_r = read_mag_raw()
        pascals, temp_c = read_baro()
    except OSError:
        time.sleep_ms(10)
        continue

    axg, ayg, azg = ax / ACCEL_LSB, ay / ACCEL_LSB, az / ACCEL_LSB
    gforce = math.sqrt(axg * axg + ayg * ayg + azg * azg)
    # body rates: gx→pitch, gy→roll, gz→yaw (our mapping)
    p = math.radians(gy / GYRO_LSB - by)   # roll rate
    q = math.radians(gx / GYRO_LSB - bx)   # pitch rate
    r = math.radians(gz / GYRO_LSB - bz)   # yaw rate
    mx, my, mz = apply_mag(mx_r, my_r, mz_r, mag_cal)

    roll += p * dt
    pitch += q * dt
    # heading rate about vertical (not body Z) — keeps sign when pitched
    cp = math.cos(pitch)
    if abs(cp) < 0.15:
        cp = 0.15 if cp >= 0 else -0.15
    yaw += ((q * math.sin(roll) + r * math.cos(roll)) / cp) * dt

    roll = 0.98 * roll + 0.02 * math.atan2(-ax, math.sqrt(ay * ay + az * az))
    pitch = 0.98 * pitch + 0.02 * math.atan2(ay, az)

    if abs(pitch) < MAG_BLEND_MAX and abs(roll) < MAG_BLEND_MAX:
        yaw_m = tilt_heading(mx, my, mz, pitch, roll)
        yaw_rel = (yaw_m - yaw_ref + math.pi) % (2 * math.pi) - math.pi
        yaw += 0.08 * ((yaw_rel - yaw + math.pi) % (2 * math.pi) - math.pi)

    alt_baro = pa_to_ft(pascals, p0)
    alt_ema = 0.92 * alt_ema + 0.08 * alt_baro
    a_along_g = (axg * g_rest[0] + ayg * g_rest[1] + azg * g_rest[2]) / g_mag0
    a_vert_g = a_along_g - g_mag0
    vz = 0.95 * vz + (a_vert_g * 32.174) * dt
    alt_ft = 0.85 * (alt_ema + vz * 0.15) + 0.15 * alt_baro

    s = 12 + min(8, abs(gforce - 1) * 6)
    if s < 10:
        s = 10
    if s > 20:
        s = 20
    pts = []
    for x, y, z in V:
        xr, yr, zr = rot(x, y, z, -yaw + YAW0, pitch, roll)
        pts.append((int(64 + xr * s), int(32 - zr * s)))

    oled_fill(0)
    for a, b in EDGES:
        line(pts[a][0], pts[a][1], pts[b][0], pts[b][1], 1)
    hdg = math.degrees(yaw) % 360
    oled_text("{:.0f}ft".format(alt_ft), 0, 0)
    oled_text("{:.1f}g".format(gforce), 90, 0)
    oled_text("{:.0f}F".format(temp_c * 9 / 5 + 32), 0, 56)
    oled_text("{:3.0f}".format(hdg), 104, 56)
    oled_show()