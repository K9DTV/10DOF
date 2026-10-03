"""Pure math for the Pico / Pico 2 10DOF HUD.

Formulas match ``10DOF.py``, ``MPU-92_68_9250.py``, and ``Plane_heading_3D.py``.
No ``machine`` import, so the same file runs under CPython and MicroPython.
"""

import math
import struct

HOME_ALT_FT = 450.0
ACCEL_LSB = 16384.0  # counts per g at ±2 g
GYRO_LSB = 131.0  # counts per deg/s at ±250 dps
G_MPS2 = 9.80665
G_FT_S2 = 32.174
M_PER_FT = 0.3048

MAG_MIN_SPAN = 80
MAG_CAL_SECS = 25
YAW0 = 3 * math.pi / 2
MAG_BLEND_MAX = math.radians(12)
MAG_BLEND_GAIN = 0.08

PRESSURE_ALT_SCALE_FT = 145366.45
PRESSURE_ALT_EXP = 0.190284

MPU_TEMP_SCALE = 333.87
MPU_TEMP_OFFSET_C = 21.0
BARO_TEMP_LSB = 256.0

STD_PA = 101325.0

I2C_SDA = 0  # GP0, physical pin 1
I2C_SCL = 1  # GP1, physical pin 2
ADDR_MPU = 0x68
ADDR_HMC = 0x1E
ADDR_BARO = 0x60
ADDR_OLED = 0x3C

MPU_WHO_OK = (0x70, 0x71, 0x73, 0x75, 0x68)

OLED_W = 128
OLED_H = 64
HUD_ALT_XY = (0, 0)
HUD_G_XY = (90, 0)
HUD_TEMP_XY = (0, 56)
HUD_HDG_XY = (104, 56)


def c_to_f(temp_c):
    return temp_c * 9.0 / 5.0 + 32.0


def f_to_c(temp_f):
    return (temp_f - 32.0) * 5.0 / 9.0


def ft_to_m(feet):
    return feet * M_PER_FT


def m_to_ft(meters):
    return meters / M_PER_FT


def pa_to_ft(pa, p0, home_alt_ft=HOME_ALT_FT):
    """Feet MSL. Startup pressure ``p0`` sits at ``home_alt_ft``."""
    if pa <= 0 or p0 <= 0:
        return home_alt_ft
    return home_alt_ft + PRESSURE_ALT_SCALE_FT * (1.0 - (pa / p0) ** PRESSURE_ALT_EXP)


def mpu_temp_c(temp_raw):
    return (temp_raw / MPU_TEMP_SCALE) + MPU_TEMP_OFFSET_C


def baro_temp_c(raw_i16):
    return raw_i16 / BARO_TEMP_LSB


def decode_baro(raw5):
    """FXPQ3115 / MPL3115 OUT_P (20-bit Pa) and OUT_T (deg C * 256)."""
    b = bytes(raw5)
    pa = ((b[0] << 16) | (b[1] << 8) | b[2]) >> 6
    tc = struct.unpack(">h", b[3:5])[0] / BARO_TEMP_LSB
    return pa, tc


def accel_g(raw):
    return raw / ACCEL_LSB


def accel_mps2(raw):
    return raw / ACCEL_LSB * G_MPS2


def gyro_dps(raw, bias_dps=0.0):
    return raw / GYRO_LSB - bias_dps


def gyro_rad_s(raw, bias_dps=0.0):
    return math.radians(gyro_dps(raw, bias_dps))


def g_force(ax_g, ay_g, az_g):
    return math.sqrt(ax_g * ax_g + ay_g * ay_g + az_g * az_g)


def pitch_roll_deg(ax, ay, az):
    """``MPU-92_68_9250.pitch_roll``. Inputs share one unit (raw or m/s^2)."""
    roll = math.degrees(math.atan2(ay, az))
    pitch = math.degrees(math.atan2(-ax, math.sqrt(ay * ay + az * az)))
    return pitch, roll


def hud_accel_attitude(ax, ay, az):
    """Accel reference inside the plane HUD complementary filter. Radians."""
    roll = math.atan2(-ax, math.sqrt(ay * ay + az * az))
    pitch = math.atan2(ay, az)
    return pitch, roll


def wrap_pi(angle):
    return (angle + math.pi) % (2 * math.pi) - math.pi


def wrap_deg360(deg):
    return deg % 360.0


def clamp_dt(dt):
    if dt <= 0 or dt > 0.2:
        return 0.05
    return dt


def rot(x, y, z, yaw, pitch, roll):
    cy, sy_ = math.cos(yaw), math.sin(yaw)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cr, sr = math.cos(roll), math.sin(roll)
    x, y = x * cy - y * sy_, x * sy_ + y * cy
    y, z = y * cp - z * sp, y * sp + z * cp
    x, z = x * cr + z * sr, -x * sr + z * cr
    return x, y, z


def mag_cal_ok(c, min_span=MAG_MIN_SPAN):
    try:
        return (
            c["maxx"] - c["minx"] >= min_span
            and c["maxy"] - c["miny"] >= min_span
            and c["maxz"] - c["minz"] >= min_span
        )
    except Exception:
        return False


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


def cube_heading(mx, my, mz, pitch, roll):
    """Tilt compensation from ``10DOF.py`` (register unpack order mx, mz, my)."""
    sp, cp = math.sin(pitch), math.cos(pitch)
    sr, cr = math.sin(roll), math.cos(roll)
    mxh = mx * cp + mz * sp
    myh = mx * sr * sp + my * cr - mz * sr * cp
    return math.atan2(myh, mxh)


def blend_angle(current, target, gain):
    return current + gain * wrap_pi(target - current)


def maybe_blend_yaw(yaw, yaw_m, yaw_ref, pitch, roll, gain=MAG_BLEND_GAIN):
    if abs(pitch) < MAG_BLEND_MAX and abs(roll) < MAG_BLEND_MAX:
        yaw_rel = wrap_pi(yaw_m - yaw_ref)
        return blend_angle(yaw, yaw_rel, gain)
    return yaw


def integrate_and_blend(angle, rate, dt, measurement, alpha=0.98):
    angle = angle + rate * dt
    return alpha * angle + (1.0 - alpha) * measurement


def yaw_rate_vertical(q, r, roll, pitch):
    """Heading rate about vertical. ``q`` is pitch rate, ``r`` is yaw rate."""
    cp = math.cos(pitch)
    if abs(cp) < 0.15:
        cp = 0.15 if cp >= 0 else -0.15
    return (q * math.sin(roll) + r * math.cos(roll)) / cp


def vertical_accel_g(axg, ayg, azg, g_rest, g_mag0):
    along = (axg * g_rest[0] + ayg * g_rest[1] + azg * g_rest[2]) / g_mag0
    return along - g_mag0


def fuse_altitude(alt_ema, vz, alt_baro, a_vert_g, dt):
    alt_ema = 0.92 * alt_ema + 0.08 * alt_baro
    vz = 0.95 * vz + (a_vert_g * G_FT_S2) * dt
    alt_ft = 0.85 * (alt_ema + vz * 0.15) + 0.15 * alt_baro
    return alt_ema, vz, alt_ft


def plane_scale(gforce):
    s = 12 + min(8, abs(gforce - 1) * 6)
    if s < 10:
        s = 10
    if s > 20:
        s = 20
    return s


def cube_scale(amag, pascals, p0):
    s = 14 + (amag - 1) * 8 + (pascals - p0) * 1.0
    if s < 6:
        s = 6
    if s > 30:
        s = 30
    return s


def project_plane(x, y, z, yaw, pitch, roll, scale, yaw0=YAW0):
    xr, _yr, zr = rot(x, y, z, -yaw + yaw0, pitch, roll)
    return int(64 + xr * scale), int(32 - zr * scale)


def project_cube(x, y, z, yaw, pitch, roll, scale):
    xr, _yr, zr = rot(x, y, z, yaw, pitch, roll)
    return int(64 + xr * scale), int(28 - zr * scale)


def mpu_who_ok(who):
    return who in MPU_WHO_OK


PLANE_V = (
    (-2.6, 0.00, 0.05), (-2.2, 0.00, 0.22), (-2.2, 0.00, -0.18), (-1.7, 0.00, 0.38),
    (-0.3, 0.45, 0.00), (-0.3, -0.45, 0.00), (0.6, 2.40, -0.05), (0.6, -2.40, -0.05),
    (0.2, 1.40, -0.02), (0.2, -1.40, -0.02), (0.0, 0.85, -0.28), (0.0, -0.85, -0.28),
    (0.25, 1.70, -0.22), (0.25, -1.70, -0.22), (2.0, 0.00, 0.05), (1.85, 0.00, 0.95),
    (1.75, 0.95, 0.15), (1.75, -0.95, 0.15), (1.2, 0.00, 0.28),
)

PLANE_EDGES = (
    (0, 1), (0, 2), (1, 3), (1, 4), (1, 5), (2, 4), (2, 5),
    (3, 18), (4, 18), (5, 18), (18, 14), (4, 14), (5, 14), (2, 14),
    (4, 8), (8, 6), (5, 9), (9, 7), (4, 6), (5, 7),
    (8, 10), (8, 12), (9, 11), (9, 13), (10, 4), (11, 5), (12, 6), (13, 7),
    (14, 15), (14, 16), (14, 17), (16, 17), (15, 18),
)

CUBE_V = (
    (-1, -1, -1), (1, -1, -1), (1, 1, -1), (-1, 1, -1),
    (-1, -1, 1), (1, -1, 1), (1, 1, 1), (-1, 1, 1),
)

CUBE_EDGES = (
    (0, 1), (1, 2), (2, 3), (3, 0), (4, 5), (5, 6), (6, 7), (7, 4),
    (0, 4), (1, 5), (2, 6), (3, 7),
)
