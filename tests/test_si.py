"""SI conversions used by the HUD: pressure, gravity, temperature, angles."""

import math

import pytest

from lib.hud_math import (
    ACCEL_LSB,
    BARO_TEMP_LSB,
    G_FT_S2,
    G_MPS2,
    GYRO_LSB,
    HOME_ALT_FT,
    M_PER_FT,
    PRESSURE_ALT_EXP,
    PRESSURE_ALT_SCALE_FT,
    STD_PA,
    accel_g,
    accel_mps2,
    baro_temp_c,
    c_to_f,
    decode_baro,
    f_to_c,
    ft_to_m,
    g_force,
    gyro_dps,
    gyro_rad_s,
    m_to_ft,
    mpu_temp_c,
    pa_to_ft,
    wrap_deg360,
    wrap_pi,
)


def test_standard_gravity_and_foot():
    assert G_MPS2 == 9.80665
    assert M_PER_FT == 0.3048
    assert ft_to_m(HOME_ALT_FT) == pytest.approx(137.16)
    assert m_to_ft(ft_to_m(450.0)) == pytest.approx(450.0)
    # 9.80665 m/s^2 expressed in feet, rounded the way the HUD integrates vz.
    assert G_FT_S2 == pytest.approx(G_MPS2 / M_PER_FT, abs=1e-3)
    assert G_FT_S2 == 32.174


def test_celsius_fahrenheit_anchors():
    assert c_to_f(0) == 32
    assert c_to_f(100) == 212
    assert c_to_f(-40) == -40
    assert f_to_c(74) == pytest.approx(23.3333333333)
    assert f_to_c(c_to_f(21.5)) == pytest.approx(21.5)


def test_sensor_temperature_scales():
    assert mpu_temp_c(0) == pytest.approx(21.0)
    assert baro_temp_c(256) == pytest.approx(1.0)
    assert baro_temp_c(-256) == pytest.approx(-1.0)
    assert BARO_TEMP_LSB == 256.0


def test_accel_and_gyro_full_scale():
    assert ACCEL_LSB == 16384.0
    assert GYRO_LSB == 131.0
    assert accel_g(16384) == pytest.approx(1.0)
    assert accel_mps2(16384) == pytest.approx(G_MPS2)
    assert accel_mps2(-16384) == pytest.approx(-G_MPS2)
    assert gyro_dps(131) == pytest.approx(1.0)
    assert gyro_dps(131, bias_dps=1.0) == pytest.approx(0.0)
    assert gyro_rad_s(131) == pytest.approx(math.pi / 180.0)
    assert g_force(0, 0, 1) == pytest.approx(1.0)
    assert g_force(3, 4, 0) == pytest.approx(5.0)


def test_pressure_altitude_identity_and_known_ratio():
    assert pa_to_ft(STD_PA, STD_PA) == HOME_ALT_FT
    assert pa_to_ft(0, STD_PA) == HOME_ALT_FT
    assert pa_to_ft(STD_PA, 0) == HOME_ALT_FT
    # 80% of the reference pressure. Independent evaluation of the NOAA form
    # h = 145366.45 * (1 - (P/P0)**0.190284).
    delta = PRESSURE_ALT_SCALE_FT * (1.0 - math.pow(0.8, PRESSURE_ALT_EXP))
    assert delta == pytest.approx(6043.1477035427, abs=1e-6)
    assert pa_to_ft(0.8 * STD_PA, STD_PA) == pytest.approx(HOME_ALT_FT + delta)
    assert pa_to_ft(0.8 * STD_PA, STD_PA) > pa_to_ft(STD_PA, STD_PA)
    assert pa_to_ft(1.02 * STD_PA, STD_PA) < HOME_ALT_FT


def test_baro_register_decode_is_integer_pascals():
    # 101325 Pa left-aligned the way OUT_P is read (>> 6), plus 25.00 C.
    word = 101325 << 6
    raw = bytes([(word >> 16) & 0xFF, (word >> 8) & 0xFF, word & 0xFF, 0x19, 0x00])
    pa, tc = decode_baro(raw)
    assert pa == 101325
    assert tc == pytest.approx(25.0)
    assert c_to_f(tc) == pytest.approx(77.0)


def test_angle_wraps_are_si_radians_and_degrees():
    assert wrap_pi(0) == 0
    assert wrap_pi(2 * math.pi) == pytest.approx(0, abs=1e-12)
    assert wrap_pi(-3 * math.pi / 2) == pytest.approx(math.pi / 2)
    assert wrap_deg360(0) == 0
    assert wrap_deg360(360) == 0
    assert wrap_deg360(-10) == pytest.approx(350)
    assert wrap_deg360(370.5) == pytest.approx(10.5)
