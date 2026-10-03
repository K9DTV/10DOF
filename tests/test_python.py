"""Host checks for the Pico scripts, HUD math, version, and license."""

import ast
import hashlib
import math
from pathlib import Path

import pytest

from lib import hud_math as hm

ROOT = Path(__file__).resolve().parents[1]

FIRMWARE = {
    "10DOF.py": "c6498430dfdd834ef72c5b2c3b110a51",
    "MPU-92_68_9250.py": "8f6f346907b1bf01d8d2bec510a17bf2",
    "Plane_heading_3D.py": "bbd1a9dc3c2e4ac272d558da4de5c781",
}


def _text(name):
    return (ROOT / name).read_text(encoding="utf-8")


def test_firmware_matches_uploaded_sources_and_parses():
    for name, digest in FIRMWARE.items():
        raw = (ROOT / name).read_bytes()
        assert hashlib.md5(raw).hexdigest() == digest
        ast.parse(raw.decode("utf-8"), filename=name)
        assert "secrets" not in raw.decode("utf-8").lower()


def test_no_secrets_file_and_photo_is_jpeg():
    assert not (ROOT / "secrets.h").exists()
    assert not (ROOT / "secrets.py").exists()
    assert "secrets.h" in (ROOT / ".gitignore").read_text(encoding="utf-8")
    photo = (ROOT / "20261003_123433.jpg").read_bytes()
    assert photo[:3] == b"\xff\xd8\xff"
    assert hashlib.md5(photo).hexdigest() == "b71e2db65fd4f726d09fa72faf7e6121"


def test_license_is_mit_james_k9dtv():
    text = (ROOT / "LICENSE").read_bytes()
    assert hashlib.md5(text).hexdigest() == "a12578c4f2a26f24927ffa72e0dcc250"
    assert text.startswith(b"MIT License\n")
    assert b"Copyright (c) 2026 James, K9DTV\n" in text
    readme = _text("README.md")
    assert "MIT" in readme
    assert "Copyright (c) 2026 James, K9DTV" in readme


def test_version_1_00_00_is_consistent():
    assert (ROOT / "VERSION").read_text(encoding="utf-8").strip() == "1.00.00"
    changelog = _text("CHANGELOG.md")
    assert "## 1.00.00" in changelog
    project = _text("pyproject.toml")
    assert 'version = "1.00.00"' in project
    ci = _text(".github/workflows/ci.yml")
    assert "pytest" in ci
    assert "requirements-dev.txt" in ci
    assert "pytest" in _text("requirements-dev.txt")
    assert "pytest" in _text("Makefile")


def test_plane_script_keeps_the_locked_constants():
    src = _text("Plane_heading_3D.py")
    for snippet in (
        "HOME_ALT_FT = 450.0",
        "MAG_MIN_SPAN = 80",
        "MAG_CAL_SECS = 25",
        "32.174",
        "145366.45",
        "0.190284",
        "0x68",
        "0x1E",
        "0x60",
        "0x3C",
        "SDA, SCL = 0, 1",
        "mag_cal.json",
    ):
        assert snippet in src
    assert hm.HOME_ALT_FT == 450.0
    assert hm.G_FT_S2 == 32.174
    assert hm.ADDR_MPU == 0x68
    assert hm.ADDR_HMC == 0x1E
    assert hm.ADDR_BARO == 0x60
    assert hm.ADDR_OLED == 0x3C
    assert "Pico" in src
    assert "Pico" in _text("10DOF.py")
    assert "Pico" in _text("MPU-92_68_9250.py")


def test_rotation_preserves_length_and_known_axes():
    assert hm.rot(1, 2, 3, 0, 0, 0) == pytest.approx((1, 2, 3))
    assert hm.rot(1, 0, 0, math.pi / 2, 0, 0) == pytest.approx((0, 1, 0), abs=1e-12)
    assert hm.rot(0, 1, 0, 0, math.pi / 2, 0) == pytest.approx((0, 0, 1), abs=1e-12)
    assert hm.rot(1, 0, 0, 0, 0, math.pi / 2) == pytest.approx((0, 0, -1), abs=1e-12)
    yaw, pitch, roll = 0.4, -0.7, 1.1
    basis = [hm.rot(1, 0, 0, yaw, pitch, roll),
             hm.rot(0, 1, 0, yaw, pitch, roll),
             hm.rot(0, 0, 1, yaw, pitch, roll)]
    for col in basis:
        assert math.sqrt(sum(v * v for v in col)) == pytest.approx(1.0)
    det = (
        basis[0][0] * (basis[1][1] * basis[2][2] - basis[1][2] * basis[2][1])
        - basis[0][1] * (basis[1][0] * basis[2][2] - basis[1][2] * basis[2][0])
        + basis[0][2] * (basis[1][0] * basis[2][1] - basis[1][1] * basis[2][0])
    )
    assert det == pytest.approx(1.0)


def test_mag_calibration_and_headings():
    assert hm.mag_cal_ok({"maxx": 1}) is False
    short = {"minx": -39, "maxx": 40, "miny": -80, "maxy": 80, "minz": -80, "maxz": 80}
    assert hm.mag_cal_ok(short) is False
    cal = {"minx": -80, "maxx": 80, "miny": -100, "maxy": 20, "minz": 0, "maxz": 80}
    assert hm.mag_cal_ok(cal) is True
    assert hm.apply_mag(0, -40, 40, cal) == pytest.approx((0.0, 0.0, 0.0))
    assert hm.apply_mag(80, 20, 80, cal) == pytest.approx((1.0, 1.0, 1.0))
    zero = {"minx": -10, "maxx": 10, "miny": 0, "maxy": 0, "minz": -5, "maxz": 5}
    assert hm.apply_mag(5, 7, 0, zero)[1] == pytest.approx(7.0)
    assert hm.tilt_heading(1, 0, 0, 0, 0) == pytest.approx(0.0)
    assert hm.tilt_heading(0, 1, 0, 0, 0) == pytest.approx(-math.pi / 2)
    assert hm.cube_heading(1, 0, 0, 0, 0) == pytest.approx(0.0)
    assert hm.cube_heading(0, 1, 0, 0, 0) == pytest.approx(math.pi / 2)


def test_attitude_filter_altitude_and_scale():
    assert hm.clamp_dt(0) == 0.05
    assert hm.clamp_dt(1) == 0.05
    assert hm.clamp_dt(-0.01) == 0.05
    assert hm.clamp_dt(0.02) == 0.02
    assert hm.pitch_roll_deg(0, 0, 1) == pytest.approx((0, 0))
    pitch, roll = hm.hud_accel_attitude(0, 0, 16384)
    assert pitch == pytest.approx(0)
    assert roll == pytest.approx(0)
    blended = hm.integrate_and_blend(0.0, 1.0, 0.1, 0.0, alpha=0.98)
    assert blended == pytest.approx(0.098)
    assert hm.yaw_rate_vertical(0.0, 0.2, 0.0, 0.0) == pytest.approx(0.2)
    steep = hm.yaw_rate_vertical(0.0, 0.2, 0.0, math.pi / 2)
    assert steep == pytest.approx(0.2 / 0.15)
    assert hm.maybe_blend_yaw(0.0, 1.0, 0.0, 0.0, 0.0) == pytest.approx(0.08)
    held = hm.maybe_blend_yaw(0.2, 1.0, 0.0, hm.MAG_BLEND_MAX, 0.0)
    assert held == 0.2
    ema, vz, alt = hm.fuse_altitude(450.0, 0.0, 450.0, 0.0, 0.05)
    assert (ema, vz, alt) == pytest.approx((450.0, 0.0, 450.0))
    assert hm.vertical_accel_g(0, 0, 1, (0, 0, 1), 1) == pytest.approx(0.0)
    assert hm.plane_scale(1) == 12
    assert hm.plane_scale(3) == 20
    assert 10 <= hm.plane_scale(0) <= 20
    assert hm.cube_scale(1, 101325, 101325) == 14
    assert hm.cube_scale(1, 101325 + 100, 101325) == 30
    assert hm.cube_scale(1, 101325 - 100, 101325) == 6


def test_wireframe_indexes_and_hud_layout():
    assert len(hm.PLANE_V) == 19
    assert len(hm.CUBE_V) == 8
    for edges, verts in ((hm.PLANE_EDGES, hm.PLANE_V), (hm.CUBE_EDGES, hm.CUBE_V)):
        for a, b in edges:
            assert 0 <= a < len(verts)
            assert 0 <= b < len(verts)
            assert a != b
    for x, y, z in hm.PLANE_V:
        px, py = hm.project_plane(x, y, z, 0, 0, 0, 12)
        assert isinstance(px, int) and isinstance(py, int)
    assert hm.project_cube(0, 0, 0, 0, 0, 0, 10) == (64, 28)
    # 8px font. Nominal strings from the running HUD fit the 128-wide panel.
    samples = (
        (hm.HUD_ALT_XY, "452ft"),
        (hm.HUD_G_XY, "1.0g"),
        (hm.HUD_TEMP_XY, "74F"),
        (hm.HUD_HDG_XY, "  2"),
    )
    for (x, y), text in samples:
        assert 0 <= x < hm.OLED_W
        assert 0 <= y <= hm.OLED_H - 8
        assert x + 8 * len(text) <= hm.OLED_W
    assert hm.mpu_who_ok(0x75) is True
    assert hm.mpu_who_ok(0x68) is True
    assert hm.mpu_who_ok(0x12) is False


def test_ci_workflow_targets_main():
    ci = _text(".github/workflows/ci.yml")
    assert "main" in ci
    assert "ubuntu-latest" in ci
    assert "3.12" in ci
