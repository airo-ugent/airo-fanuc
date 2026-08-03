# SPDX-License-Identifier: Apache-2.0
"""Validation and unit handling in :class:`airo_fanuc.robot_profile.RobotProfile`.

A profile is the only thing standing between a typo and a set of clamps the RT core
will happily enforce, so every way of building a wrong one should raise at construction
rather than at the first motion.
"""

from __future__ import annotations

import numpy as np
import pytest
from conftest import TEST_PROFILE

from airo_fanuc.robot_profile import SUPPORTED_NDOF, ProfileError, RobotProfile

_OK: dict[str, object] = {
    "name": "t",
    "model": "test",
    "velocity_limits_deg_s": [120.0] * 6,
    "acceleration_limits_deg_s2": [240.0] * 6,
    "jerk_limits_deg_s3": [1920.0] * 6,
    "position_limits_lower_deg": [-180.0] * 6,
    "position_limits_upper_deg": [180.0] * 6,
}


def _profile(**overrides: object) -> RobotProfile:
    return RobotProfile.from_degrees(**{**_OK, **overrides})  # type: ignore[arg-type]


def test_degrees_are_converted_to_radians() -> None:
    p = _profile()
    assert np.allclose(p.velocity_limits, np.radians([120.0] * 6))
    assert np.allclose(p.jerk_limits, np.radians([1920.0] * 6))
    # 180°/s is exactly π rad/s, which is the value the datasheet row rounds to 3.142.
    assert _profile(velocity_limits_deg_s=[180.0] * 6).velocity_limits[0] == pytest.approx(np.pi)


def test_degree_properties_round_trip() -> None:
    p = _profile()
    assert np.allclose(p.position_limits_lower_deg, [-180.0] * 6)
    assert np.allclose(p.position_limits_upper_deg, [180.0] * 6)


def test_limits_are_read_only() -> None:
    """One profile is shared by everything a driver builds from it, so an in-place edit
    would silently change clamps the RT core is already running."""
    with pytest.raises(ValueError, match="read-only"):
        TEST_PROFILE.velocity_limits[0] = 99.0


def test_wrong_length_rejected() -> None:
    with pytest.raises(ProfileError, match="velocity_limits has 3 values, expected 6"):
        _profile(velocity_limits_deg_s=[120.0] * 3)


def test_non_finite_rejected() -> None:
    with pytest.raises(ProfileError, match="non-finite"):
        _profile(jerk_limits_deg_s3=[float("inf")] * 6)


@pytest.mark.parametrize(
    "field",
    ["velocity_limits_deg_s", "acceleration_limits_deg_s2", "jerk_limits_deg_s3"],
)
def test_zero_or_negative_clamp_rejected(field: str) -> None:
    """A zero clamp is a robot that cannot move — a config error, not a safe default."""
    with pytest.raises(ProfileError, match="must be positive"):
        _profile(**{field: [0.0] * 6})


def test_crossed_position_limits_rejected() -> None:
    lower = [-180.0] * 6
    upper = [180.0] * 6
    upper[3] = -190.0
    with pytest.raises(ProfileError, match=r"position limits cross on joint\(s\) \[3\]"):
        _profile(position_limits_lower_deg=lower, position_limits_upper_deg=upper)


def test_unsupported_ndof_rejected() -> None:
    """kNumJoints is a C++ compile-time constant, so a 7-axis profile cannot run here."""
    with pytest.raises(ProfileError, match="compiled for 6 joints"):
        RobotProfile.from_degrees(
            name="seven",
            model="test",
            ndof=7,
            velocity_limits_deg_s=[120.0] * 7,
            acceleration_limits_deg_s2=[240.0] * 7,
            jerk_limits_deg_s3=[1920.0] * 7,
            position_limits_lower_deg=[-180.0] * 7,
            position_limits_upper_deg=[180.0] * 7,
        )


def test_supported_ndof_matches_the_driver() -> None:
    from airo_fanuc.driver import _NDOF

    assert SUPPORTED_NDOF == _NDOF


def test_describe_names_the_model_and_provenance() -> None:
    text = TEST_PROFILE.describe()
    assert TEST_PROFILE.model in text
    assert TEST_PROFILE.source in text
