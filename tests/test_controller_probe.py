# SPDX-License-Identifier: Apache-2.0
"""The controller probe, and the full-preflight gate that consumes it.

The fixtures under ``tests/goldens/ftp/`` are real files served by our R-30iB's ``md:``
device, redacted of the customer reference and serial numbers and — for ``symotn.va``,
which is 650 kB on the wire — trimmed to the ``Field:`` blocks the probe reads plus a
few decoys with confusable names.

None of these tests touch the network: the FTP transport is a dozen lines of
:mod:`ftplib` and the parsers are where the risk is, since a silently mis-parsed
velocity limit becomes the ceiling the RT core clamps against.
"""

from __future__ import annotations

import pathlib

import numpy as np
import pytest

from airo_fanuc.controller_probe import (
    OPTION_EXTERNAL_CONTROL,
    ControllerFacts,
    ControllerLimits,
    ControllerProbeError,
    OrderFile,
    VersionInfo,
    extract_sysvars,
    format_profile_source,
    p_level_key,
    parse_orderfile,
    parse_version_dg,
    profile_from_controller,
)
from airo_fanuc.preflight import (
    PreflightReport,
    _check_external_control,
    _check_p_level,
    _cross_check_profile,
)
from airo_fanuc.robot_profile import RobotProfile

_FIXTURES = pathlib.Path(__file__).parent / "goldens" / "ftp"

# The values our controller actually reports, which are what these fixtures capture.
_VELOCITY_DEG_S = (120.0, 120.0, 180.0, 180.0, 180.0, 180.0)
_LOWER_DEG = (-179.999, -179.999, -270.0, -190.0, -179.999, -225.0)
_UPPER_DEG = (179.999, 179.999, 270.0, 190.0, 179.999, 225.0)


def _fixture(name: str) -> str:
    return (_FIXTURES / name).read_text(encoding="latin-1")


# --------------------------------------------------------------------------- #
# P-level: one quantity, three spellings across the controller's own files.
# --------------------------------------------------------------------------- #


@pytest.mark.parametrize(
    "text,expected",
    [
        ("V9.40/P82", (9, 40, 82)),  # orderfil.dat
        ("V9.40P/84", (9, 40, 84)),  # version.dg
        ("V9.40P84", (9, 40, 84)),  # controller_facts thresholds
        ("V9.40P/77", (9, 40, 77)),
        ("Software Edition No.: V9.40P/84", (9, 40, 84)),
        ("V9.40534", None),  # a root version carries no P-level
        ("", None),
        ("nonsense", None),
    ],
)
def test_p_level_key(text: str, expected: tuple[int, int, int] | None) -> None:
    assert p_level_key(text) == expected


def test_p_level_key_orders_the_bands() -> None:
    """The gate is a comparison, so the ordering is the property that matters."""
    assert p_level_key("V9.40/P82") < p_level_key("V9.40P84")  # type: ignore[operator]
    assert p_level_key("V9.40/P82") > p_level_key("V9.40P81")  # type: ignore[operator]


# --------------------------------------------------------------------------- #
# orderfil.dat — the option manifest and the P-level the gate bands on.
# --------------------------------------------------------------------------- #


def test_parse_orderfile() -> None:
    order = parse_orderfile(_fixture("orderfil.dat"))
    assert order.deliver_version == "V9.40/P82"
    assert order.customer == "Example Customer"
    assert len(order.options) == 19
    assert order.has_option(OPTION_EXTERNAL_CONTROL)
    assert order.option_description(OPTION_EXTERNAL_CONTROL) == "External Control Pkg"
    # The mechanical unit and the DCS option are ordinary entries in the same manifest.
    assert order.option_description("H705") == "CRX-10iA/L"
    assert order.has_option("J567")


def test_parse_orderfile_matches_bare_and_full_codes() -> None:
    """The leading 1A05B-26xx group varies by controller, so a bare code must match."""
    order = parse_orderfile(_fixture("orderfil.dat"))
    assert order.has_option("S636")
    assert order.has_option("1A05B-2600-S636")
    assert not order.has_option("S999")


def test_parse_orderfile_strips_the_software_variant_suffix() -> None:
    """`1A05B-2680-H510#7DF5` is one option, not a code ending in a hash."""
    order = parse_orderfile(_fixture("orderfil.dat"))
    assert "1A05B-2680-H510" in order.option_codes


def test_parse_orderfile_tolerates_junk() -> None:
    order = parse_orderfile("!Orderfile Header\ngarbage\n\n!Orderfile Starts\n")
    assert order.deliver_version is None
    assert order.options == ()
    assert not order.has_option("S636")


# --------------------------------------------------------------------------- #
# version.dg — the model sits on the line AFTER its label, not on it.
# --------------------------------------------------------------------------- #


def test_parse_version_dg() -> None:
    v = parse_version_dg(_fixture("version.dg"))
    assert v.model == "CRX-10iA/L"
    assert v.software_edition == "V9.40P/84"
    assert v.root_version == "V9.40534"
    assert v.boot_monitor == "V9.40P/77"
    assert v.servo_code == "V16.01"
    assert v.dcs_version == "V4.5.15"
    assert v.serial == "E000000"


def test_parse_version_dg_without_a_personality_block() -> None:
    v = parse_version_dg("F Number: E000000\nSoftware Edition No.: V9.40P/84\n")
    assert v.model is None
    assert v.software_edition == "V9.40P/84"


def test_order_file_and_version_disagree_on_the_p_level() -> None:
    """A real property of this controller, not a parse bug: the order file records what
    was shipped and version.dg what is running. The gate bands on the order file, so the
    two being different has to survive parsing rather than be normalised away."""
    order = parse_orderfile(_fixture("orderfil.dat"))
    version = parse_version_dg(_fixture("version.dg"))
    assert p_level_key(order.deliver_version or "") == (9, 40, 82)
    assert p_level_key(version.software_edition or "") == (9, 40, 84)


# --------------------------------------------------------------------------- #
# symotn.va — the arm's own limits.
# --------------------------------------------------------------------------- #


def test_extract_sysvars_reads_arrays_and_scalars() -> None:
    got = extract_sysvars(
        _fixture("symotn_excerpt.va"),
        ("$PARAM_GROUP[1].$JNTVELLIM", "$MRR_GRP[1].$MAX_PAYLOAD"),
    )
    assert got["$PARAM_GROUP[1].$JNTVELLIM"] == (*_VELOCITY_DEG_S, 0.0, 0.0, 0.0)
    assert got["$MRR_GRP[1].$MAX_PAYLOAD"] == 10.0


def test_extract_sysvars_does_not_confuse_a_longer_name() -> None:
    """``$LOWERLIMS`` and ``$LOWERLIMSDF`` are different variables; the second is the
    factory default and picking it up instead would be silent and wrong."""
    got = extract_sysvars(_fixture("symotn_excerpt.va"), ("$PARAM_GROUP[1].$LOWERLIMS",))
    assert set(got) == {"$PARAM_GROUP[1].$LOWERLIMS"}
    assert got["$PARAM_GROUP[1].$LOWERLIMS"][:6] == _LOWER_DEG


def test_extract_sysvars_is_case_insensitive_and_skips_absent_names() -> None:
    got = extract_sysvars(
        _fixture("symotn_excerpt.va"),
        ("$param_group[1].$jntvellim", "$PARAM_GROUP[1].$NOT_A_VARIABLE"),
    )
    assert list(got) == ["$param_group[1].$jntvellim"]


def test_extract_sysvars_on_an_empty_file() -> None:
    assert extract_sysvars("", ("$PARAM_GROUP[1].$JNTVELLIM",)) == {}


# --------------------------------------------------------------------------- #
# ControllerLimits: axis count and the active-vs-master cross-check.
# --------------------------------------------------------------------------- #


def _limits(**over: object) -> ControllerLimits:
    base: dict[str, object] = {
        "velocity_deg_s": (*_VELOCITY_DEG_S, 0.0, 0.0, 0.0),
        "lower_deg": (*_LOWER_DEG, 0.0, 0.0, 0.0),
        "upper_deg": (*_UPPER_DEG, 0.0, 0.0, 0.0),
        "master_velocity_deg_s": _VELOCITY_DEG_S,
        "master_lower_deg": _LOWER_DEG,
        "master_upper_deg": _UPPER_DEG,
        "max_payload_kg": 10.0,
        "jnt23_uplim": 0.0,
        "jnt23_lowli": 0.0,
    }
    base.update(over)
    return ControllerLimits(**base)  # type: ignore[arg-type]


def test_ndof_is_the_count_of_positive_velocity_limits() -> None:
    """Group arrays are ARRAY[9] whatever the arm is; the zero-filled tail is not axes."""
    assert _limits().ndof == 6
    assert _limits(velocity_deg_s=(120.0, 120.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)).ndof == 2
    assert _limits(velocity_deg_s=(0.0,) * 9).ndof == 0


def test_jnt23_envelope_inactive_when_both_bounds_are_zero() -> None:
    assert not _limits().jnt23_active
    assert _limits(jnt23_uplim=45.0).jnt23_active


def test_active_and_master_limits_agree_on_our_controller() -> None:
    assert _limits().disagreements() == []


def test_disagreement_is_reported_per_joint_and_names_both_sources() -> None:
    narrowed = list(_LOWER_DEG)
    narrowed[3] = -90.0  # someone narrowed J4 in the active copy
    found = _limits(lower_deg=(*narrowed, 0.0, 0.0, 0.0)).disagreements()
    assert len(found) == 1
    assert "lower J4" in found[0]
    assert "$PARAM_GROUP" in found[0] and "$MRR_GRP" in found[0]


def test_master_copy_absent_is_not_a_disagreement() -> None:
    assert _limits(master_lower_deg=(), master_upper_deg=(), master_velocity_deg_s=()).disagreements() == []


# --------------------------------------------------------------------------- #
# ControllerFacts / the derived profile.
# --------------------------------------------------------------------------- #


def _facts(**over: object) -> ControllerFacts:
    base: dict[str, object] = {
        "ip": "192.0.2.10",
        "order": parse_orderfile(_fixture("orderfil.dat")),
        "version": parse_version_dg(_fixture("version.dg")),
        "limits": _limits(),
        "tp_programs": ("gripdisp", "grprun", "rmi_move", "stream_motn"),
    }
    base.update(over)
    return ControllerFacts(**base)  # type: ignore[arg-type]


def test_facts_expose_the_model_and_option_verdict() -> None:
    facts = _facts()
    assert facts.model == "CRX-10iA/L"
    assert facts.has_external_control
    assert facts.has_tp_program("stream_motn")
    assert not facts.has_tp_program("nope")


def test_missing_order_file_reads_as_unknown_not_absent() -> None:
    """A file that would not fetch must never be evidence that an option is missing."""
    facts = _facts(order=None)
    assert not facts.has_external_control  # False, but the summary says "not found"
    assert "not found" in facts.summary()


def test_profile_from_controller_reads_limits_and_derives_the_rest() -> None:
    profile, facts = profile_from_controller("192.0.2.10", facts=_facts())
    assert profile.ndof == 6
    assert profile.model == "CRX-10iA/L"
    assert np.allclose(np.degrees(profile.velocity_limits), _VELOCITY_DEG_S)
    assert np.allclose(profile.position_limits_lower_deg, _LOWER_DEG)
    assert np.allclose(profile.position_limits_upper_deg, _UPPER_DEG)
    # a = 2·v and j = 8·a, derived rather than read.
    assert np.allclose(profile.acceleration_limits, 2.0 * profile.velocity_limits)
    assert np.allclose(profile.jerk_limits, 16.0 * profile.velocity_limits)
    assert profile.max_payload_kg == 10.0
    assert facts.model == "CRX-10iA/L"


def test_derived_profile_source_says_what_was_measured_and_what_was_not() -> None:
    """The source string is the only thing standing between a reader and the assumption
    that every number in a derived profile came off the controller."""
    profile, _ = profile_from_controller("192.0.2.10", facts=_facts())
    assert "$PARAM_GROUP" in profile.source
    assert "derived, not measured" in profile.source


def test_profile_from_controller_honours_the_derivation_factors() -> None:
    profile, _ = profile_from_controller(
        "192.0.2.10", facts=_facts(), accel_from_velocity=1.0, jerk_from_accel=3.0
    )
    assert np.allclose(profile.acceleration_limits, profile.velocity_limits)
    assert np.allclose(profile.jerk_limits, 3.0 * profile.acceleration_limits)


def test_profile_from_controller_refuses_without_limits() -> None:
    with pytest.raises(ControllerProbeError, match="did not report joint limits"):
        profile_from_controller("192.0.2.10", facts=_facts(limits=None))


def test_derived_profile_trims_the_zero_filled_array_tail() -> None:
    """ARRAY[9] in, 6 joints out — a profile carrying three zero-velocity axes would be
    rejected by RobotProfile, so the trim is what makes the derivation work at all."""
    profile, _ = profile_from_controller("192.0.2.10", facts=_facts())
    assert profile.velocity_limits.shape == (6,)


def test_emitted_profile_source_round_trips() -> None:
    """The CLI's --emit-profile output has to be code that rebuilds the same profile."""
    profile, _ = profile_from_controller("192.0.2.10", facts=_facts())
    namespace: dict[str, object] = {}
    exec(format_profile_source(profile), namespace)  # noqa: S102 - our own generated source
    rebuilt = namespace["PROFILE"]
    assert isinstance(rebuilt, RobotProfile)
    assert np.allclose(rebuilt.velocity_limits, profile.velocity_limits)
    assert np.allclose(rebuilt.position_limits_lower, profile.position_limits_lower)
    assert np.allclose(rebuilt.jerk_limits, profile.jerk_limits)
    assert rebuilt.model == profile.model
    assert rebuilt.max_payload_kg == profile.max_payload_kg


def test_dataclasses_are_constructible_empty() -> None:
    """A partial probe still has to produce a usable report."""
    assert OrderFile().deliver_version is None
    assert VersionInfo().model is None
    assert ControllerFacts(ip="192.0.2.10").limits is None


# --------------------------------------------------------------------------- #
# The full-preflight gate that consumes the probe. This is the part that can BLOCK a
# bring-up, so the governing rule is tested directly: the controller positively stating
# something disqualifying is a block, while a file that would not fetch is a warning.
# --------------------------------------------------------------------------- #


def _gate(facts: ControllerFacts, **kwargs: object) -> PreflightReport:
    report = PreflightReport()
    report.controller = facts
    for w in facts.warnings:
        report.warnings.append(f"controller probe: {w}")
    _check_p_level(report, facts)
    _check_external_control(report, facts)
    profile = kwargs.get("profile")
    if profile is not None:
        _cross_check_profile(report, facts, profile)  # type: ignore[arg-type]
    return report


def test_gate_reproduces_the_recorded_warn_band_verdict() -> None:
    """P82 sits in the P81-P83 vibration-risk band: a warning, explicitly not a block."""
    report = _gate(_facts())
    assert report.ok
    assert any("WARN band" in w for w in report.warnings)


def test_gate_hard_blocks_below_the_floor() -> None:
    order = OrderFile(deliver_version="V9.40/P70", options=(("1A05B-2600-S636", "External Control Pkg"),))
    report = _gate(_facts(order=order))
    assert not report.ok
    assert any("below the hard floor" in b for b in report.hard_blocks)


def test_gate_passes_clean_above_the_warn_threshold() -> None:
    order = OrderFile(deliver_version="V9.40/P84", options=(("1A05B-2600-S636", "External Control Pkg"),))
    version = VersionInfo(software_edition="V9.40P/84")
    report = _gate(_facts(order=order, version=version))
    assert report.ok
    assert not any("WARN band" in w for w in report.warnings)
    assert not any("order file says" in w for w in report.warnings)


def test_gate_reports_the_order_file_vs_running_edition_divergence() -> None:
    report = _gate(_facts())
    assert any("the gate used the order file" in w for w in report.warnings)


def test_gate_does_not_band_without_an_order_file() -> None:
    report = _gate(_facts(order=None))
    assert report.ok
    assert any("P-level not banded" in w for w in report.warnings)


def test_missing_s636_with_the_programs_present_is_a_warning() -> None:
    """J519/R912 are bundled into S636 rather than ordered separately, so the installed
    TP programs are functional evidence that outranks a manifest omission."""
    order = OrderFile(deliver_version="V9.40/P84", options=(("1A05B-2680-H705", "CRX-10iA/L"),))
    report = _gate(_facts(order=order))
    assert report.ok
    assert any("treating External Control as present" in w for w in report.warnings)


def test_missing_s636_and_missing_programs_is_a_hard_block() -> None:
    order = OrderFile(deliver_version="V9.40/P84", options=(("1A05B-2680-H705", "CRX-10iA/L"),))
    report = _gate(_facts(order=order, tp_programs=("gripdisp", "grprun")))
    assert not report.ok
    assert any("External Control Package" in b for b in report.hard_blocks)


def test_unreadable_order_file_never_blocks_on_a_missing_option() -> None:
    """Absence of evidence is not evidence of absence: an unreachable FTP server must
    not fail a bring-up for a driver that does not need it."""
    report = _gate(_facts(order=None, tp_programs=()))
    assert report.ok
    assert report.hard_blocks == []


def test_profile_matching_the_controller_produces_no_limit_warnings() -> None:
    probed, _ = profile_from_controller("192.0.2.10", facts=_facts())
    report = _gate(_facts(), profile=probed)
    assert not any("profile" in w for w in report.warnings)


def test_profile_wider_than_the_controller_is_flagged_as_the_dangerous_direction() -> None:
    wide = RobotProfile.from_degrees(
        name="wide",
        model="test",
        velocity_limits_deg_s=list(_VELOCITY_DEG_S),
        acceleration_limits_deg_s2=[240.0] * 6,
        jerk_limits_deg_s3=[1920.0] * 6,
        position_limits_lower_deg=[-200.0, *(_LOWER_DEG[1:])],
        position_limits_upper_deg=list(_UPPER_DEG),
    )
    report = _gate(_facts(), profile=wide)
    flagged = [w for w in report.warnings if "lower limit J1" in w]
    assert len(flagged) == 1
    assert "WIDER than the controller" in flagged[0]


def test_profile_narrower_than_the_controller_is_flagged_as_conservative() -> None:
    narrow = RobotProfile.from_degrees(
        name="narrow",
        model="test",
        velocity_limits_deg_s=list(_VELOCITY_DEG_S),
        acceleration_limits_deg_s2=[240.0] * 6,
        jerk_limits_deg_s3=[1920.0] * 6,
        position_limits_lower_deg=list(_LOWER_DEG),
        position_limits_upper_deg=[90.0, *(_UPPER_DEG[1:])],
    )
    report = _gate(_facts(), profile=narrow)
    flagged = [w for w in report.warnings if "upper limit J1" in w]
    assert len(flagged) == 1
    assert "conservative" in flagged[0]


def test_profile_velocity_divergence_names_the_direction() -> None:
    fast = RobotProfile.from_degrees(
        name="fast",
        model="test",
        velocity_limits_deg_s=[200.0, *(_VELOCITY_DEG_S[1:])],
        acceleration_limits_deg_s2=[400.0] * 6,
        jerk_limits_deg_s3=[3200.0] * 6,
        position_limits_lower_deg=list(_LOWER_DEG),
        position_limits_upper_deg=list(_UPPER_DEG),
    )
    report = _gate(_facts(), profile=fast)
    flagged = [w for w in report.warnings if "velocity J1" in w]
    assert len(flagged) == 1
    assert "ABOVE the controller" in flagged[0]


def test_cross_check_reports_an_axis_count_mismatch_and_stops() -> None:
    """Comparing a 6-vector against a 2-axis controller elementwise would be noise; one
    line naming the mismatch is the useful output."""
    report = _gate(
        _facts(limits=_limits(velocity_deg_s=(120.0, 120.0, *(0.0,) * 7))),
        profile=RobotProfile.from_degrees(
            name="six",
            model="test",
            velocity_limits_deg_s=list(_VELOCITY_DEG_S),
            acceleration_limits_deg_s2=[240.0] * 6,
            jerk_limits_deg_s3=[1920.0] * 6,
            position_limits_lower_deg=list(_LOWER_DEG),
            position_limits_upper_deg=list(_UPPER_DEG),
        ),
    )
    joint_lines = [w for w in report.warnings if "reports 2 joints" in w]
    assert len(joint_lines) == 1
    assert not any("lower limit" in w for w in report.warnings)
