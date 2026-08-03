# SPDX-License-Identifier: Apache-2.0
"""Read what the controller already knows about itself, over FTP.

The R-30iB serves a virtual ``md:`` device over FTP holding generated diagnostic and
variable dumps. Three of those files carry facts this driver would otherwise ask a
human to transcribe:

===================  ============================================================
``md:\\orderfil.dat``  the order file: software "Deliver Ver" (the P-level the
                     bring-up gate bands on) and every ordered option code, so
                     **S636** presence is a lookup rather than a claim.
``md:\\version.dg``    software edition, root version, boot monitor, servo/DCS
                     versions, the robot serial, and the mechanical unit's model
                     name ("Default Personality").
``md:\\symotn.va``     the motion system variables as ASCII, including
                     ``$PARAM_GROUP[1].$JNTVELLIM`` and
                     ``$LOWERLIMS`` / ``$UPPERLIMS`` — the arm's ACTIVE joint
                     velocity and position limits, in degrees.
===================  ============================================================

That last one is the point: joint velocity and position limits do not have to be
copied out of a datasheet, because the controller enforcing them will say what they
are. :func:`profile_from_controller` turns them straight into a
:class:`~airo_fanuc.robot_profile.RobotProfile`.

**What is still not on the controller**: acceleration and jerk *clamps*. The
controller publishes no equivalent, so those remain a caller decision — derived here
from the measured velocity by the documented rule, with the derivation recorded in the
profile's ``source`` rather than passed off as a controller reading.

Two properties worth keeping in mind:

* **Read-only.** Nothing here writes to the controller, and the probe never touches
  motion, RMI or Stream Motion. It is safe to run against a live cell.
* **Not on the bring-up path.** ``symotn.va`` is ~650 kB (~3 s on a point-to-point
  link), so this is an on-demand probe — :func:`airo_fanuc.preflight.run_preflight`
  reaches for it only under ``full=True``. Per-connect bring-up never FTPs.

Stdlib only (``ftplib``), so it costs the distribution nothing.
"""

from __future__ import annotations

import logging
import re
from dataclasses import dataclass, field
from ftplib import FTP, all_errors
from typing import TYPE_CHECKING

from .exceptions import FanucError
from .robot_profile import RobotProfile

if TYPE_CHECKING:
    from collections.abc import Iterable, Sequence

logger = logging.getLogger("airo_fanuc.controller_probe")

__all__ = [
    "ControllerFacts",
    "ControllerLimits",
    "ControllerProbeError",
    "OrderFile",
    "VersionInfo",
    "extract_sysvars",
    "p_level_key",
    "parse_orderfile",
    "parse_version_dg",
    "probe_controller",
    "profile_from_controller",
]

#: The controller's virtual diagnostic device, and the files read from it.
MD_DEVICE = "md:"
ORDERFILE = "orderfil.dat"
VERSION_DG = "version.dg"
MOTION_SYSVARS = "symotn.va"

#: FANUC's FTP server accepts anonymous access; no credential is configured by
#: default. Overridable for a controller where one has been set.
DEFAULT_FTP_USER = "anonymous"
DEFAULT_FTP_PASSWORD = "airo-fanuc@localhost"

#: The External Control Package (J519 Stream Motion + R912 RMI). Neither J519 nor R912
#: is a separate order code — they are bundled — so this is the code to look for.
OPTION_EXTERNAL_CONTROL = "S636"

#: Active (RW) group parameters, in DEGREES, and the read-only master copy of the same
#: three in RADIANS. Both are read: the active set is what binds, and the master set is
#: a free consistency check on the parse at full precision.
_SYSVAR_VELOCITY = "$PARAM_GROUP[1].$JNTVELLIM"
_SYSVAR_LOWER = "$PARAM_GROUP[1].$LOWERLIMS"
_SYSVAR_UPPER = "$PARAM_GROUP[1].$UPPERLIMS"
_SYSVAR_JNT23_UP = "$PARAM_GROUP[1].$JNT23_UPLIM"
_SYSVAR_JNT23_LOW = "$PARAM_GROUP[1].$JNT23_LOWLI"
_SYSVAR_MASTER_VELOCITY = "$MRR_GRP[1].$JNTVELLIM"
_SYSVAR_MASTER_LOWER = "$MRR_GRP[1].$LOWERLIMS"
_SYSVAR_MASTER_UPPER = "$MRR_GRP[1].$UPPERLIMS"
_SYSVAR_MAX_PAYLOAD = "$MRR_GRP[1].$MAX_PAYLOAD"

_WANTED_SYSVARS = (
    _SYSVAR_VELOCITY,
    _SYSVAR_LOWER,
    _SYSVAR_UPPER,
    _SYSVAR_JNT23_UP,
    _SYSVAR_JNT23_LOW,
    _SYSVAR_MASTER_VELOCITY,
    _SYSVAR_MASTER_LOWER,
    _SYSVAR_MASTER_UPPER,
    _SYSVAR_MAX_PAYLOAD,
)


class ControllerProbeError(FanucError):
    """The controller could not be read, or a file it served did not parse.

    A probe failure is diagnostic, not fatal: callers that can proceed without the
    facts (bring-up preflight, for one) should catch this and degrade to a warning
    rather than refusing to run.
    """


# ---------------------------------------------------------------------------
# Parsing — pure functions over the file text, so every one of them is testable
# against a captured fixture with no controller in the room.
# ---------------------------------------------------------------------------

# `1A05B-2600-S636 ! External Control Pkg`  → ("1A05B-2600-S636", "External Control Pkg")
_ORDER_OPTION_RE = re.compile(r"^\s*(1A05B-[0-9A-Z-]+?)(?:#\w+)?\s*!\s*(.*?)\s*$")
# `!SOF Ref5: Deliver Ver  - V9.40/P82`     → ("Deliver Ver", "V9.40/P82")
_ORDER_REF_RE = re.compile(r"^\s*!SOF Ref\d+:\s*(.+?)\s*-\s*(.*?)\s*$")

# `V9.40/P82`, `V9.40P84`, `V9.40P/84` → (9, 40, 82)
_P_LEVEL_RE = re.compile(r"V(\d+)\.(\d+)\s*/?\s*P\s*/?\s*(\d+)", re.IGNORECASE)

# `Field: $PARAM_GROUP[1].$JNTVELLIM  ARRAY[9] OF REAL`
_SYSVAR_ARRAY_RE = re.compile(r"^\s*Field:\s*(\S+)\s+ARRAY\[(\d+)\]\s+OF\s+(\w+)\s*$", re.IGNORECASE)
# `  [1] = 1.200000e+02`
_SYSVAR_ELEMENT_RE = re.compile(r"^\s*\[(\d+)\]\s*=\s*(\S+)\s*$")
# `Field: $MRR_GRP[1].$MAX_PAYLOAD Access: RO: REAL = 1.000000e+01`
_SYSVAR_SCALAR_RE = re.compile(r"^\s*Field:\s*(\S+)\s+Access:\s*\w+:\s*(\w+)\s*=\s*(.*?)\s*$", re.IGNORECASE)


def p_level_key(text: str) -> tuple[int, int, int] | None:
    """``"V9.40/P82"`` → ``(9, 40, 82)``, for ordered comparison. ``None`` if absent.

    Tolerates every spelling this controller uses for the same thing: the order file
    writes ``V9.40/P82``, ``version.dg`` writes ``V9.40P/84``, and
    ``controller_facts`` writes ``V9.40P84``.
    """
    m = _P_LEVEL_RE.search(text or "")
    if m is None:
        return None
    return (int(m.group(1)), int(m.group(2)), int(m.group(3)))


@dataclass(frozen=True)
class OrderFile:
    """Decoded ``md:\\orderfil.dat``."""

    #: The "Deliver Ver" reference — the P-level the bring-up gate bands on. Note this
    #: is NOT always the running software edition: on our controller the order file says
    #: V9.40/P82 while ``version.dg`` reports V9.40P/84, and the recorded gate decision
    #: uses this field.
    deliver_version: str | None = None
    robot_fe_no: str | None = None
    customer: str | None = None
    #: ``(full code, description)`` per ordered option, in file order.
    options: tuple[tuple[str, str], ...] = ()
    refs: tuple[tuple[str, str], ...] = ()

    @property
    def option_codes(self) -> tuple[str, ...]:
        """Just the codes, e.g. ``("1A05B-2680-H510", ...)``."""
        return tuple(code for code, _ in self.options)

    def has_option(self, code: str) -> bool:
        """Is an option present? Matches on the trailing code (``"S636"``) or the full
        part number, because the leading ``1A05B-26xx`` group varies by controller."""
        needle = code.upper()
        return any(needle in c.upper() for c in self.option_codes)

    def option_description(self, code: str) -> str | None:
        needle = code.upper()
        for full, desc in self.options:
            if needle in full.upper():
                return desc
        return None


def parse_orderfile(text: str) -> OrderFile:
    """Parse ``orderfil.dat``. Unrecognised lines are skipped, not an error: the file
    is a vendor-generated manifest and gains lines (``!Option added by ...``) over a
    controller's life."""
    refs: list[tuple[str, str]] = []
    options: list[tuple[str, str]] = []
    for line in text.splitlines():
        ref = _ORDER_REF_RE.match(line)
        if ref is not None:
            refs.append((ref.group(1), ref.group(2)))
            continue
        opt = _ORDER_OPTION_RE.match(line)
        if opt is not None:
            options.append((opt.group(1), opt.group(2)))

    by_label = {label.strip().lower(): value for label, value in refs}
    return OrderFile(
        deliver_version=by_label.get("deliver ver") or None,
        robot_fe_no=by_label.get("robot f/e no") or None,
        customer=by_label.get("customer") or None,
        options=tuple(options),
        refs=tuple(refs),
    )


@dataclass(frozen=True)
class VersionInfo:
    """Decoded ``md:\\version.dg``."""

    #: The mechanical unit, from the "Default Personality (from FD)" block — e.g.
    #: ``"CRX-10iA/L"``. Informational: it names the arm in a report and in a derived
    #: profile, and gates nothing.
    model: str | None = None
    software_edition: str | None = None
    root_version: str | None = None
    boot_monitor: str | None = None
    servo_code: str | None = None
    dcs_version: str | None = None
    tp_core_firmware: str | None = None
    serial: str | None = None


def parse_version_dg(text: str) -> VersionInfo:
    """Parse ``version.dg``: labelled ``Label : Value`` lines, plus the model, which
    sits on the line *after* the "Default Personality" marker rather than on it."""
    labelled: dict[str, str] = {}
    model: str | None = None
    lines = [ln.rstrip() for ln in text.replace("\r", "").splitlines()]
    for i, line in enumerate(lines):
        if ":" in line:
            label, _, value = line.partition(":")
            labelled.setdefault(label.strip().lower(), value.strip())
        if "default personality" in line.lower() and i + 1 < len(lines):
            # `CRX-10iA/L            V9.40P/84` — the model, then its version.
            model = lines[i + 1].split()[0] if lines[i + 1].split() else None

    return VersionInfo(
        model=model,
        software_edition=labelled.get("software edition no.") or None,
        root_version=labelled.get("root version") or None,
        boot_monitor=labelled.get("boot monitor") or None,
        servo_code=labelled.get("servo code") or None,
        dcs_version=labelled.get("dcs") or None,
        tp_core_firmware=labelled.get("tp core firmware") or None,
        serial=labelled.get("f number") or None,
    )


def _sysvar_scalar(raw: str) -> float | None:
    try:
        return float(raw)
    except ValueError:
        return None


def extract_sysvars(text: str, names: Iterable[str]) -> dict[str, float | tuple[float, ...]]:
    """Pull named variables out of a FANUC ``.va`` ASCII variable dump.

    The format has two shapes, both handled here::

        Field: $MRR_GRP[1].$MAX_PAYLOAD Access: RO: REAL = 1.000000e+01
        Field: $PARAM_GROUP[1].$JNTVELLIM  ARRAY[9] OF REAL
         [1] = 1.200000e+02
         [2] = 1.200000e+02

    Selective by name rather than parsing the whole file: ``symotn.va`` is ~10k
    variables and only a handful are wanted. Array elements are returned in index
    order with no trimming — a group array is ``ARRAY[9]`` whatever the arm's real
    joint count, and deciding how many of those slots are real joints is the caller's
    job (:func:`_infer_ndof`). Names are matched case-insensitively; a name that never
    appears is simply absent from the result.
    """
    wanted = {n.upper(): n for n in names}
    out: dict[str, float | tuple[float, ...]] = {}
    collecting: str | None = None
    elements: dict[int, float] = {}

    def flush() -> None:
        nonlocal collecting, elements
        if collecting is not None and elements:
            out[collecting] = tuple(elements[i] for i in sorted(elements))
        collecting, elements = None, {}

    for line in text.replace("\r", "").splitlines():
        if collecting is not None:
            el = _SYSVAR_ELEMENT_RE.match(line)
            if el is not None:
                value = _sysvar_scalar(el.group(2))
                if value is not None:
                    elements[int(el.group(1))] = value
                continue
            flush()

        arr = _SYSVAR_ARRAY_RE.match(line)
        if arr is not None:
            key = wanted.get(arr.group(1).upper())
            if key is not None:
                collecting, elements = key, {}
            continue

        sca = _SYSVAR_SCALAR_RE.match(line)
        if sca is not None:
            key = wanted.get(sca.group(1).upper())
            if key is not None:
                value = _sysvar_scalar(sca.group(3))
                if value is not None:
                    out[key] = value

    flush()
    return out


def _floats(value: float | tuple[float, ...] | None) -> tuple[float, ...]:
    if value is None:
        return ()
    return value if isinstance(value, tuple) else (value,)


def _infer_ndof(velocity_deg_s: Sequence[float]) -> int:
    """How many of a group array's 9 slots are real joints.

    A joint's velocity limit is positive by definition — a zero would be an axis that
    cannot move — so the count of leading positive entries is the axis count. The
    trailing slots of the ``ARRAY[9]`` are zero-filled on a 6-axis arm.
    """
    ndof = 0
    for v in velocity_deg_s:
        if v <= 0.0:
            break
        ndof += 1
    return ndof


@dataclass(frozen=True)
class ControllerLimits:
    """The arm's own limits, as the controller reports them.

    ``*_deg`` come from the active RW ``$PARAM_GROUP`` copy, which is what binds.
    ``master_*`` are the read-only ``$MRR_GRP`` copy of the same three quantities,
    stored by the controller in radians and converted here — kept so
    :meth:`disagreements` can check the parse against a second source rather than
    trusting one regex.
    """

    velocity_deg_s: tuple[float, ...] = ()
    lower_deg: tuple[float, ...] = ()
    upper_deg: tuple[float, ...] = ()
    master_velocity_deg_s: tuple[float, ...] = ()
    master_lower_deg: tuple[float, ...] = ()
    master_upper_deg: tuple[float, ...] = ()
    max_payload_kg: float | None = None
    #: ``$JNT23_UPLIM`` / ``$JNT23_LOWLI`` — the J2/J3 coupled envelope. ``0.0``/``0.0``
    #: means the coupled check is INACTIVE on this controller.
    jnt23_uplim: float | None = None
    jnt23_lowli: float | None = None

    @property
    def ndof(self) -> int:
        return _infer_ndof(self.velocity_deg_s)

    @property
    def jnt23_active(self) -> bool:
        """Is the J2/J3 coupled envelope check enabled? Both bounds zero means no."""
        return bool(self.jnt23_uplim or self.jnt23_lowli)

    def disagreements(self, *, tol_deg: float = 0.01) -> list[str]:
        """Where the active copy and the read-only master copy differ.

        Empty is the expected result. A non-empty list means either the parse is wrong
        or someone has narrowed the active limits away from the model's defaults — both
        worth surfacing, neither safe to silently pick a side on.
        """
        out: list[str] = []
        n = self.ndof
        pairs = (
            ("velocity", self.velocity_deg_s, self.master_velocity_deg_s),
            ("lower", self.lower_deg, self.master_lower_deg),
            ("upper", self.upper_deg, self.master_upper_deg),
        )
        for label, active, master in pairs:
            if not master:
                continue
            for j in range(min(n, len(active), len(master))):
                if abs(active[j] - master[j]) > tol_deg:
                    out.append(
                        f"{label} J{j + 1}: $PARAM_GROUP {active[j]:.4f}° vs "
                        f"$MRR_GRP {master[j]:.4f}°"
                    )
        return out


@dataclass(frozen=True)
class ControllerFacts:
    """Everything one probe run could read. Every field is optional: a file that could
    not be fetched or parsed leaves its field ``None`` and adds a line to
    :attr:`warnings`, so a partial probe is still a usable report."""

    ip: str
    ftp_banner: str = ""
    order: OrderFile | None = None
    version: VersionInfo | None = None
    limits: ControllerLimits | None = None
    #: Lowercased ``.tp`` program names present on ``md:``. This is how the
    #: site-installation prerequisites are checked without a pendant:
    #: ``stream_motn`` and ``rmi_move`` back the S636 option code with functional
    #: evidence, and ``gripdisp`` / ``grprun`` are the gripper path the wheel cannot
    #: ship.
    tp_programs: tuple[str, ...] = ()
    warnings: tuple[str, ...] = ()

    @property
    def model(self) -> str | None:
        return self.version.model if self.version else None

    @property
    def has_external_control(self) -> bool:
        """Is S636 ordered? Absent an order file this is False — unknown, not absent —
        so read it together with :attr:`tp_programs`, which is functional evidence."""
        return bool(self.order and self.order.has_option(OPTION_EXTERNAL_CONTROL))

    def has_tp_program(self, name: str) -> bool:
        return name.lower() in self.tp_programs

    def summary(self) -> str:
        parts = [f"ip={self.ip}"]
        if self.model:
            parts.append(f"model={self.model}")
        if self.order and self.order.deliver_version:
            parts.append(f"deliver_ver={self.order.deliver_version}")
        if self.version and self.version.software_edition:
            parts.append(f"sw_edition={self.version.software_edition}")
        if self.version and self.version.serial:
            parts.append(f"serial={self.version.serial}")
        parts.append(f"S636={'yes' if self.has_external_control else 'not found'}")
        if self.limits is not None:
            parts.append(f"ndof={self.limits.ndof}")
            parts.append(f"v_max={max(self.limits.velocity_deg_s, default=0.0):.0f}deg/s")
        if self.warnings:
            parts.append(f"warnings={list(self.warnings)}")
        return "controller[" + ", ".join(parts) + "]"


# ---------------------------------------------------------------------------
# Fetch
# ---------------------------------------------------------------------------


@dataclass
class _Fetcher:
    """One FTP session, reused across the three files (a fresh connect per file costs
    a handshake and can hit the controller's session limit)."""

    ip: str
    timeout_s: float
    user: str
    password: str
    ftp: FTP = field(init=False)
    banner: str = field(default="", init=False)

    def __enter__(self) -> _Fetcher:
        self.ftp = FTP()
        try:
            self.ftp.connect(self.ip, 21, timeout=self.timeout_s)
            self.banner = self.ftp.getwelcome() or ""
            self.ftp.login(self.user, self.password)
            self.ftp.cwd(MD_DEVICE)
        except all_errors as exc:
            raise ControllerProbeError(
                f"cannot read {MD_DEVICE} over FTP on {self.ip}: {exc}. The probe needs the "
                f"controller's FTP server reachable and {self.user!r} accepted; nothing else "
                f"in this driver depends on it."
            ) from exc
        return self

    def __exit__(self, *_: object) -> None:
        try:
            self.ftp.quit()
        except all_errors:
            try:
                self.ftp.close()
            except all_errors:
                pass

    def listing(self) -> tuple[str, ...]:
        names: list[str] = []
        self.ftp.retrlines("NLST", names.append)
        return tuple(n.strip().lower() for n in names if n.strip())

    def text(self, name: str) -> str:
        chunks = bytearray()
        self.ftp.retrbinary(f"RETR {name}", chunks.extend)
        # These files are ASCII with the occasional stray high byte in a version
        # string, so decode leniently rather than failing a whole probe on one byte.
        return chunks.decode("latin-1")


def probe_controller(
    ip: str,
    *,
    timeout_s: float = 15.0,
    user: str = DEFAULT_FTP_USER,
    password: str = DEFAULT_FTP_PASSWORD,
    want_limits: bool = True,
) -> ControllerFacts:
    """Read the controller's own account of itself over FTP. Read-only.

    Raises :class:`ControllerProbeError` only if the session cannot be established at
    all. Once connected, each file is best-effort: one that cannot be fetched or parsed
    becomes a warning on the returned :class:`ControllerFacts` and leaves its field
    ``None``.

    ``want_limits=False`` skips ``symotn.va``, which is the only large read (~650 kB,
    ~3 s) — worth it when all that is needed is the version/option gate.
    """
    warnings: list[str] = []
    order: OrderFile | None = None
    version: VersionInfo | None = None
    limits: ControllerLimits | None = None
    tp_programs: tuple[str, ...] = ()

    with _Fetcher(ip, timeout_s, user, password) as fetcher:
        banner = fetcher.banner

        try:
            tp_programs = tuple(sorted(n[:-3] for n in fetcher.listing() if n.endswith(".tp")))
        except all_errors as exc:
            warnings.append(f"could not list {MD_DEVICE}: {exc}")

        for name, parse, assign in (
            (ORDERFILE, parse_orderfile, "order"),
            (VERSION_DG, parse_version_dg, "version"),
        ):
            try:
                parsed = parse(fetcher.text(name))
            except all_errors as exc:
                warnings.append(f"could not fetch {name}: {exc}")
                continue
            except (ValueError, IndexError) as exc:  # pragma: no cover - defensive
                warnings.append(f"could not parse {name}: {exc}")
                continue
            if assign == "order":
                order = parsed  # type: ignore[assignment]
            else:
                version = parsed  # type: ignore[assignment]

        if want_limits:
            try:
                sysvars = extract_sysvars(fetcher.text(MOTION_SYSVARS), _WANTED_SYSVARS)
            except all_errors as exc:
                warnings.append(f"could not fetch {MOTION_SYSVARS}: {exc}")
            else:
                limits = _limits_from_sysvars(sysvars, warnings)

    facts = ControllerFacts(
        ip=ip,
        ftp_banner=banner,
        order=order,
        version=version,
        limits=limits,
        tp_programs=tp_programs,
        warnings=tuple(warnings),
    )
    logger.info("%s", facts.summary())
    return facts


def _rad_tuple_to_deg(values: tuple[float, ...]) -> tuple[float, ...]:
    return tuple(v * 180.0 / 3.141592653589793 for v in values)


def _limits_from_sysvars(
    sysvars: dict[str, float | tuple[float, ...]], warnings: list[str]
) -> ControllerLimits | None:
    velocity = _floats(sysvars.get(_SYSVAR_VELOCITY))
    lower = _floats(sysvars.get(_SYSVAR_LOWER))
    upper = _floats(sysvars.get(_SYSVAR_UPPER))
    if not (velocity and lower and upper):
        missing = [
            name
            for name, got in ((_SYSVAR_VELOCITY, velocity), (_SYSVAR_LOWER, lower), (_SYSVAR_UPPER, upper))
            if not got
        ]
        warnings.append(f"{MOTION_SYSVARS} did not yield {', '.join(missing)}")
        return None

    limits = ControllerLimits(
        velocity_deg_s=velocity,
        lower_deg=lower,
        upper_deg=upper,
        master_velocity_deg_s=_rad_tuple_to_deg(_floats(sysvars.get(_SYSVAR_MASTER_VELOCITY))),
        master_lower_deg=_rad_tuple_to_deg(_floats(sysvars.get(_SYSVAR_MASTER_LOWER))),
        master_upper_deg=_rad_tuple_to_deg(_floats(sysvars.get(_SYSVAR_MASTER_UPPER))),
        max_payload_kg=sysvars.get(_SYSVAR_MAX_PAYLOAD)  # type: ignore[arg-type]
        if isinstance(sysvars.get(_SYSVAR_MAX_PAYLOAD), float)
        else None,
        jnt23_uplim=sysvars.get(_SYSVAR_JNT23_UP)  # type: ignore[arg-type]
        if isinstance(sysvars.get(_SYSVAR_JNT23_UP), float)
        else None,
        jnt23_lowli=sysvars.get(_SYSVAR_JNT23_LOW)  # type: ignore[arg-type]
        if isinstance(sysvars.get(_SYSVAR_JNT23_LOW), float)
        else None,
    )
    if limits.ndof == 0:
        warnings.append(f"{_SYSVAR_VELOCITY} reported no positive joint velocity limit")
        return None
    for line in limits.disagreements():
        warnings.append(f"active vs master limit mismatch — {line}")
    return limits


# ---------------------------------------------------------------------------
# Deriving a profile
# ---------------------------------------------------------------------------

#: Acceleration and jerk are NOT on the controller in any clamp-shaped form, so a
#: derived profile scales them off the measured velocity. Same rule the hand-written
#: CRX-10iA/L profile uses; see ``examples/crx10ial.py`` for why these factors and what
#: is still open about them.
DEFAULT_ACCEL_FROM_VELOCITY = 2.0
DEFAULT_JERK_FROM_ACCEL = 8.0


def profile_from_controller(
    ip: str,
    *,
    accel_from_velocity: float = DEFAULT_ACCEL_FROM_VELOCITY,
    jerk_from_accel: float = DEFAULT_JERK_FROM_ACCEL,
    name: str | None = None,
    timeout_s: float = 15.0,
    facts: ControllerFacts | None = None,
) -> tuple[RobotProfile, ControllerFacts]:
    """Build a :class:`~airo_fanuc.robot_profile.RobotProfile` from the controller.

    Velocity and joint position limits are the controller's ACTIVE values, so they need
    no transcription and cannot drift from what the controller enforces. Acceleration
    and jerk are *derived* — ``a = accel_from_velocity · v``, ``j = jerk_from_accel · a``
    — because the controller publishes no clamp equivalent; the profile's ``source``
    records which half is which so a reader is never misled about what was measured.

    Returns the profile and the :class:`ControllerFacts` it came from, because the facts
    carry the warnings (a limit mismatch, a file that would not parse) that a caller
    should see before trusting the profile. Pass ``facts`` to reuse an earlier probe
    instead of re-fetching.

    Raises :class:`ControllerProbeError` if the controller cannot be reached or reported
    no limits — there is no partial profile worth returning, since a missing velocity
    limit is not something to guess.
    """
    facts = facts if facts is not None else probe_controller(ip, timeout_s=timeout_s)
    if facts.limits is None:
        raise ControllerProbeError(
            f"controller {ip} did not report joint limits, so no profile can be derived "
            f"({'; '.join(facts.warnings) or 'no reason given'}). Write the profile by hand "
            f"instead — see examples/crx10ial.py."
        )

    limits = facts.limits
    n = limits.ndof
    velocity = list(limits.velocity_deg_s[:n])
    accel = [accel_from_velocity * v for v in velocity]
    jerk = [jerk_from_accel * a for a in accel]

    model = facts.model or "unknown FANUC"
    derived = (
        f"velocity + position limits read from {ip} $PARAM_GROUP via FTP "
        f"({MOTION_SYSVARS}); acceleration = {accel_from_velocity:g}x velocity and "
        f"jerk = {jerk_from_accel:g}x acceleration derived, not measured"
    )
    profile = RobotProfile.from_degrees(
        name=name or (model.lower().replace("/", "").replace("-", "") if facts.model else "probed"),
        model=model,
        velocity_limits_deg_s=velocity,
        acceleration_limits_deg_s2=accel,
        jerk_limits_deg_s3=jerk,
        position_limits_lower_deg=list(limits.lower_deg[:n]),
        position_limits_upper_deg=list(limits.upper_deg[:n]),
        ndof=n,
        max_payload_kg=limits.max_payload_kg,
        source=derived,
    )
    return profile, facts


def format_profile_source(profile: RobotProfile) -> str:
    """Render a profile as the Python that reconstructs it, for pasting into a project.

    A derived profile is a snapshot of what the controller said at one moment. Writing
    it down makes it reproducible, reviewable in a diff, and available offline (a
    ``--fake`` run has no controller to ask), which is why this exists alongside
    :func:`profile_from_controller` rather than instead of it.
    """

    def fmt(values: object) -> str:
        import numpy as np

        return "[" + ", ".join(f"{float(v):g}" for v in np.asarray(values).tolist()) + "]"

    payload = "None" if profile.max_payload_kg is None else f"{profile.max_payload_kg:g}"
    return "\n".join(
        [
            "from airo_fanuc.robot_profile import RobotProfile",
            "",
            f"# {profile.source}",
            "PROFILE = RobotProfile.from_degrees(",
            f"    name={profile.name!r},",
            f"    model={profile.model!r},",
            f"    velocity_limits_deg_s={fmt(np_degrees(profile.velocity_limits))},",
            f"    acceleration_limits_deg_s2={fmt(np_degrees(profile.acceleration_limits))},",
            f"    jerk_limits_deg_s3={fmt(np_degrees(profile.jerk_limits))},",
            f"    position_limits_lower_deg={fmt(profile.position_limits_lower_deg)},",
            f"    position_limits_upper_deg={fmt(profile.position_limits_upper_deg)},",
            f"    max_payload_kg={payload},",
            f"    source={profile.source!r},",
            ")",
        ]
    )


def np_degrees(values: object) -> object:
    """``numpy.degrees`` without importing numpy at module scope (this module is
    otherwise stdlib-only, and is importable on a host with no extension built)."""
    import numpy as np

    return np.degrees(np.asarray(values))


def _report(facts: ControllerFacts) -> str:
    """The human-readable probe report."""
    lines = [f"controller {facts.ip}", f"  ftp banner        : {facts.ftp_banner.strip()}"]
    v = facts.version
    o = facts.order
    if v is not None:
        lines += [
            f"  model             : {v.model}",
            f"  serial (F number) : {v.serial}",
            f"  software edition  : {v.software_edition}",
            f"  root / boot       : {v.root_version} / {v.boot_monitor}",
            f"  servo / DCS       : {v.servo_code} / {v.dcs_version}",
        ]
    if o is not None:
        lines += [
            f"  order deliver ver : {o.deliver_version}   <- the P-level the gate bands on",
            f"  customer          : {o.customer}",
            f"  options           : {len(o.options)} ordered",
            f"  S636 external ctl : {'PRESENT' if facts.has_external_control else 'NOT FOUND'}"
            f" ({o.option_description(OPTION_EXTERNAL_CONTROL)})",
        ]
    for prog, why in (
        ("stream_motn", "Stream Motion TP program (J519)"),
        ("rmi_move", "RMI TP program (R912)"),
        ("gripdisp", "gripper dispatcher — site prerequisite, not shipped in the wheel"),
        ("grprun", "GRIPDISP RUN-fork launcher"),
    ):
        lines.append(f"  tp {prog:<12}   : {'present' if facts.has_tp_program(prog) else 'ABSENT'}  ({why})")

    limits = facts.limits
    if limits is not None:
        lines += [
            f"  joints            : {limits.ndof}",
            f"  velocity  (deg/s) : {[round(x, 3) for x in limits.velocity_deg_s[: limits.ndof]]}",
            f"  lower     (deg)   : {[round(x, 3) for x in limits.lower_deg[: limits.ndof]]}",
            f"  upper     (deg)   : {[round(x, 3) for x in limits.upper_deg[: limits.ndof]]}",
            f"  max payload (kg)  : {limits.max_payload_kg}",
            f"  J2/J3 envelope    : {'ACTIVE' if limits.jnt23_active else 'inactive (both bounds 0)'}",
            f"  active vs master  : {'agree' if not limits.disagreements() else limits.disagreements()}",
        ]
    else:
        lines.append("  joint limits      : NOT READ")
    for w in facts.warnings:
        lines.append(f"  WARNING           : {w}")
    return "\n".join(lines)


def main(argv: Sequence[str] | None = None) -> int:
    """``python -m airo_fanuc.controller_probe --ip <addr>``. Read-only."""
    import argparse

    ap = argparse.ArgumentParser(
        description="Read a FANUC controller's own account of itself over FTP. Read-only: "
        "fetches diagnostic files from md: and writes nothing.",
    )
    ap.add_argument("--ip", required=True, help="controller address")
    ap.add_argument("--user", default=DEFAULT_FTP_USER)
    ap.add_argument("--password", default=DEFAULT_FTP_PASSWORD)
    ap.add_argument("--timeout", type=float, default=15.0, help="per-operation FTP timeout (s)")
    ap.add_argument(
        "--no-limits",
        action="store_true",
        help=f"skip {MOTION_SYSVARS} (~650 kB, the only slow read) and report versions/options only",
    )
    ap.add_argument(
        "--emit-profile",
        action="store_true",
        help="print a RobotProfile as Python source, ready to paste into a project",
    )
    ap.add_argument(
        "--accel-from-velocity",
        type=float,
        default=DEFAULT_ACCEL_FROM_VELOCITY,
        help="acceleration clamp as a multiple of velocity (default %(default)s); the "
        "controller publishes no acceleration clamp, so this is a decision, not a reading",
    )
    ap.add_argument(
        "--jerk-from-accel", type=float, default=DEFAULT_JERK_FROM_ACCEL, help="default %(default)s"
    )
    args = ap.parse_args(argv)

    try:
        facts = probe_controller(
            args.ip,
            timeout_s=args.timeout,
            user=args.user,
            password=args.password,
            want_limits=not args.no_limits,
        )
    except ControllerProbeError as exc:
        print(f"probe failed: {exc}")
        return 1

    print(_report(facts))
    if args.emit_profile:
        try:
            profile, _ = profile_from_controller(
                args.ip,
                accel_from_velocity=args.accel_from_velocity,
                jerk_from_accel=args.jerk_from_accel,
                facts=facts,
            )
        except ControllerProbeError as exc:
            print(f"\ncannot emit a profile: {exc}")
            return 1
        print("\n" + "-" * 76)
        print(format_profile_source(profile))
    return 0


if __name__ == "__main__":  # pragma: no cover
    raise SystemExit(main())
