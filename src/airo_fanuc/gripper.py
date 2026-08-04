# SPDX-License-Identifier: Apache-2.0
"""The register-dispatcher gripper protocol, and the Robotiq 2F-85 preset for it.

A FANUC controller offers no gripper API. What it offers is numeric registers and
the ability to run a Teach-Pendant program, so this driver actuates a gripper the
only way available over RMI: it writes registers, and a TP program running on the
controller watches them and does the work. That indirection is what makes the
mechanism generic — the driver never knows what is bolted to the flange, only which
registers a dispatcher is watching and which values it understands.

:class:`RegisterGripperProtocol` is that contract, as data. It says which three
registers to use, which value in the action register means open and which means
close, and which modifier values each verb accepts.
:class:`~airo_fanuc.gripper_worker.GripperWorker` executes it;
:data:`ROBOTIQ_2F85` fills it in for the one gripper this package has been
exercised against. A site with a different gripper writes its own dispatcher and
its own protocol instance and changes nothing else.

What the protocol cannot express, because the mechanism cannot: anything continuous,
and any feedback. A modifier is a bucket index chosen by the dispatcher's author, not
a width in millimetres or a force in newtons, and the only readable gripper state is
the trigger register — set, meaning the dispatcher is mid-action; clear, meaning it
finished. ``docs/gripper.md`` states the TP-side half of the contract for someone
writing a dispatcher for their own gripper.
"""

from __future__ import annotations

from dataclasses import dataclass

# ---------------------------------------------------------------------------
# The generic contract
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class RegisterGripperProtocol:
    """Which registers a dispatcher TP program watches, and what may be written to them.

    Frozen: one instance is shared by the worker, the bring-up liveliness probe and
    the preflight TP-program check, and all three must be reasoning about the same
    registers.

    The three-register shape is not a convention this driver invented, it is what a
    poll loop on a TP program can act on: one register to say *something is pending*
    (the only value the driver sets and the dispatcher clears), one to say *which
    verb*, and one to carry the verb's single argument. Splitting the verb from its
    argument is what lets the dispatcher read a complete command in one scan.
    """

    #: Trigger. The driver writes 1 here LAST, after the other two are settled; the
    #: dispatcher clears it to 0 when the physical action has finished. Nothing else
    #: writes it, in either direction — it is the whole completion handshake.
    trigger_reg: int
    #: Which verb. Read by the dispatcher on the scan that sees the trigger set.
    action_reg: int
    #: The verb's argument, meaning whatever the dispatcher's author decided it means
    #: for that verb — see :attr:`open_modifiers` / :attr:`close_modifiers`.
    modifier_reg: int

    #: Values written to :attr:`action_reg` for the two verbs the driver exposes.
    open_action: int
    close_action: int

    #: Every modifier the dispatcher accepts for each verb. A value outside these is
    #: refused synchronously rather than written: the dispatcher has no way to report
    #: "I did not understand that", so an unknown modifier is actuated as whatever its
    #: `else` branch does.
    open_modifiers: tuple[int, ...]
    close_modifiers: tuple[int, ...]

    #: What each verb means with no argument given.
    default_open_modifier: int
    default_close_modifier: int

    #: The TP programs this protocol is served by, both of which live on controller
    #: flash. ``launcher_program`` is what bring-up calls; it must RUN-fork
    #: ``dispatcher_program`` and return (see ``deploy/GRPRUN.LS`` and
    #: ``docs/gripper.md`` for why a fork and not a call). Preflight checks both are
    #: installed before it lets a gripper-enabled session proceed.
    launcher_program: str
    dispatcher_program: str

    #: How long the dispatcher takes to answer, which is a property of the dispatcher and
    #: not of this driver. The defaults suit a gripper that strokes in well under a second;
    #: a tool that takes longer than ``dispatch_timeout_s`` to finish would otherwise report
    #: a spurious timeout on every command, and bring-up builds the worker, so there is no
    #: other moment at which a caller could raise it.
    #:
    #: * ``trigger_settle_s`` — delay before the first poll, so it cannot read the
    #:   pre-trigger 0 and report done instantly.
    #: * ``poll_hz`` — how often the trigger register is read.
    #: * ``dispatch_timeout_s`` — the bound on one command. Longer than the slowest motion
    #:   the dispatcher performs, or every command times out.
    trigger_settle_s: float = 0.1
    poll_hz: float = 20.0
    dispatch_timeout_s: float = 5.0

    #: For error messages and the bring-up log — the driver never branches on it.
    name: str = "unspecified"

    @property
    def tp_programs(self) -> tuple[str, ...]:
        """The two program names, lowercased, as the preflight check compares them."""
        return (self.dispatcher_program.lower(), self.launcher_program.lower())

    def modifiers_for(self, action: int) -> tuple[int, ...]:
        """The accepted modifiers for an action-register value."""
        if action == self.open_action:
            return self.open_modifiers
        if action == self.close_action:
            return self.close_modifiers
        raise ValueError(f"{self.name}: {action} is neither the open nor the close action")


# ---------------------------------------------------------------------------
# The Robotiq 2F-85 preset
# ---------------------------------------------------------------------------
#
# The register numbers and action codes below are fixed by the GRIPDISP TP program
# running on our controller, not chosen here. GRIPDISP loops on R[1]; when it sees
# R[1]=1 it reads R[2] for the verb and R[3] for the modifier, calls the matching
# Robotiq block, and clears R[1].

REG_CMD = 1  # trigger: 0=idle, 1=execute
REG_ACTION = 2  # action: 1=open, 2=close
REG_R3 = 3  # action-dependent modifier: open → width bucket, close → force class
# One register, two names for its two readings: callers reasoning about open widths
# spell it REG_OPEN_STATE, callers reasoning about close-force classes spell it
# REG_R3. Both resolve to R[3].
REG_OPEN_STATE = REG_R3

ACTION_OPEN = 1
ACTION_CLOSE = 2

# Open-width buckets. The mm figures are the physical opening the Robotiq reaches for
# the POSITION value GRIPDISP sends; they are here to make the buckets meaningful to a
# caller, not because anything reads them.
OPEN_FULL = 0  # POSITION 0   → ~85 mm, fully open
OPEN_MID = 1  # POSITION 75  → ~60 mm
OPEN_NARROW = 2  # POSITION 150 → ~35 mm

VALID_OPEN_STATES = (OPEN_FULL, OPEN_MID, OPEN_NARROW)

# Close target/force pairs. The classes are ordered by grip force, and the choice is
# about the object: a rigid or easily-crushed part wants LIGHT, a compressible one only
# holds once squeezed and wants HARD.
FORCE_LIGHT = 0  # POSITION 220, FORCE 100
FORCE_MEDIUM = 1  # POSITION 220, FORCE 150
FORCE_HARD = 2  # POSITION 255, FORCE 255 — full close at full force

VALID_CLOSE_FORCES = (FORCE_LIGHT, FORCE_MEDIUM, FORCE_HARD)

#: Default close force when a caller does not name one.
DEFAULT_CLOSE_FORCE = FORCE_MEDIUM

#: The shipped preset: a Robotiq 2F-85 on the controller's tool I/O, driven by the
#: GRIPDISP dispatcher. This is the one gripper the package has been exercised
#: against, and it is the default only so that construct-and-go works on our cell —
#: a different gripper supplies its own instance through
#: :attr:`~airo_fanuc.config.DriverPolicy.gripper_protocol`.
ROBOTIQ_2F85 = RegisterGripperProtocol(
    name="Robotiq 2F-85 via GRIPDISP",
    trigger_reg=REG_CMD,
    action_reg=REG_ACTION,
    modifier_reg=REG_R3,
    open_action=ACTION_OPEN,
    close_action=ACTION_CLOSE,
    open_modifiers=VALID_OPEN_STATES,
    close_modifiers=VALID_CLOSE_FORCES,
    default_open_modifier=OPEN_FULL,
    default_close_modifier=DEFAULT_CLOSE_FORCE,
    launcher_program="GRPRUN",
    dispatcher_program="GRIPDISP",
)


__all__ = [
    "RegisterGripperProtocol",
    "ROBOTIQ_2F85",
    "REG_CMD",
    "REG_ACTION",
    "REG_R3",
    "REG_OPEN_STATE",
    "ACTION_OPEN",
    "ACTION_CLOSE",
    "OPEN_FULL",
    "OPEN_MID",
    "OPEN_NARROW",
    "VALID_OPEN_STATES",
    "FORCE_LIGHT",
    "FORCE_MEDIUM",
    "FORCE_HARD",
    "VALID_CLOSE_FORCES",
    "DEFAULT_CLOSE_FORCE",
]
