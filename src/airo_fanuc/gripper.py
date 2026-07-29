# SPDX-License-Identifier: Apache-2.0
"""Robotiq 2F-85 register protocol for the FANUC GRIP_DISP TP program.

Ported verbatim (constants + R[3] action-dependent semantics) from the
battle-tested ``dries`` ``grocery_bot.robot.fanuc.gripper`` — the behavioral
ground truth. The register contract is a controller fact, not a design choice,
so it is reproduced here exactly; :class:`~airo_fanuc.gripper_worker.GripperWorker`
consumes these constants.

The gripper is wired to the FANUC controller's tool I/O.  A TP program
(``GRIPDISP``, on-disk name — the docs call it ``GRIP_DISP`` for
readability) runs in an infinite loop on the controller, polling numeric
register R[1]; when it sees R[1]=1 it reads R[2] for the action (1=open,
2=close) and R[3] for an action-dependent modifier, then calls the
corresponding Robotiq block and clears R[1] back to 0.

R[3] modifier semantics by action:

* **Open** (R[2]=1) — R[3] selects which open width:

  * 0 → POSITION 0   (~85 mm physical opening, fully open)
  * 1 → POSITION 75  (~60 mm physical opening, mid)
  * 2 → POSITION 150 (~35 mm physical opening, narrow)

* **Close** (R[2]=2) — R[3] selects close target position + force:

  * 0 → POSITION 220, FORCE 100 (light; for balls)
  * 1 → POSITION 220, FORCE 150 (medium; default for most items)
  * else → POSITION 255, FORCE 255 (hard; for compressible items
    like towels). We write the literal value 2 for this branch.

:class:`~airo_fanuc.gripper_worker.GripperWorker` writes those registers via
:class:`~airo_fanuc.rmi_client.RmiClient`.
"""

from __future__ import annotations

REG_CMD = 1  # command flag: 0=idle, 1=execute
REG_ACTION = 2  # action: 1=open, 2=close
REG_R3 = 3  # action-dependent modifier (see module docstring):
#   on open  → open-state selector
#   on close → close-force selector
# Alias mirroring the dries `robot/fanuc/gripper.py` register contract (the module
# P5b migrates onto this one), so consumers that spell R[3] by its open-state name
# keep resolving. Same register, two names for its two readings.
REG_OPEN_STATE = REG_R3

ACTION_OPEN = 1
ACTION_CLOSE = 2

OPEN_FULL = 0
OPEN_MID = 1
OPEN_NARROW = 2

VALID_OPEN_STATES = (OPEN_FULL, OPEN_MID, OPEN_NARROW)

# Close-force selector values written to R[3] *before* an ACTION_CLOSE
# trigger. See the module docstring for the position/force pairs the
# GRIP_DISP TP program applies for each value.
FORCE_LIGHT = 0
FORCE_MEDIUM = 1
FORCE_HARD = 2

VALID_CLOSE_FORCES = (FORCE_LIGHT, FORCE_MEDIUM, FORCE_HARD)

# Default close force when a caller does not specify one. Matches the
# medium / for-most-items setting in the TP program.
DEFAULT_CLOSE_FORCE = FORCE_MEDIUM


__all__ = [
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
