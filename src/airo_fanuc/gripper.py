# SPDX-License-Identifier: Apache-2.0
"""Robotiq 2F-85 register protocol for the FANUC GRIP_DISP TP program.

The register contract is fixed by the TP program running on the controller, not
chosen here, so the constants below reproduce it exactly;
:class:`~airo_fanuc.gripper_worker.GripperWorker` consumes them and writes the
registers via :class:`~airo_fanuc.rmi_client.RmiClient`.

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

  * 0 → POSITION 220, FORCE 100 (light; least grip force, for rigid or
    easily-crushed objects)
  * 1 → POSITION 220, FORCE 150 (medium; the default)
  * else → POSITION 255, FORCE 255 (hard; full close at full force, for
    compressible objects that only hold once squeezed). We write the
    literal value 2 for this branch.

**Three discrete buckets per action, and nothing else.** There is no continuous
width command — R[3] carries a bucket index, not millimetres — and no width
feedback: the only gripper state readable over this protocol is R[1] (1 = the TP
program is mid-action, 0 = done), which is why a caller polls R[1] rather than
comparing a commanded width against a measured one.
"""

from __future__ import annotations

REG_CMD = 1  # command flag: 0=idle, 1=execute
REG_ACTION = 2  # action: 1=open, 2=close
REG_R3 = 3  # action-dependent modifier (see module docstring):
#   on open  → open-state selector
#   on close → close-force selector
# One register, two names for its two readings: callers reasoning about open
# widths spell it REG_OPEN_STATE, callers reasoning about close-force classes
# spell it REG_R3. Both resolve to R[3].
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

# Default close force when a caller does not specify one: the TP program's
# medium bucket (POSITION 220 / FORCE 150).
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
