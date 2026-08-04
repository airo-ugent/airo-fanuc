# airo-fanuc documentation

Start with whichever question you actually have.

## Getting it running

| | |
|---|---|
| [**Portability**](portability.md) | **Will this work on my robot?** Four tiers, from FANUC-generic to not portable at all, plus a checklist for a new arm and the two questions only your controller can answer. |
| [**Configuration**](configuration.md) | Every field of `RobotProfile`, `DriverConfig` and `DriverPolicy`, with defaults and units — and how to derive a profile for your arm from the controller itself. |
| [**Safety and preconditions**](safety.md) | What must be true before you connect, the fault model, the ARM gate, and the limitations worth knowing before an operator is at the pendant. |
| [**Controller notes**](controller-notes.md) | Measured behaviour of the controller this driver was developed against — and the only place with the exact system-variable names, alarm texts, error IDs, recovery procedures and FTP prerequisites. Worth reading before a first bring-up, not only when working on the driver. |
| [`examples/README.md`](../examples/README.md) | The ordered bring-up ladder: eight steps from a no-motion connect to streamed servoing, each ending in a PASS/FAIL verdict. |

## Writing code against it

| | |
|---|---|
| [**API reference**](api.md) | Every public method, what blocks, what it returns, what it raises. The full `get_state()` key list with units, `MotionHandle` semantics, and the exception hierarchy. |
| [**Gripper**](gripper.md) | How to drive *your own* gripper: the completion contract, and the specification your Teach-Pendant dispatcher must satisfy. Our Robotiq 2F-85 is the worked example at the end. |
| [**Troubleshooting**](troubleshooting.md) | Symptom-first. Bring-up failures, refused commands, unexpected motion, and how to read the real-time numbers. |

## Working on the driver

| | |
|---|---|
| [**Architecture**](architecture.md) | The two protocols and why both are needed, the C++ layering, the thread map, and the path from a submit to the wire. |
| [**Invariants**](invariants.md) | The binding safety and motion invariants, each with the one-line symptom of breaking it. Read this before changing anything on the 8 ms path. |
| [**Development**](development.md) | Build, test, the sanitizer gate, the hardware-free test rig, and what a test here is expected to do. |
