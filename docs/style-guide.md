# MEPA2002SAR Code Style Guide

The single source of truth for how code in this repository is laid out and written. If a comment or wiki page disagrees with this guide, this guide wins.

## Language

English everywhere — identifiers, comments, log messages, commit messages.

## Constants vs ROS parameters

A value belongs in `declare_parameter()` if a launch file (or YAML config, or `ros2 param set`) might want to override it per deployment. Otherwise it's a module-level `UPPER_SNAKE` constant.

**ROS parameter** — values that vary across robots, deployments, or operator tuning:
- Hardware identifiers — `device` paths, `i2c_bus`, `serial_port`, GPIO pin numbers.
- Tunable runtime values — display ranges, thresholds, frame IDs, publish rates if the operator might want to change them.
- Anything already overridden in a launch file's `parameters=[...]` block.

**Module constant** — values that are part of the code's contract, not its configuration:
- Physics constants — `ABSOLUTE_ZERO_C`, conversion factors.
- Format identifiers — encoding strings (`'mono16'`, `'bgr8'`), FOURCC codes.
- Topic names that the launch file expects via `remappings=` (don't reinvent remapping as a parameter).
- Hardware capabilities that are paired with a format choice — e.g. camera resolution paired with the FOURCC code.

When in doubt: if changing the value would require a recompile in any reasonable deployment, it's a parameter.

## Naming

Descriptive names carry the meaning that comments used to.

- **Variables and functions describe intent, not type or mechanics.** `target_linear_velocity` over `vx`, `wheel_circumference_m` over `wc`. Include units in the name when ambiguous: `_m`, `_rad`, `_hz`, `_s`, `_deg`.
- **Booleans read as questions.** `is_connected`, `has_valid_fix`, `should_publish_odom` — not `flag`, `status`, `check`.
- **Functions are verbs; classes are nouns.** `publish_wheel_odometry()`, not `wheel_odometry()`. `MecanumAllocator`, not `MecanumAllocation`.
- **Domain abbreviations are fine.** `imu`, `lidar`, `pwm`, `crc`, `rpm`, `pub`, `sub`, `msg`, `cmd` — everyone in ROS/robotics reads these. Invent-your-own abbreviations (`tgt`, `vel_cb`, `srl`) are not.
- **Callbacks named after what they handle.** `on_cmd_vel(msg)` preferred over `cmd_vel_callback(msg)`. Be consistent within a file.
- **Constants `UPPER_SNAKE`.** Private helpers prefixed `_`.
- **The name should make a comment unnecessary.** If you're tempted to write `# the offset from the IMU mount` above a variable called `offset`, rename it `imu_mount_offset_m` and delete the comment.

## Comments

Minimal. Only short `why` comments survive; `what` comments get deleted because the code already says what.

**One-line limit.** If an explanation needs more than one line, the explanation belongs in the wiki, not in the source. The code keeps a one-liner pointer.

Keep:
- Hidden constraints (`# MAXIM CRC8 polynomial — must match ESP32 firmware`)
- Hardware quirks (`# IMU returns yaw mirrored when mounted upside-down`)
- Workarounds with a reason (`# rclpy timer drifts under load; use clock-based scheduling instead`)
- Wiki pointers when the reason is non-obvious and too long for one line (`# Raw accel (gravity included) — required by MOLA-LO. See wiki: IMU/Gravity-Correction`)
- Section dividers (`# --- Odometry ---`) when the block has 2+ distinct logical groups. Use them to label the groups (parameters / timing / driver state / publishers), not as decoration. A small file doing one thing (load pins → publish in a loop) doesn't need them; a class that genuinely separates concerns does. Same rule for import groups: label `# stdlib` / `# third-party` / `# ROS` whenever there are 2+ groups present.

Delete:
- Restating the code (`# subscribe to cmd_vel` above `self.create_subscription(Twist, 'cmd_vel', ...)`)
- "Establish connection to UART", "Timer which sends data", "Parameters for robot's dimensions"
- Multi-line explanatory blocks — move the explanation to the wiki and leave a pointer.
- TODO/FIXME without an owner or date

No required docstrings except a one-line module docstring at the top of each file. Add function docstrings only when the function is non-obvious from name + signature.

## File template — Python ROS node

```python
#!/usr/bin/env python3
"""One-line description of what this node does."""

# --- Imports ---
# stdlib
import math
import time

# third-party
import serial

# ROS
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

# --- Constants ---
DEFAULT_SERIAL_PORT = '/dev/ttyUSB0'
DEFAULT_BAUD_RATE = 921600


class NodeName(Node):
    def __init__(self):
        super().__init__('node_name')

        # --- Parameters ---
        self.declare_parameter('serial_port', DEFAULT_SERIAL_PORT)
        ...

        # --- Publishers & subscribers ---
        self.cmd_vel_sub = self.create_subscription(...)
        ...

        # --- State ---
        self.x = 0.0
        ...

        # --- Timers ---
        self.write_timer = self.create_timer(0.05, self.send_serial_data)

    # --- Callbacks ---
    def on_cmd_vel(self, msg): ...

    # --- Helpers ---
    def _calculate_crc8(self, data): ...


def main(args=None):
    rclpy.init(args=args)
    node = NodeName()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

Order within `__init__`: parameters → publishers/subscribers → state → timers. Order within the class: `__init__` → callbacks → helpers.

**`main()` shape:** use the minimal form above by default. Add `try/finally` only when the node holds an external resource that must be released on exception (e.g. GPIO pins via `GPIO.cleanup()`, an open serial port). `KeyboardInterrupt` handling is unnecessary — `rclpy.spin` exits cleanly on Ctrl+C.

## Launch files

- Same import grouping (stdlib → third-party → ROS).
- Top-level constants for paths, topic names, default args.
- One `generate_launch_description()` with a clear return.
- Group nodes by subsystem with section comments when the file is long.

## URDF / xacro

- Consistent indentation (2 spaces).
- Macros named for what they describe (`<xacro:macro name="wheel" .../>`), parameters with units in the name.
- One `<link>` and one `<joint>` per logical part; avoid copy-paste blocks — extract a macro instead.

## C++ (ESP32 firmware)

Same spirit as Python:

- Headers grouped: standard, Arduino/framework, project headers.
- Section dividers with `// --- Name ---`.
- `why` comments only.
- Constants in `UPPER_SNAKE` at the top of the file.
- One class/concern per file when practical.

## Reference example

[`ros_gz_application/nodes/IMU.py`](../src/ros_gz_application/nodes/IMU.py) is the canonical example of this guide applied. When in doubt, look there first.
