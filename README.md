# single_uav_hover

A minimal ROS 2 package for flying a single PX4 vehicle through
arm → takeoff → hover → (optional horizontal nudge) → land, driven by
keyboard commands over OFFBOARD position setpoints. Built for PX4 SITL
first, with parameters designed to carry over to real hardware running
PX4's uXRCE-DDS client.

## Overview

The package ships two nodes that run as separate processes:

- **`hover_control`** — the flight state machine. Holds the PX4
  publisher/subscriber topics, runs a 10 Hz timer, and walks the
  vehicle through `idle → taking_off → hovering → landing → idle`.
- **`hover_commander`** — a small blocking `input()` loop that publishes
  single-digit commands to `/single_uav_hover/command`. Kept in its own
  terminal so its prompt isn't scrolled away by `hover_control`'s status
  logging.

```
hover_commander  --[/single_uav_hover/command (Int32)]-->  hover_control  --[OffboardControlMode, TrajectorySetpoint, VehicleCommand]-->  PX4 (uXRCE-DDS)
```

## Commands

Sent from the `hover_commander` terminal:

| Command | Effect | Valid from state |
|---|---|---|
| `1` | Request OFFBOARD + ARM, climb to `hover_altitude_m` | `idle` |
| `2` | Land (ramp to ground, disarm on touchdown) | `taking_off`, `hovering` |
| `3` | Step `+horizontal_step_m` in X | `hovering` |
| `4` | Step `+horizontal_step_m` in Y | `hovering` |

Horizontal targets are ramped toward at `horizontal_speed_mps`, never
jumped to directly. `Ctrl+C` on `hover_control` triggers an emergency
landing sequence (descend + disarm) regardless of current state, before
the process exits.

## State machine

- `idle` — on the ground, disarmed, setpoint stream already flowing
  (PX4 requires this before it will accept an OFFBOARD switch).
- `taking_off` — requests OFFBOARD, then ARM, then ramps `target_z`
  up to `-hover_altitude_m` (NED) at `climb_rate_mps`.
- `hovering` — holds hover altitude; ramps `target_x`/`target_y` toward
  the current goal set by commands `3`/`4`.
- `landing` — ramps `target_z` back to `0.0` at `land_descent_rate_mps`;
  disarms and returns to `idle` once close to the ground.

## Parameters

All declared on `hover_control`, overridable via `--ros-args -p`:

| Parameter | Default | Meaning |
|---|---|---|
| `namespace` | `/MAV4` | uXRCE-DDS topic prefix for this vehicle (`<namespace>/fmu/in/...`, `<namespace>/fmu/out/...`). Default matches the real-hardware vehicle this package was first flown on; override for SITL or any other vehicle to match its actual uXRCE-DDS client namespace. |
| `hover_altitude_m` | `0.5` | Hover height above the arming point, in meters (positive up). |
| `climb_rate_mps` | `0.3` | Takeoff climb rate. |
| `land_descent_rate_mps` | `0.3` | Landing descent rate. |
| `horizontal_speed_mps` | `0.2` | Rate at which `target_x`/`target_y` ramp toward a horizontal goal. |
| `horizontal_step_m` | `0.5` | Size of the step added to the goal by commands `3`/`4`. |

## Build

```bash
cd ~/ros2_ws
colcon build --packages-select single_uav_hover
source install/setup.bash
```

## Run

Needs PX4 (SITL or real hardware) and the Micro XRCE-DDS Agent already
running and bridged to the vehicle's `namespace`.

```bash
ros2 run single_uav_hover hover_control --ros-args -p namespace:=/MAV4
ros2 run single_uav_hover hover_commander   # separate terminal
```

For PX4 SITL, `namespace` typically needs to be overridden to match
whatever topic prefix that instance's uXRCE-DDS client publishes under
(e.g. `/px4_1` style namespacing) — it will not be `/MAV4` unless the
SITL instance is configured that way.

## Test

```bash
colcon test --packages-select single_uav_hover
colcon test-result --verbose
```

Runs the standard `ament_python` lint suite: `test_copyright.py`,
`test_flake8.py`, `test_pep257.py`.

## Notes / gotchas

- **QoS**: PX4 subscriptions use `BEST_EFFORT` + `TRANSIENT_LOCAL`,
  `KEEP_LAST` depth 1 to match PX4's uXRCE-DDS side. A mismatched QoS
  profile means subscriptions silently receive nothing.
- **Vehicle targeting**: `VehicleCommand.target_system` is sent as `0`
  (broadcast) rather than a specific system ID, since PX4 assigns
  `MAV_SYS_ID = instance + 1` rather than any ID this node tracks. The
  vehicle is still addressed correctly because commands travel over its
  own private `<namespace>/fmu/in/vehicle_command` topic.
- **Shutdown handling**: `rclpy.init()` disables the default SIGINT
  handler (`SignalHandlerOptions.NO`) so `Ctrl+C` raises a normal
  `KeyboardInterrupt` instead of tearing down the context immediately —
  this is what lets the landing sequence run before exit.
- **Dependency**: `px4_msgs` message definitions must match the PX4
  firmware/vehicle version in use, or subscriptions silently receive
  garbled or empty fields.

## License

TODO — not yet declared (see `package.xml`).
