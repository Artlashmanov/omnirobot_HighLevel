# High-level architecture and platform layer

The repository is split into layers so the same Pi5 high-level stack can later run different robot base form factors.

Current implemented platform: `omni4` — a 4-wheel omnidirectional base using the current STM32 CAN protocol.

## Layer model

```text
Web UI / autonomous software / Nav2
        |
        v
MANUAL/AUTO command mux
        |
        v
Platform command contract
        |
        v
Platform adapter/profile: omni4
        |
        v
Base driver / CAN protocol: stm32_omni_v1
        |
        v
STM32 motor controller firmware
```

The upper layers should not know how many wheels the base has or which CAN bytes the STM32 expects. They publish movement intent. The platform layer decides which commands are valid for this base and how they map to the base driver.

## Current platform profile

Runtime selection lives in `/etc/omni-robot/omni.env`:

```bash
ROBOT_PLATFORM=omni4
OMNI_PLATFORM_CONFIG=${OMNI_HOME}/config/platforms/omni4.json
```

The current profile is stored at:

```text
config/platforms/omni4.json
```

It defines:

- platform name and display name;
- motion interface: `discrete_mode_speed_v1`;
- CAN protocol: `stm32_omni_v1`;
- supported motion modes;
- wheel names;
- current web teleop button-to-mode mapping.

## Current command contract

For `omni4`, the internal platform command is JSON on `/omni/motion_cmd`:

```json
{"mode":"FORWARD","speed_pct":30}
```

Supported modes are defined by the platform profile:

- `STOP`
- `FORWARD`
- `BACKWARD`
- `LEFT`
- `RIGHT`
- `ROTATE_CCW`
- `ROTATE_CW`

`speed_pct` is clamped to `0..100`; `STOP` always forces `speed_pct=0`.

## What is already platform-aware

- `tools/cmd_mux_node.py` validates motion commands through `src/omni_pi/platforms.py`.
- `teleop_web/app.py` validates web commands through the same platform profile.
- `teleop_web/templates/index.html` reads `/api/platform` and uses the profile's button mapping.
- `src/ros2_ws/src/omni_bridge/omni_bridge/can_bridge_node.py` loads the selected profile and refuses unsupported CAN protocols.
- `install/verify-install.sh` prints the active platform profile.

## How a future platform should be added

Do not rewrite the web UI, mux, service install scripts, or common runtime environment.

Add a new platform in this order:

1. Add `config/platforms/<platform>.json`.
2. Add a platform adapter/driver if its `motion_interface` or `can_protocol` differs from `omni4`.
3. Add or update protocol documentation under `docs/`.
4. Set `ROBOT_PLATFORM=<platform>` and `OMNI_PLATFORM_CONFIG=...` in `/etc/omni-robot/omni.env`.
5. Run `./install/verify-install.sh`.

For example, a future 6x6 Ackermann base should probably use a different command contract such as speed plus steering angle. That should be a new adapter beside `omni4`, not a rewrite of the existing high-level stack.
