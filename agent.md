# Agent Guide — control_your_robot

This branch is a shared-memory robot runtime. Devices run in child processes. Agents attach to a running service and talk to components by name.

`main` is the collect / deploy stack. Do not copy `my_robot/`, `example/`, `tools/`, or `task_env/` onto this branch unless asked. `developer_README.md` only points back to `main`.

## Project purpose

`cyr` starts the service, publishes each component's latest observation, and accepts commands. The runtime core does not import concrete drivers. The class path lives in YAML and is loaded inside the child process.

Hardware is optional. `config/runtime/test_robot.yml` uses in-process test devices. `config/runtime/marvin_lab.yml` is the Marvin lab bench.

`robot.harness` is the layer a policy or agent uses on top of `Session`: aligned observations, an action contract, and an episode log. The gate and joint tracker run inside the component process, not in the client.

## Layout

| Path | Role |
|------|------|
| `src/robot/runtime/` | Shared-memory bus, supervisor, session API, `cyr` CLI, HMI |
| `src/robot/harness/` | Body model, pre-command gate, joint tracking, action contract, observation, episode |
| `src/robot/controller/` | Motion devices. `class_path` under `robot.controller` accepts commands |
| `src/robot/sensor/` | Observation devices. Commands sent to them are rejected by `Session` |
| `config/runtime/` | Service configs |
| `config/bodies/` | URDF-backed body descriptions |
| `third_party/` | Vendor SDKs that stay in-tree |

## Conventions

- End-effector field is `eef`: xyz in meters plus quaternion **wxyz**.
- Joint commands are radians, in the same order `get()` publishes.
- Hand joints are 20 values: thumb, index, middle, ring, pinky, each flex, abd, pip, dip.
- `accepted: true` means the node has popped the command. With `motion.enabled`, it also means feedback is inside `tol_rad`.
- A gate or tracking rejection holds that component until `{"op":"release"}`. It does not latch service safety.
- A raised exception from `move` latches safety and stops the service. Read failures stay soft.
- `class_path` containing `.sensor.` does not accept commands.
- Debug level for the older prints: `INFO_LEVEL` = `DEBUG` | `INFO` | `ERROR`.
- Do not commit `Log/`, `save/`, `data/`, or device logs.

## Devices and servo modes

Onboarding a device follows `.cursor/skills/add-device/SKILL.md`.

Standard arm mode names are `position`, `mit`, `gravity_compensation`, and `impedance`. `setup.control_mode` is passed into `set_up`. Each arm declares `supported_modes`. A name outside that list raises, and the error lists the modes that device implements. Publish `control_mode` when a toolkit needs to read the active mode.

`servo_step(dt)` runs in the component loop, before `get()`, when the class defines it. `dt` is `1/rate_hz` for that component. An arm whose SDK servos internally switches mode once and does not integrate a second copy on the host.

Toolkits live under `toolkits/<name>/agent.md`. Read that file and run its check before teaching the steps.

## Runtime

- One latest-value shared-memory segment per service. Values are `msgpack_numpy`.
- One process per component, except Marvin arms that share `robot_ip`. Those run in one process so they share one UDP socket.
- Observations are each component's latest frame plus `timestamp_ns`. Cross-component alignment is `RobotHarness.get_obs()`, which reports `skew_ms`.
- Commands are a ring of 8. When it is full, the oldest command is dropped. A tracked move does not pop the next command until it finishes or times out.
- An abnormal component exit latches safety, safe-stops the other devices, rejects later commands, and shuts the service down.
- Several services can be live. `runtime.json` stores a `services` list and still accepts the old single-object file.
- Discovery record: `$XDG_STATE_HOME/cyr/runtime.json` (default `~/.local/state/cyr/runtime.json`). Logs: `cyr.log`, device stdout in `driver.log`.

## Config

Top-level keys: `name`, `shm_name`, `cmd_capacity` (default 8), `components`. Unknown keys are rejected.

Component keys: `name`, `class_path`, `class_name`, `args`, `setup`, `publish`, `rate_hz`, `max_bytes`, and optional `body`, `gate`, `motion`. Control parameters belong in `setup`, which is passed to `set_up(**setup)`.

`body` is a path relative to the runtime YAML. `gate` checks limits, one-shot step size or the whole tracked path, clearance, and xyz FK error. Clearance parks the other arms at `home`. `motion.enabled` interpolates joints and waits for feedback. IK solves xyz only. The quaternion is not a target.

## CLI

Install this repo into the environment you use to run `cyr` (`pip install -e .`). `/usr/bin/cyr` on Debian is unrelated.

```bash
cyr start --config config/runtime/test_robot.yml
cyr status
cyr snapshot
cyr echo left_arm
cyr call left_arm '{"op":"reset"}' --timeout 15
cyr call left_arm '{"joint":[0, 1.3, -1, 0, 0.5, 0]}'
cyr call left_arm '{"op":"release"}'
cyr hmi
cyr stop
```

`cyr snapshot` and `cyr call` print one JSON line. `call` exits 0 only when `accepted` is true. `hmi` does not start the service. It shows data only when exactly one service is running. `q` leaves the UI. `x` then `y` stops every service. `restart` requires exactly one live service. `stop` stops all of them.

The CLI cannot choose among several services. Use `Session.attach(name)`.

## Session API

```python
from robot.runtime.session import Session

session = Session.attach()          # name or shm_name when more than one is live
session.describe()
session.snapshot()                  # arrays longer than 8, and images, are summarized
session.read_raw("left_arm")        # full arrays, including pixels
session.reset("left_arm")
session.call("left_arm", {"joint": [0, 1.3, -1, 0, 0.5, 0]}, timeout=15)
session.close()
```

The first wait is `timeout`. After the node reports `motion_active` for that seq, the wait becomes `motion_timeout_s + 0.5`.

## Harness

```python
from robot.harness import RobotHarness

robot = RobotHarness(session, mode="joint", control_hz=15, max_skew_ms=80)
robot.get_obs()
robot.take_action({"left_arm_joint": [...], "left_ee_joint_state": [...]})
robot.step(action)
robot.finish_episode(True, "done")
```

`mode` is `joint` or `eef`. Training keys are `left_arm_joint`, `left_ee_pose`, `left_ee_joint_state` / `left_hand_joint`, and the same names on the right. Direct `left_arm: {"joint": ...}` also works.

Body kinematics need `pinocchio` and the URDF path in `config/bodies/marvin.yml`. Both are outside the default install. Tests skip when they are missing.

## Marvin lab

`config/runtime/marvin_lab.yml`, shared memory `cyr_marvin_lab`.

| Name | Kind | Notes |
|------|------|-------|
| `left_arm`, `right_arm` | Marvin controller | Same `192.168.1.190`, arms A and B, one process. Default `control_mode: impedance`. Gate and tracking are on. `reset` uses the body home. Safety stop holds feedback and does not disable |
| `left_hand`, `right_hand` | Wuji Hand 2 | UDP `192.168.1.110:7447` and `192.168.1.111:7447`. Twenty joints. Internal step `max_joint_step_rad` 0.08. Safety stop disables the hand |
| `left_glove`, `right_glove` | Moxian sensor | `/dev/moxian_left`, `/dev/moxian_right`. Raw `pressure` |
| `cam_head`, `cam_left_wrist`, `cam_right_wrist` | Orbbec sensor | Color only |

Wuji SDK: `CYR_WUJI_SITE`, or the sibling `xone_new` env when the current prefix is not already named `xone_new`. Marvin SDK: `MARVIN_SDK_ROOT`, otherwise the sibling `X-Robot-Pipeline_deploy` checkout.

## Tests

From the repo root:

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python -m pytest src/robot/runtime/tests src/robot/harness/tests -q --tb=line
```

Do not use a Python that auto-loads ROS pytest plugins.
