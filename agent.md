# Agent Guide — control_your_robot

This branch is a shared-memory robot runtime. Devices run in child processes. Agents attach to a running service and talk to components by name.

## Project purpose

`cyr` starts the service, publishes each component's latest observation, and accepts fire-and-forget commands. The runtime core does not import concrete drivers. The class path lives in YAML and is loaded inside the child process.

Hardware is optional. `config/runtime/test_robot.yml` uses in-process test devices. `config/runtime/marvin_lab.yml` is the Marvin lab bench.

## Layout

| Path | Role |
|------|------|
| `src/robot/runtime/` | Shared-memory bus, supervisor, session API, `cyr` CLI, HMI |
| `src/robot/controller/` | Motion devices. `class_path` under `robot.controller` accepts commands |
| `src/robot/sensor/` | Observation devices. Commands sent to them are dropped |
| `config/runtime/` | Service configs |
| `third_party/` | Vendor SDKs that stay in-tree |

There is no `my_robot/`, `example/`, `tools/`, or `task_env/` on this branch. Do not recreate the old collect / deploy stack unless asked.

## Conventions

- End-effector field is `eef`: xyz in meters plus quaternion **wxyz**.
- Joint commands are radians, in the same order `get()` publishes.
- A command is accepted when the node has popped it. That is not the same as the device having reached the pose.
- `class_path` containing `.sensor.` does not accept commands. Controllers do.
- Debug level: `INFO_LEVEL` = `DEBUG` | `INFO` | `ERROR`.
- Do not commit `Log/`, `save/`, `data/`, or device logs.

## Runtime

- One latest-value shared-memory segment per service. Values are `msgpack_numpy`.
- One process per component, except Marvin arms that share `robot_ip`. Those run in one process so they share one UDP socket.
- Observations are each component's latest frame plus `timestamp_ns`. There is no cross-component sync.
- Commands are a ring of 8. When it is full, the oldest command is dropped.
- An abnormal component exit or a failed motion command latches safety, safe-stops the other devices, rejects later commands, and shuts the service down. Read failures stay soft.
- Discovery record: `$XDG_STATE_HOME/cyr/runtime.json` (default `~/.local/state/cyr/runtime.json`). Logs: `cyr.log`, device stdout in `driver.log`.

## Config

Top-level keys: `name`, `shm_name`, `cmd_capacity` (default 8), `components`.

Component keys: `name`, `class_path`, `class_name`, `args`, `setup`, `publish`, `rate_hz`, `max_bytes`. Unknown keys are rejected. Control parameters belong in `setup`, which is passed to `set_up(**setup)`.

## CLI

Install this repo into the environment you use to run `cyr` (`pip install -e .`). `/usr/bin/cyr` on Debian is unrelated.

```bash
cyr start --config config/runtime/test_robot.yml
cyr status
cyr snapshot
cyr echo left_arm
cyr call left_arm '{"joint":[0,0,0,0,0,0]}'
cyr call left_arm '{"op":"reset"}'
cyr hmi
cyr stop
```

`cyr snapshot` and `cyr call` print one JSON line. `call` exits 0 only when the command was accepted. `hmi` does not start the service. `q` leaves the UI. `x` then `y` stops the service.

## Session API

```python
from robot.runtime.session import Session

session = Session.attach()
session.describe()
session.snapshot()
session.call("left_arm", {"joint": [0, 0, 0, 0, 0, 0]})
session.reset("left_arm")
session.close()
```

`snapshot` summarizes images as shape, dtype, and nbytes. It does not inline pixel buffers.

## Marvin lab

`config/runtime/marvin_lab.yml`, shared memory `cyr_marvin_lab`.

| Name | Kind | Notes |
|------|------|--------|
| `left_arm`, `right_arm` | Marvin controller | Same `192.168.1.190`, arms A and B, one process. Default `control_mode: impedance` with the deploy K/D, vel/acc 50, and per-arm `tool_dyn`. `{"control_mode":"position"}` switches to joint position mode. `reset` goes to the deploy home joints. Safety stop holds the current feedback and does not disable. |
| `left_hand`, `right_hand` | Wuji Hand 2 controller | UDP `192.168.1.110:7447` and `192.168.1.111:7447`. Twenty joints in the published model order. The hand streams the latest goal at `rate_hz`; each tick moves at most `max_joint_step_rad` (0.08). Safety stop disables the hand. |
| `left_glove`, `right_glove` | Moxian sensor | `/dev/moxian_left`, `/dev/moxian_right`. Publishes raw `pressure`. |
| `cam_head`, `cam_left_wrist`, `cam_right_wrist` | Orbbec sensor | Color only. |

Wuji SDK: if the current env is not `xone_new`, the hand process loads that env's `wuji-sdk`. Override with `CYR_WUJI_SITE`. Marvin SDK root: `MARVIN_SDK_ROOT`, otherwise the sibling `X-Robot-Pipeline_deploy` checkout.

Send absolute `joint` targets at or below the component `rate_hz`. A faster burst can queue up to 8 commands; each one takes its own step before the next replaces the goal.

## Tests

From the repo root:

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 /path/to/venv/bin/python -m pytest src/robot/runtime/tests -q --tb=line
```

Do not use a Python that auto-loads ROS pytest plugins.
