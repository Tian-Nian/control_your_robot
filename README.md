[![中文](https://img.shields.io/badge/中文-简体-blue)](./README_CN.md)
[![English](https://img.shields.io/badge/English-English-green)](./README.md)

# control_your_robot

This branch runs robot controllers and sensors as a shared-memory service. A client reads each component's latest observation and sends commands by component name.

```bash
pip install -e .
cyr start --config config/runtime/test_robot.yml
cyr snapshot
cyr stop
```

`cyr` must be the console script from this install. On Debian, `/usr/bin/cyr` is a different program.

## Commands

| Command | What it does |
|---------|----------------|
| `cyr start --config FILE` | Start the service in the background |
| `cyr serve FILE` | Run in the foreground |
| `cyr stop` | Safe-stop devices and stop the service |
| `cyr restart` | Restart with the last config |
| `cyr status` | Show whether the service is running |
| `cyr hmi` | Open the terminal status view. It does not start the service |
| `cyr echo NAME` | Print one component's latest observation |
| `cyr snapshot` | Print every component as one JSON line |
| `cyr call NAME JSON` | Send one command. Exit 0 means the node accepted it |

```bash
cyr call left_arm '{"joint":[0,0,0,0,0,0]}'
cyr call left_arm '{"op":"reset"}'
cyr call left_arm '{"joint":[0,0,0,0,0,0]}' --delta
```

`accepted: true` means the component process has taken the command. It does not mean the arm or hand has finished moving.

Python:

```python
from robot.runtime.session import Session

session = Session.attach()
session.snapshot()
session.call("left_arm", {"joint": [0, 0, 0, 0, 0, 0]})
session.reset("left_arm")
session.close()
```

Images in a snapshot are summarized as shape, dtype, and nbytes.

## Config

See `config/runtime/test_robot.yml`. Top-level keys are `name`, `shm_name`, `cmd_capacity`, and `components`. Each component has `name`, `class_path`, `class_name`, `args`, `setup`, `publish`, `rate_hz`, and `max_bytes`. `setup` is passed to `set_up`.

A class under `robot.sensor` publishes only. A class under `robot.controller` also accepts `move` and `reset`.

The service writes `$XDG_STATE_HOME/cyr/runtime.json` and appends logs to `$XDG_STATE_HOME/cyr/cyr.log`.

## Marvin lab

`config/runtime/marvin_lab.yml` registers the bench:

- `left_arm` / `right_arm`: one Marvin controller at `192.168.1.190`. They share one process. Startup uses joint impedance (`K` 16/8, `D` 1.8, vel/acc 50) and the existing tool dynamics. Switch with `{"control_mode":"position"}` or `{"control_mode":"impedance"}`. Joints are radians. `eef` is xyz meters plus quaternion wxyz.
- `left_hand` / `right_hand`: Wuji Hand 2 over UDP, 20 joints in the published order. The process keeps streaming the latest goal. Each tick moves at most `max_joint_step_rad`.
- `left_glove` / `right_glove`: Moxian tactile sensors, raw pressure.
- `cam_head` / `cam_left_wrist` / `cam_right_wrist`: Orbbec color cameras.

```bash
cyr start --config config/runtime/marvin_lab.yml
cyr call left_arm '{"control_mode":"position"}'
cyr call left_hand '{"joint":[0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]}'
```

A failed motion command latches safety and stops the whole service. Stopping the service holds the Marvin arms and disables the Wuji hands.

## Tests

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python -m pytest src/robot/runtime/tests -q --tb=line
```

## Directory

| Path | Content |
|------|---------|
| `src/robot/runtime/` | Service, shared memory, session API, HMI |
| `src/robot/controller/` | Arm, mobile, dexterous-hand, and test controllers |
| `src/robot/sensor/` | Cameras, tactile sensors, and test sensors |
| `config/runtime/` | `test_robot.yml`, `marvin_lab.yml` |
| `third_party/` | Vendor libraries |
| `agent.md` | Notes for coding agents |

# WECHAT

<p align="center">
  <img src="imgs/Wechat.jpg" alt="wechat_group" width="400">
  <img src="imgs/myWechat.jpg" alt="my_wechat" width="400">
</p>

If the WeChat group has expired, add the personal account to join.
