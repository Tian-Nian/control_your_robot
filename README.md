[![中文](https://img.shields.io/badge/中文-简体-blue)](./README_CN.md)
[![English](https://img.shields.io/badge/English-English-green)](./README.md)

# control_your_robot

This branch runs robot controllers and sensors as a shared-memory service. A client reads each component's latest observation and sends commands by component name. `main` still has the collect / deploy stack. This branch does not.

```bash
pip install -e .
cyr start --config config/runtime/test_robot.yml
cyr snapshot
cyr stop
```

`cyr` must be the console script from this install. On Debian, `/usr/bin/cyr` is a different program. Check with `which cyr`.

Hardware is optional. `config/runtime/test_robot.yml` uses in-process test devices. `config/runtime/piper_sim.yml` is the Piper simulator in position mode. Gravity and impedance use `piper_sim_gravity.yml` and `piper_sim_impedance.yml`. `config/runtime/marvin_lab.yml` is the Marvin lab bench.

## Commands

| Command | What it does |
|---------|----------------|
| `cyr start --config FILE` | Start one service in the background. The same `shm_name` is not started twice |
| `cyr serve FILE` | Run in the foreground |
| `cyr stop` | Safe-stop every running service and remove it |
| `cyr restart` | Restart when exactly one service is running, using its last config |
| `cyr status` | List running services. Exit 3 means none |
| `cyr hmi` | Terminal status view. It does not start the service, and it attaches only when exactly one service is running |
| `cyr echo NAME` | Print one component's latest raw observation. Images stay summarized |
| `cyr snapshot` | Print every component as one JSON line. Long arrays are summarized |
| `cyr call NAME JSON` | Send one command. Exit 0 only when `accepted` is true |

```bash
cyr call left_arm '{"op":"reset"}'
cyr call left_arm '{"joint":[0, 1.3, -1, 0, 0.5, 0]}'
cyr call left_arm '{"joint":[0, 0.05, 0, 0, 0, 0]}' --delta
cyr call left_arm '{"op":"release"}'
cyr call left_arm '{"control_mode":"gravity_compensation"}'
```

`cyr hmi`: `q` leaves the UI. `x`, then `y` or Enter, stops every running service. `n` cancels that prompt. Up / down or `k` / `j` move the selection.

The CLI attaches to the only live service. If more than one is running, `echo`, `snapshot`, and `call` fail and ask you to use `Session.attach(name)` from Python.

## What `accepted` means

`cyr call` and `Session.call` return:

```json
{"accepted": true, "seq": 3, "error": "", "state": {}}
```

`accepted: true` means the component process has taken the command and, when joint tracking is enabled, the feedback is inside tolerance. It is not a promise that a later sensor frame still matches the goal.

`accepted: false` with a gate or tracking error means this component is holding its current command. Later commands to that component are refused until `{"op":"release"}`. The rest of the service keeps running. A driver exception is different: it latches safety and shuts the whole service down. Read failures stay soft and show up in the component `error` field.

While a tracked move is active, the node does not take the next command. Commands stay in the ring until the move finishes, times out, or the ring drops the oldest one.

## Python session

```python
from robot.runtime.session import Session

session = Session.attach()                 # or Session.attach("marvin_lab")
session.describe()
session.snapshot()                         # images and arrays longer than 8 are summarized
session.read_raw("cam_head")               # pixels kept
session.reset("left_arm")
session.call("left_arm", {"joint": [0, 1.3, -1, 0, 0.5, 0]}, timeout=15)
session.close()
```

`attach(name)` matches the service `name` or `shm_name`. With one service, the name can be omitted. With several, it is required.

`call` first waits up to `timeout` (default 2 seconds) for the node to take the command. If that command starts joint tracking, the wait extends to the component `motion.timeout_s` plus half a second. `Session.reset` waits 15 seconds by default. `cyr call` does the same for `{"op":"reset"}` when `--timeout` is left at its default. Reset returns after the controller has sent the whole trajectory. Marvin arms in the lab config use 12 seconds of tracking, so pass `--timeout 15` or `timeout=15` for a long move.

## Policy harness

`robot.harness` sits on a `Session`. It aligns observations, maps a training action onto component commands, and can record an episode.

```python
from robot.harness import RobotHarness
from robot.runtime.session import Session

session = Session.attach("marvin_lab")
robot = RobotHarness(session, mode="joint", control_hz=15, max_skew_ms=80)
observation = robot.get_obs()
robot.start_episode("episode.json")
outcome = robot.step({
    "left_arm_joint": [0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    "left_ee_joint_state": [0.0] * 20,
})
robot.finish_episode(True, "placed")
session.close()
```

`get_obs()` keeps image pixels. It reports `skew_ms`, `within_skew`, and `missing`. When a component config points at a body file, the observation also has `flange` and `palm` poses: xyz meters plus quaternion wxyz.

`mode="joint"` refuses an arm target that only has `eef`. `mode="eef"` still sends the `eef` field; the runtime solves joints when that component has a body model.

These action fields map onto components:

| Field | Command |
|-------|---------|
| `left_arm` / `right_arm` | `{"joint": ...}` and/or `{"eef": ...}` |
| `left_arm_joint`, `right_arm_joint` | arm `joint` |
| `left_ee_pose`, `right_ee_pose` | arm `eef`, length 7, xyz + wxyz |
| `left_hand` / `right_hand`, `left_hand_joint`, `right_hand_joint`, `left_ee_joint_state`, `right_ee_joint_state` | hand `joint`, length 20 |

Hand order is thumb, index, middle, ring, pinky. Each finger is flex, abd, pip, dip. That is the order the hand controller publishes.

`Episode` writes JSON. Arrays with rank 2 or higher, or more than 32 numbers, keep shape and dtype only. A 20-joint hand vector is stored in full.

## Config

See `config/runtime/test_robot.yml`. Unknown keys are rejected.

Top level: `name`, `shm_name`, `cmd_capacity` (default 8), `components`.

Each component: `name`, `class_path`, `class_name`, `args`, `setup`, `publish`, `rate_hz`, `max_bytes` (at least 64). Optional: `body`, `gate`, `motion`.

`setup` is passed to `set_up`. `args` construct the class. `publish` selects fields from `get()`. A class path containing `.sensor.` publishes only. A controller accepts `move` and `reset`.

`body` is a path, relative to the runtime YAML, to a body description such as `config/bodies/marvin.yml`.

`gate` checks a joint command before it is sent:

| Key | Meaning |
|-----|---------|
| `arm` | arm name inside the body file, such as `left` |
| `lower`, `upper` | used when there is no body model |
| `max_step_rad` | largest joint change allowed in one command when tracking is off. Default 0.2 |
| `min_clearance_m` | minimum distance for the configured collision pairs |
| `max_fk_error_m` | allowed xyz error after IK or against a supplied `eef`. Default 0.008 |

Clearance uses the commanded arm and holds every other arm at its `home` pose. It does not use the other arm's live joints.

`motion`, when `enabled: true`, walks the joint goal in steps of `max_step_rad` (default 0.05) and waits until feedback is inside `tol_rad` (default 0.02) or `timeout_s` expires. The whole joint-space segment is checked against limits and clearance before the first step. A timeout holds the component; it does not latch service safety.

`{"op":"release"}` clears that hold. `{"op":"reset"}` calls the controller `reset`. Do not home with `{"joint":[0,0,...]}`: in position mode that command sets the goal to zero in one step.

Reset runs a smootherstep from the current joints, peak speed 1.7 rad/s, at least 0.8 s, and calls `set_joint` on each sample. If the arm is already there, it sends the current joints once. An arm sitting in impedance or gravity compensation is switched to `position` first, because some `set_joint` implementations do not move the joints in those modes. Optional `start_state` is the joint goal. When it is omitted, a body file supplies `home`; otherwise the controller uses its own `_home` or `_init_pose`, and only then zeros. A configured gate checks limits and clearance along that path before the first sample.

```bash
cyr call left_arm '{"op":"reset"}'
cyr call left_arm '{"op":"reset","start_state":[0, 1.3, -1, 0, 0.5, 0]}'
```

An `eef` command with no `joint`, on a component that has a body, is solved for position only. The quaternion is not tracked. `--delta` on `eef` is added to the current flange pose before IK. `--delta` on `joint` is added to the current joints, then gated as an absolute target.

## Servo modes

`setup.control_mode` is passed to `set_up`. Change it later with `cyr call NAME '{"control_mode":"..."}'`. Each arm implements only its own `supported_modes`. An unknown name raises and lists the modes that device has. Aliases: `pos`, `mit_control`, `gravity`, `imp`, `imp_joint`, `teleop`, `teleop_imp`, `cart`, `imp_cart`, `imp_force`.

| Mode | Who | What the host sends |
|------|-----|---------------------|
| `position` | every arm | A joint goal to the drive or the simulator. PiperSim in this mode moves straight to the goal |
| `mit` | Piper, PiperAgx, Y1, PiperSim | `kp`, `kd`, `q_des`, `qd_des`, `tau_ff` |
| `gravity_compensation` | arms that can send joint MIT and have a URDF | `kp = 0`, the arm's small `kd`, and `tau_ff` equal to link gravity plus the identified flange load. A real arm then multiplies `torque_scale`. The host does not integrate |
| `impedance` | the same arms, plus Marvin | On an MIT arm, `kp` and `kd` are `joint_K` and `joint_D` from the config, and `tau_ff` is still that gravity torque. Impedance is refused until `set_payload`, or until `tool_in_urdf` says the load is already in the URDF. Real arms do not reuse the simulator's default gains |
| `cartesian`, `force` | Marvin | The drive's own Cartesian impedance and end-effector force. These names stay as they are |

An arm that can send joint MIT inherits `MitServo`. Piper and Y1 refresh the command on the control tick. PiperAgx keeps its 200 Hz thread and no longer computes a second URDF gravity. Without a URDF, gravity and impedance are rejected. Piper reads the URDF from `setup.urdf`. Marvin has no joint MIT. Its impedance is `set_imp_joint_state`, and the load is the 10-number tool dynamics passed to `set_tool`.

## Toolkits

Each toolkit's steps are in `toolkits/<name>/agent.md`. Run that directory's `check.py` first. When it fails, follow the questions it prints.

Gravity identification fits flange mass `m` and center of mass `c`. Inertia is not observable from a static hold. Leave the arm in `position`, send at least eight joint targets with different wrist orientations through `cyr call`, and let `collect.py` record torque after the arm stops. The residual is measured torque minus link `g(q)`. The fit is written back with `{"op":"set_payload"}`, then the mode switches to `gravity_compensation`. Collect again after changing the gripper. If the load is already in the URDF, pass `--tool-in-urdf` and skip collection.

Joint impedance does not fit `K` and `D`. On a real arm those values belong in `setup`. The simulator defaults exist only in `piper_sim_impedance.yml`. An MIT arm will not enter impedance before `set_payload`. If the load is already in the URDF, set `tool_in_urdf: true`. That simulator file does.

```bash
cyr start --config config/runtime/piper_sim_gravity.yml
python toolkits/gravity_compensation/check.py --tool-in-urdf
python toolkits/gravity_compensation/collect.py --service piper_sim --arm left_arm
cyr start --config config/runtime/piper_sim_impedance.yml
python toolkits/impedance/check.py
```

Hand-eye lives in `toolkits/handeye/`. PiperSim has no camera, so the check asks for a camera name and whether the camera is eye-in-hand or eye-to-hand. `toolkits/handeye_charuco_roma/` is the previous capture front end and stays as it is.

## Body file

`config/bodies/marvin.yml` describes the Marvin M6: joint names, home, base, flange, palm, and the rotation from each base frame into the unified frame. Limits come from the URDF unless `lower` and `upper` are written in the body file. Poses and clearance need `pinocchio`, which is not installed by this package. The URDF path in that file points outside the repo. If it is missing, body kinematics are skipped by the tests.

The pose vector is xyz meters plus quaternion wxyz. `flange_to_palm` is the palm origin in the flange frame, in meters.

## Marvin lab

`config/runtime/marvin_lab.yml`, shared memory `cyr_marvin_lab`.

| Name | Kind | Notes |
|------|------|-------|
| `left_arm`, `right_arm` | Marvin controller | Same `192.168.1.190`, arms A and B, one process so they share one UDP socket. Startup uses joint impedance (`K` 16/8, `D` 1.8, vel/acc 50) and the existing tool dynamics. Joints are radians. `eef` is xyz meters plus quaternion wxyz. Gate: 0.2 rad without tracking, 8 mm FK error, 40 mm clearance. Tracking: 0.05 rad per tick, 0.02 rad tolerance, 12 s. `reset` goes to the body home joints. Service stop holds the current feedback and does not disable the arms |
| `left_hand`, `right_hand` | Wuji Hand 2 | UDP `192.168.1.110:7447` and `192.168.1.111:7447`. Twenty joints in the published model order. The hand streams the latest goal at `rate_hz`; each tick moves at most `max_joint_step_rad` (0.08). No body gate. Service stop disables the hand |
| `left_glove`, `right_glove` | Moxian sensor | `/dev/moxian_left`, `/dev/moxian_right`. Publishes raw `pressure` |
| `cam_head`, `cam_left_wrist`, `cam_right_wrist` | Orbbec sensor | Color only, by serial |

```bash
cyr start --config config/runtime/marvin_lab.yml
cyr call left_arm '{"control_mode":"position"}'
cyr call left_hand '{"op":"reset"}'
cyr call left_arm '{"op":"reset"}' --timeout 15
```

Wuji SDK: if the current env is not named `xone_new`, the hand process loads that env's `wuji-sdk` from the sibling prefix. Override with `CYR_WUJI_SITE`. Marvin SDK root: `MARVIN_SDK_ROOT`, otherwise the sibling `X-Robot-Pipeline_deploy` checkout.

## Runtime behavior

- One latest-value shared-memory segment per service. Values are `msgpack_numpy`.
- One process per component, except Marvin arms that share `robot_ip`.
- Observations are each component's latest frame plus `timestamp_ns`. There is no cross-component sync unless you use `RobotHarness.get_obs()`.
- The command ring defaults to 8. When it is full, the oldest command is dropped.
- An abnormal component exit latches safety, safe-stops the other devices, rejects later commands, and shuts the service down.
- Several services can run at once. Discovery is `$XDG_STATE_HOME/cyr/runtime.json` (default `~/.local/state/cyr/runtime.json`) under `services`. Logs append to `cyr.log`. Device stdout appends to `driver.log`.
- `INFO_LEVEL` is `DEBUG`, `INFO`, or `ERROR` for the older controller debug prints.

## Tests

From the repo root, with a Python that does not auto-load ROS pytest plugins:

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python -m pytest src/robot/runtime/tests src/robot/harness/tests -q --tb=line
```

The Marvin URDF and `pinocchio` tests skip when those are absent.

## Directory

| Path | Content |
|------|---------|
| `src/robot/runtime/` | Service, shared memory, session API, `cyr`, HMI |
| `src/robot/harness/` | Body model, command gate, joint tracking, action contract, aligned observation, episode log |
| `src/robot/controller/` | Arm, mobile, dexterous-hand, and test controllers |
| `src/robot/sensor/` | Cameras, tactile sensors, and test sensors |
| `config/runtime/` | `test_robot.yml`, `piper_sim.yml`, `piper_sim_gravity.yml`, `piper_sim_impedance.yml`, `marvin_lab.yml` |
| `config/bodies/` | Body description used by the gate and harness |
| `toolkits/` | Gravity compensation, joint impedance, Charuco hand-eye |
| `third_party/` | Vendor libraries |
| `agent.md` | Notes for coding agents |
| `developer_README.md` | Where the old collect / deploy notes went |

# WECHAT

<p align="center">
  <img src="imgs/Wechat.jpg" alt="wechat_group" width="400">
  <img src="imgs/myWechat.jpg" alt="my_wechat" width="400">
</p>

If the WeChat group has expired, add the personal account to join.
