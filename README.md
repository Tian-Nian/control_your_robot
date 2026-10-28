[![中文](https://img.shields.io/badge/中文-简体-blue)](./README_CN.md)
[![English](https://img.shields.io/badge/English-English-green)](./README.md)

# control_your_robot

This branch runs robot controllers and sensors as a shared-memory service. A client reads each component's latest observation and sends commands by component name. Joint teleop, HDF5 collection, and XPolicyLab deployment all attach to that service.

```bash
pip install -e .
cyr start --config config/runtime/test_robot.yml
cyr snapshot
cyr stop
```

`cyr` must be the console script from this install. On Debian, `/usr/bin/cyr` is a different program. Check with `which cyr`.

Hardware is optional. `config/runtime/test_robot.yml` uses in-process test devices. `config/runtime/piper_sim.yml` is the Piper simulator in position mode. Gravity and impedance use `piper_sim_gravity.yml` and `piper_sim_impedance.yml`. `config/runtime/marvin_lab.yml` is the Marvin lab bench.

## Optional MuJoCo and visual agent

Piper Sim remains the teaching default. Install the contact/vision stack explicitly in a separate environment:

```bash
python3 -m venv .venv-harness
source .venv-harness/bin/activate
python -m pip install -r requirements/harness-vision.txt
python -m pip install --no-deps -e .
python examples/piper_mujoco_harness.py --pick-test
bash examples/piper_mujoco.sh start
bash examples/piper_mujoco.sh viewer
# Configure a vision-capable model API using the guide before starting:
bash examples/piper_mujoco.sh agent
```

The default `robowork` architecture uses one agent, head/wrist RGB-D, explicit skills, read-only planning and single-use execution. It supports tool/base/camera-frame translation and rotation plus joint control. Gripper targets persist across motion and model inference, with bounded simulated closing preload. Simulator object/contact truth is excluded from model input. Enter, explicit finish, or runtime faults end the loop; there are no former stage/recovery budgets.

The same device accepts `cyr call` for absolute/delta joints, flange poses, gripper and arm-only reset. Queued motion interpolates and checks joint feedback. Restart rebuilds the entire scene; resume uses fresh observations in the existing scene. Rerun is a separate read-only viewer, and reports summarize logged failures and exit sources offline.

See the [complete current usage guide](docs/harness_robowork.md) for installation, API configuration, every entry point and option, tool semantics, manual commands, logs and troubleshooting. The [PositionExecutor API](docs/harness.md) and [scene asset guide](assets/sim/piper_mujoco/README.md) remain separate. Previous layered behavior requires explicit `--architecture layered`.

Fixed-trajectory contact grasp, 15-second hold and release are validated; reliable autonomous visual grasping and three-bowl stacking are not. Plans do not certify scene collision freedom. MuJoCo is not installed by default.

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
| `cyr teleop --source SRC --follower NAME` | Copy `joint` and `gripper` from the source onto the follower. Holds the control lock |
| `cyr collect --task TASK` | Record the latest observations to `save/TASK/N.hdf5`. Read only |
| `cyr replay --file PATH` | Play joints from a collect file or an xspark file through the deploy action path |
| `cyr deploy --policy NAME` | Call `XPolicyLab.policy.NAME.deploy.eval_one_episode` on this service |

```bash
cyr call left_arm '{"op":"reset"}'
cyr call left_arm '{"joint":[0, 1.3, -1, 0, 0.5, 0]}'
cyr call left_arm '{"joint":[0, 0.05, 0, 0, 0, 0]}' --delta
cyr call left_arm '{"op":"release"}'
cyr call left_arm '{"control_mode":"gravity_compensation"}'
```

`cyr hmi`: `q` leaves the UI. `x`, then `y` or Enter, stops every running service. `n` cancels that prompt. Up / down or `k` / `j` move the selection.

The CLI attaches to the only live service. If more than one is running, `echo`, `snapshot`, `call`, `teleop`, `collect`, `replay`, and `deploy` fail and ask you to use `Session.attach(name)` from Python.

## Teleop, collection, and XPolicyLab

These commands use a service that is already running. They do not start one. A service left over from an older shared-memory layout has to be rebuilt with `cyr restart` before teleop or deployment. `cyr hmi` can still read that older segment.

Teleop, deployment, and replay each hold one service-wide control lock. The holder is the only client that may write joint, end-effector, gripper, and reset targets. Another process that tries to write gets `accepted: false` and a warning that names the holder (`遥操`, `部署`, or `回放`). `snapshot`, `echo`, `get_obs`, and collection keep reading. The holder renews the lock with a heartbeat. The lock drops when that process exits or the heartbeat stops, and the arm then stops chasing the last streamed target. While nobody holds the lock, a one-shot `cyr call` still writes.

### Joint teleop

`cyr teleop` copies the source component's `joint` and `gripper` onto the follower. Both names have to be components of the same service.

```bash
cyr teleop --source master_arm --follower left_arm --rate 50 --lease 2
```

`--rate` is how often the latest target is stored. The default is 50 Hz. `--lease` is the lock timeout in seconds. The default is 2. Ctrl+C releases the lock. The follower applies the stored target on its own tick. When the component has `max_step_rad`, each tick moves at most that far. `config/runtime/piper_sim.yml` publishes only `left_arm`, so teleop needs a second component that publishes `joint`.

### Collection

`cyr collect` reads the latest frame from every component. It does not take the control lock, send motion, or reset the arm. Home the arm yourself before the first Enter.

```bash
cyr call left_arm '{"op":"reset","start_state":[0, 1.3, -1, 0, 0.5, 0]}'
cyr collect --task pick --out save --freq 30
```

Press Enter to start writing, and Enter again to finish. `--freq` is the save rate, default 30 Hz. The episode is `save/pick/0.hdf5`, then `1.hdf5`, and so on. Each component is a group. Each published field is a dataset. `skew_ms` is the spread of the timestamps in that sample, in milliseconds. `save/pick/config.json` lists the field names.

A uint8 `color` or `image` with shape HxWx3 is stored as JPEG. The array in memory is RGB. Encoding matches XPolicyLab `encode_image_bit`: OpenCV receives one RGB-to-BGR conversion, and the file carries the `XPL-RGB1` comment so `decode_image_bit` returns RGB. Bytes that are already encoded are stored as they are, without a new comment.

The training layout is a separate step. `action[t]` is `state[t+1]`, so the file needs at least two frames.

```python
from robot.data.xspark import to_xspark

to_xspark("save/pick/0.hdf5", "save/pick_xspark.hdf5", instruction="pick up the cup", frequency=30)
```

Arm `joint` becomes `{side}_arm_joint_state`, `eef` becomes `{side}_ee_pose` (xyz + wxyz, length 7), and `gripper` becomes `{side}_ee_joint_state`. A `{side}_hand` joint overwrites that end-effector joint state. The file attributes are `instruction`, `frequency`, and `data_format_version` `v1.0`.

### Replay

```bash
cyr replay --file save/pick/0.hdf5 --hz 30
```

Replay holds the lock as `回放` and sends every step through the same `take_action` as deployment. It leaves the arm where it is. An xspark file is read from the `action` group. A raw collect file is read from each arm's `joint` and `gripper`.

### XPolicyLab

`cyr deploy` builds a `RuntimeEnv` and calls `XPolicyLab.policy.<name>.deploy.eval_one_episode`. Put the directory that contains the `XPolicyLab` package on `PYTHONPATH`. The policy server must already be listening. The default address is `127.0.0.1:9999`.

```bash
PYTHONPATH=/path/to/X-Robot-Pipeline_deploy \
  cyr deploy --policy FTP_1_v3_joint --instruction "put the cup in the box" \
  --hz 25 --steps 200 --host 127.0.0.1 --port 9999
```

`RuntimeEnv` is the `TASK_ENV` that loop already uses: `get_obs`, `take_action`, `is_episode_end`, and `reset`. This client resets every command-accepting `*_arm` before `eval_one_episode`. The policy module then calls `model_client.call("reset")` for its own state. The command-line path is joint mode. `--hz` defaults to 25. `--steps 0` does not end the episode from a step count, so pass the number of steps to run. `--lease` defaults to 2 seconds, and the lock role is `部署`. If teleop or replay already holds the lock, deploy stops before the policy loop.

`get_obs()` returns `left_arm_joint_state`, `left_ee_pose` (xyz + wxyz, length 7), and `left_ee_joint_state`, plus the right-side names when those components exist. On a gripper arm, `left_ee_joint_state` is the gripper scalar. A 20-joint `{side}_hand` publication replaces it. Camera `color` is JPEG bytes carrying `XPL-RGB1`. The policy server's `decode_image_bit` turns that buffer into RGB. The observation also has `instruction`, `additional_info.frequency`, and `data_format_version` `v1.0`.

`take_action` stores the latest target and then sleeps for `1/hz`. A length-1 `*_ee_joint_state` is sent as that arm's gripper. A length-20 vector stays on the hand. Action chunks stay inside the policy module.

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
| `src/robot/data/` | HDF5 collection, xspark conversion, standard RGB JPEG |
| `src/robot/teleop/` | Joint master-slave follower |
| `src/robot/deploy/` | TCP client for an XPolicyLab policy server |
| `src/robot/harness/` | Body model, command gate, joint tracking, action contract, `RuntimeEnv`, episode log |
| `src/robot/controller/` | Arm, mobile, dexterous-hand, and test controllers |
| `src/robot/sensor/` | Cameras, tactile sensors, and test sensors |
| `config/runtime/` | `test_robot.yml`, `piper_sim.yml`, `piper_sim_gravity.yml`, `piper_sim_impedance.yml`, `marvin_lab.yml` |
| `config/bodies/` | Body description used by the gate and harness |
| `toolkits/` | Gravity compensation, joint impedance, Charuco hand-eye |
| `third_party/` | Vendor libraries |
| `agent.md` | Notes for coding agents |
| `developer_README.md` | Why the old `my_robot/` layout stays on `main` |

# WECHAT

<p align="center">
  <img src="imgs/Wechat.jpg" alt="wechat_group" width="400">
  <img src="imgs/myWechat.jpg" alt="my_wechat" width="400">
</p>

If the WeChat group has expired, add the personal account to join.

## Verified position harness

Run `python examples/piper_harness.py` for an isolated Piper Sim task with fresh-feedback verification, request deduplication and a JSONL event journal. Add `--viewer` for a separate Rerun window. See [the harness guide](docs/harness.md) for prerequisites, API and interruption semantics.

For the current RGB-D tool workspace, use the [current MuJoCo agent guide](docs/harness_robowork.md).

MuJoCo exception: without a body adapter, the runtime uses the device’s local `solve_ik` for full flange position AND orientation. With motion tracking enabled, its `runtime_reset_target` is interpolated without blocking physics. These are distinct from the older body position-only IK and controller-specific reset implementations.

The launcher now shares the default discovery directory with direct `cyr` and opens the read-only viewer on `start`. Default reset returns to READY, not all-zero joints. See [migration and reset instructions](docs/harness_robowork.md#服务目录迁移可视化与-reset) before migrating an older project-local service; migration resets the scene.
