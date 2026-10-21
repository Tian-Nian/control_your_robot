[![中文](https://img.shields.io/badge/中文-简体-blue)](./README_CN.md)  
[![English](https://img.shields.io/badge/English-English-green)](./README.md)

[Chinese WIKI](https://tian-nian.github.io/control_your_robot-doc/)

# A new open-source project has been released, building upon this project.
[X-One-Pipeline](https://github.com/XsparkAI/X-One-Pipeline)

X-One-Pipeline currently ships a Y1-IMETA sample. Drivers for other arms stay in this repository under `src/robot/controller`. To add a robot inside X-One-Pipeline, add a config such as `config/x-one.yml`, a `${robot}.py` under that project's `src/robot/robot`, and register it in `src/robot/robot/__init__.py`. Those two paths are part of X-One-Pipeline. This repository does not contain `config/x-one.yml`.

# Recent updates (2026-10)

The shared-memory runtime is on [`feat/shm-runtime`](https://github.com/Tian-Nian/control_your_robot/tree/feat/shm-runtime). Controllers and sensors run as a service. A client reads the latest observation and sends commands with `cyr` or `robot.runtime.session.Session`. That branch also checks joint commands before they are sent, tracks them in limited steps, and exposes `robot.harness` for policy code.

`main` remains the collect, xspark, and XPolicyLab deploy stack documented below. `cyr` and `config/runtime/` are not on this branch.

# Recent updates (2026-07)

This branch adds [XPolicyLab](https://github.com/XPolicyLab/XPolicyLab) deployment support, Xspark data conversion, teleop IPC improvements, and several protocol / naming fixes.

## XPolicyLab integration

Upstream: [XPolicyLab](https://github.com/XPolicyLab/XPolicyLab)

### End-to-end workflow

```
control_your_robot collect
        ↓
tools/transform2xspark.py          # raw HDF5 → xspark v1.0 layout
        ↓
tools/write_xpolicy_instruction.py # ensure instruction + metadata (if not set during transform)
        ↓
copy/symlink into XPolicyLab tree   # data/{bench}/{task}/{env_cfg}/data/episode_*.hdf5
        ↓
XPolicyLab policy/<POLICY>/process_data.sh + train.sh
        ↓
offline replay eval (ReplayRobot) or real-robot deploy (real_env_client)
```

**Step 1 — Collect raw data**

```bash
python example/collect/collect.py
# or teleop:
python example/teleop/master_slave_arm_teleop_fs.py
```

Raw layout: `save/{task_name}/*.hdf5` with flat groups such as `left_arm/`, `cam_head/`.

**Step 2 — Convert to xspark v1.0**

```bash
python tools/transform2xspark.py save/test_robot \
  --save-dir ./data \
  --task-name stack_bowls \
  --skip-video
```

Output layout:

```
data/stack_bowls/
├── data/episode_0000000.hdf5
├── preview_video/                 # optional
└── config.json
```

**Step 3 — Ensure language instruction (optional during convert)**

`transform2xspark.py` only writes `instruction` / `instructions` when it finds text from:

- `--instruction`
- sidecar JSON (`0.json`, `info.json`)
- `task_info/{task_name}.json` or `task_instructions/{task_name}.json`

If none are found, the episode is still converted and **`instruction` keys are omitted** so you can add them later:

```bash
python tools/write_xpolicy_instruction.py data/stack_bowls/data \
  --task-name stack_bowls \
  --verify
```

**Step 4 — Place data for XPolicyLab training**

```bash
# Example target inside an XPolicyLab workspace
mkdir -p ../XPolicyLab/data/RoboDojo/stack_bowls/piper_x/data
cp data/stack_bowls/data/episode_*.hdf5 ../XPolicyLab/data/RoboDojo/stack_bowls/piper_x/data/
```

Then follow the target policy README, e.g.:

```bash
cd ../XPolicyLab/policy/Pi_05
bash install.sh
bash process_data.sh RoboDojo stack_bowls piper_x joint
bash train.sh RoboDojo stack_bowls piper_x joint 0 0
```

**Step 5 — Offline evaluation (HDF5 replay)**

Use `my_robot/replay_robot.py` with `example/deploy/replay_robot.yml` (copy and edit for your task). Point `hdf5_paths` to the **`data/`** subdirectory and set `hdf5_format: xspark`. The replay client reads the top-level `instruction` dataset and sends it to the policy server as `obs["instruction"]`.

**Step 6 — Real-robot deployment**

Split-machine flow (see XPolicyLab policy README):

```bash
# policy machine
bash setup_eval_policy_server.sh ...

# robot machine — uses example/deploy/real_env_client.py via task_env shim
bash setup_eval_env_client.sh ...
```

Protocol: WebSocket via `client_server.ws.WsModelClient` (not legacy `robodojo_ws`).

### xspark v1.0 episode schema (XPolicyLab-compatible)

Each `episode_{index:07d}.hdf5` contains:

| Location | Type | Notes |
|----------|------|-------|
| `instruction` | UTF-8 string | Required before training / offline eval; may be added later |
| `instructions` | UTF-8 string array | Same text as a one-element list (Pi_05 / LeRobot converters) |
| `data_format_version` | `"v1.0"` | Trajectory protocol version |
| `additional_info/frequency` | int | Control/collect rate (typically 25 or 30) |
| `vision/{cam}/colors` | `(T,)` JPEG bytes | `cam_head`, `cam_left_wrist`, `cam_right_wrist` |
| `vision/{cam}/shape` | `(3,)` | Image shape metadata |
| `state/*_joint_states` | `(T, DOF)` | Arm + gripper proprioception |
| `state/*_ee_poses` | `(T, 7)` | xyz + quaternion **wxyz** |
| `action/*` | same keys as state | `action[t] = state[t+1]` |

Repo helpers:

- **`example/deploy/real_env_client.py`**: real-robot environment client
- **`my_robot/replay_robot.py`** + **`example/deploy/replay_robot.yml`**: offline HDF5 replay
- **`example/deploy/robot_client_server/`**: lightweight model client/server helpers
- **`task_env/real_env_client.py`**: import shim for XPolicyLab robodojo
- **`src/robot/utils/base/xpolicy_hdf5.py`**: metadata read/write/validate helpers

## Xspark data pipeline

- **`X_spark_format_pipeline`** in `src/robot/utils/base/data_transform_pipeline.py`
- **`tools/transform2xspark.py`**: batch-convert raw collect HDF5 to xspark layout
- **`tools/write_xpolicy_instruction.py`**: add/update `instruction`, `instructions`, `data_format_version`, `additional_info/frequency`
- Supports legacy `qpos` → `eef` and `slave_*_arm` → `left_arm` / `right_arm` renaming
- Use `--skip-video` or `XSPARK_SKIP_PREVIEW=1` for faster batch conversion

```bash
python tools/transform2xspark.py save/test_robot --save-dir ./data --skip-video
python tools/write_xpolicy_instruction.py data/xspark/data --task-name xspark --verify
```

## Teleop performance

- **`example/teleop/master_slave_arm_teleop_fs.py`** now uses **`MsgpackSharedDict`** instead of `multiprocessing.Manager().dict()` for master → slave → collector IPC, avoiding slow JSON-like proxy serialization on every control tick.
- Graceful process shutdown avoids corrupting HDF5 on episode finish.
- Dependency: `msgpack>=1.0.5` (see `pyproject.toml` / `requirements.txt`).

## Fixes and refactors

- Unified end-effector field naming: **`qpos` → `eef`** across controllers, robots, and scripts.
- XPolicyLab ws protocol update: **`robodojo_ws` → `ws`** in deploy client.
- Agent-oriented repo guide: **`agent.md`**.

# WECHAT
<p align="center">
  <img src="imgs/Wechat.jpg" alt="wechat_group" width="400">
  <img src="imgs/myWechat.jpg" alt="my_wechat" width="400">
</p>
if the wechat group overdue, you could add my wechat to join in.

# Piper new sdk pyAgxArm support!
We now support the new Piper SDK--pyAgxArm, which is more efficient and easier to use. You can refer to `src/robot/controller/PiperAgx_controller.py` for details. 
In this new sdk, we support reset robotic-arm while in teleop mode, cause we enable gravity-compensation by MIT control. But be careful when you reset the robot, make sure the robot is in a safe position to avoid any damage.
We also provide a simple teleop script `example/teleop/master_slave_arm_teleop_fs.py` to show how to use the new sdk to control the robot, you can have a try. Also a robot demo `agilex_piperAgx_dual_base.py` provided in `my_robot`.

# Robot Node support!
Use `example/collect/collect_node.py` to collect robot data in parallel. The script wraps the robot class with a node.

# Visualize data with Rerun
Use `scripts/visual_hdf5_rerun.sh` to visualize data. Supported inputs:
1. control_your_robot raw data
2. ACT / RDT HDF5 data

# The collector can write another format
`CollectAny._add_data_transform_pipeline()` selects a writer. Both writers are in `src/robot/utils/base/data_transform_pipeline.py` on this branch:
1. `image_rgb_encode_pipeline` encodes each RGB image as JPEG before saving, which uses less disk.
2. `general_hdf5_rdt_format_pipeline` writes RDT HDF5 without an instruction.

# RoboTwin deploy pipeline
RoboTwin deployment is supported. For a Sim2Real run:
1. Link a RoboTwin policy into this repo's `policy/` directory.
```bash
# example
ln -s path/to/RoboTwin/policy/pi0 path/to/control_your_robot/policy/
```

2. Edit `deploy.sh` at the repository root.
Pass `--robotwin` first, then the extra RoboTwin arguments:
```bash
# pi0 eval.sh
python script/eval_policy.py --config policy/$policy_name/deploy_policy.yml \
    --overrides \
    --task_name ${task_name} \
    --task_config ${task_config} \
    --train_config_name ${train_config_name} \
    --model_name ${model_name} \
    --ckpt_setting ${model_name} \
    --seed ${seed} \
    --policy_name ${policy_name} 

# your deploy.sh
python example/deploy/deploy.py \
    --base_model_name "openpi"\
    --base_model_class "None"\
    --base_model_path "None"\
    --base_task_name "test"\
    --base_robot_name "test_robot"\
    --base_robot_class "TestRobot"\
    --robotwin \
    --overrides \
    --task_name ${task_name} \
    --task_config ${task_config} \
    --train_config_name ${train_config_name} \
    --model_name ${model_name} \
    --ckpt_setting ${model_name} \
    --seed ${seed} \
    --policy_name ${policy_name} 
```

By `RoboTwin` pipeline, some of the `base_` index may not make effect, here use None instead. Also some of the RoboTwin setting not take effect, you could ignore it.

Other deploy info you could refer to RoboTwin Document, like some of the model should modify `deploy_policy.yml`.

# Control Your Robot!
This project aims to provide a comprehensive and ready-to-use pipeline for embodied intelligence research, covering everything from robotic arm control, data collection, to Vision-Language-Action (VLA) model training and deployment.

## Quick Start!
Since this project includes several test examples, such as robotic arm tests, visual simulation, and full robot simulation, it is possible to quickly understand the overall framework without requiring any physical hardware.  
Because no hardware is needed, you can install the environment simply by running:

```
 pip install -r requirements.txt
 # or
 GIT_LFS_SKIP_SMUDGE=1 uv sync
GIT_LFS_SKIP_SMUDGE=1 uv pip install -e .
```  
This project provides special debug levels: `"DEBUG"`, `"INFO"`, and `"ERROR"`. To fully observe the data flow, set it to `"DEBUG"`:
```bash
export INFO_LEVEL="DEBUG"
```

Alternatively, you can set it in the main function:
```python
import os
os.environ["INFO_LEVEL"] = "DEBUG" # DEBUG , INFO, ERROR
```

1. Data Collection Tests

To collect your robot data, just change the robot class in this file.
```bash
# Node collect
python example/collect/collect_node.py
# Single-threaded (delays inside one call can pile up)
python example/collect/collect.py
```

2. Model Deployment Tests

To deploy a model, put the model class in `src/robot/policy/test_policy/inference_model.py`, following the other packages under `src/robot/policy/`. Point the robot at it from `deploy.sh` in the repository root. There is no `scripts/deploy.sh`.

```bash
# General deployment script
bash deploy.sh
# Offline data replay consistency test
# Collect at least three episodes first: python example/collect/collect.py
bash eval_offline.sh
```

3. Remote Deployment and Data Transfer

Just change the robot / model class in the file, and correctly set the input/output transform.

```bash
# Start the server first, simulating the inference side (allows multiple connections, listens on a port)
python scripts/server.py
# On the client side, collect data and execute commands (example only executes 10 times)
python scripts/client.py
```

4. Interesting Scripts
```python
# Collect keypoints and perform trajectory replay
python scripts/collect_moving_ckpt.py 
# SAPIEN simulation, see src/robot/planner/README.md
```

5. Debug Scripts
```bash
# Packages live under src/. Use an editable install, or set PYTHONPATH.
PYTHONPATH=src python -m robot.controller.TestArm_controller
PYTHONPATH=src python -m robot.sensor.TestVision_sensor
PYTHONPATH=src python -m my_robot.test_robot
```

6. Data Conversion Scripts
```bash
# After running python example/collect/collect.py and obtaining trajectories
python scripts/convert2rdt_hdf5.py save/test_robot/ save/rdt/

# TO openpi dataset(lerobot2.1)
python scripts/convert2act_hdf5.py save/test_robot/ save/openpi_hdf5/
# you should copy an instruction from task_instructions/ to save/openpi_hdf5/
python scripts/convert2openpi.py --raw_dir save/openpi_hdf5/ --repo_id my_repo_id
```

7. upload data
**Important!!!**

`vision_sensor` publishes color as JPEG when `is_jpeg` is true, which is the default. The old `encode_rgb` flag has been removed.

```bash
# In the original dataset, image files occupy a large amount of storage space, which is unfavorable for data transmission. Therefore, a compression and decompression script is provided. It performs JPEG processing on the images to enable faster transfer. The script is configured by default for a dual-arm, three-view setup, but it can be adjusted according to specific needs.
# Compress. This writes path/to/folder/_zip/.
python scripts/upload_zip.py path/to/folder --encode

# Decompress.
python scripts/upload_zip.py path/to/folder
```

8. Teleop by joint or eef
```bash
# We provide two general architectures for teleoperation-based data collection. The first one is relatively simple, where the teleoperation control frequency is aligned with the data collection frequency. The second one is slightly more complex, allowing the teleoperation frequency to far exceed the data recording frequency. Both architectures have been validated through real-world robot experiments.
# same freq
python example/teleop/master_slave_arm_teleop.py 
# diff freq (300Hz control / 60Hz collect; msgpack IPC)
python example/teleop/master_slave_arm_teleop_fs.py
```

9. Xspark + XPolicyLab data prep
```bash
python tools/transform2xspark.py save/test_robot --save-dir ./data --skip-video
python tools/write_xpolicy_instruction.py data/xspark/data --task-name xspark --verify
```

See **XPolicyLab integration** above for the full training / offline eval / deploy flow.

## On real robot!
1. Implement Your Robot Configuration
``` python
Required:
__init__(self): Initialize all controllers and sensors
set_up(self): Execute the set_up parameters for each controller and sensor

Optional:
is_start(self): Check if the robot arm is currently moving
reset(self): Reset the robot arm to its initial position
```
2. Some Additional Functions
``` python
replay(self, hdf5_path, key_banned, is_collect, episode_id):
    Replay previously collected trajectories.
    Note: key_banned must be set. If you are replaying based on joints, ban 'eef'; otherwise, ban the other key.
    is_collect and episode_id are linked: they indicate that the replayed trajectory data is being collected and saved under the given episode_id.
    If episode_id is not set, you might overwrite the originally collected trajectories.

show_pic(self, data_path, pic_name):
    Replay video from a specified camera.
    Can be used directly and will automatically decode compressed images in the raw data.
```

### 🤖 Supported Devices

#### 🎛️ Controllers
Classes that are already in `src/robot/controller/`:

| Robotic arm | Mobile base | Hand |
|-------------|-------------|------|
| Agilex Piper, Piper Agx | Agilex Tracer 2.0 | `dexhand_controller.py` is a base class, not a finished hand |
| RealMan 65B, including the ROS variant | Agilex Bunker | |
| Daran ALOHA, Y1 | | |
| Franka (`FrankaFranky_controller.py`), UR (`URUrx_controller.py`) | | |

JAKA does not have a controller in this branch. Wuji Hand 2 is on [`feat/shm-runtime`](https://github.com/Tian-Nian/control_your_robot/tree/feat/shm-runtime), not here.

#### 📡 Sensors
Classes that are already in `src/robot/sensor/`: RealSense, OpenCV, ROS images, Vitac3D, ROS / ROS 2 tactile gloves, Pika, and Quest. Orbbec and Moxian are on `feat/shm-runtime`. Open an issue or a pull request for another sensor.

## Directory Overview
| Directory       | Description                  | Main Content |
|----------------|-----------------------------|--------------|
| **📂 controller** | Robot controller wrappers  | Classes for controlling arms, mobile bases, etc. |
| **📂 sensor**    | Sensor wrappers            | RealSense, OpenCV, ROS images, Vitac3D, tactile gloves, Pika, Quest |
| **📂 utils**     | Utility functions          | Math, logging, and other helper functions |
| **📂 data**      | Data collection module     | Classes for data recording and processing |
| **📂 my_robot**  | Robot integration wrappers | Full robot system composition classes |
| **📂 policy**    | VLA model policies         | Vision-Language-Action model implementations |
| **📂 scripts**   | Example scripts            | Main entry points and test scripts |
| **📂 third_party** | Third-party dependencies | External libraries requiring compilation |
| **📂 planner**   | Motion planning module     | `curobo` planner wrappers + simulated robot code |
| **📂 example**   | Example workflows          | Data collection, model deployment examples |
