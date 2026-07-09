# Agent Guide — control_your_robot

This file orients coding agents to the repo layout, conventions, and the most common end-to-end workflows.

## Project purpose

`control_your_robot` is a robotics data + deployment stack:

- **Collect** teleop / scripted episodes to HDF5
- **Transform** raw HDF5 to XPolicyLab xspark v1.0
- **Deploy** VLA policies on replay or real hardware via [XPolicyLab](https://github.com/XPolicyLab/XPolicyLab)

Hardware is optional: `my_robot/test_robot.py` and `TestArmController` / `TestVisonSensor` run without real devices.

## Layout

| Path | Role |
|------|------|
| `src/robot/controller/` | Arm / mobile / hand controllers |
| `src/robot/sensor/` | Cameras and other sensors |
| `src/robot/data/collect_any.py` | Generic HDF5 collector |
| `src/robot/utils/base/data_transform_pipeline.py` | Post-write pipelines (`X_spark_format_pipeline`) |
| `src/robot/utils/base/xpolicy_hdf5.py` | XPolicyLab metadata read/write/validate |
| `src/robot/utils/worker/` | Multiprocess workers + `MsgpackSharedDict` IPC |
| `my_robot/` | Robot compositions (`test_robot`, `replay_robot`, piper/realman/y1) |
| `example/collect/` | Data collection entry points |
| `example/deploy/` | Deployment (`real_env_client.py`, `robot_client_server/`) |
| `example/teleop/` | Master–slave teleop |
| `tools/transform2xspark.py` | Raw HDF5 → xspark layout + metadata (when instruction provided) |
| `tools/write_xpolicy_instruction.py` | Patch instruction/metadata on existing xspark HDF5 |
| `task_env/` | Shim re-export for XPolicyLab `robodojo` imports |

## Conventions

- **End-effector field**: use `eef`, not legacy `qpos`.
- **Collect config**: `save_path`, `task_name`, `save_format`, `save_freq`.
- **CollectAny API**: `CollectAny(condition=..., move_check=..., resume=...)`.
- **Debug levels**: `INFO_LEVEL` = `DEBUG` | `INFO` | `ERROR`.
- **Teleop IPC**: prefer `MsgpackSharedDict` over `Manager().dict()`.

## XPolicyLab workflow

Repo: [https://github.com/XPolicyLab/XPolicyLab](https://github.com/XPolicyLab/XPolicyLab)

### 1. Collect

```bash
python example/collect/collect.py
python example/teleop/master_slave_arm_teleop_fs.py
```

Raw HDF5 groups: `left_arm`, `right_arm`, `cam_head`, ...

### 2. Convert to xspark v1.0

```bash
python tools/transform2xspark.py save/test_robot \
  --save-dir ./data \
  --task-name my_task \
  --instruction "Pick up the red block." \
  --skip-video
```

Writes:

```
data/my_task/data/episode_0000000.hdf5
```

When `xpolicy_instruction` is resolved, the pipeline also writes:

- `instruction` (UTF-8 string)
- `instructions` (UTF-8 string array, one element)

If no instruction is found in `task_info`, sidecars, or `--instruction`, these keys are **skipped**. The pipeline still writes:

- `data_format_version = "v1.0"`
- `additional_info/frequency` (from `save_freq`, default 25)

Instruction resolution order (first match wins):

1. `--instruction`
2. sidecar JSON (`0.json`, `info.json`)
3. `task_info/{task_name}.json` or `task_instructions/{task_name}.json`

### 3. Patch metadata (optional)

```bash
python tools/write_xpolicy_instruction.py data/my_task/data \
  --task-name my_task \
  --frequency 25 \
  --verify
```

Skipped episodes (no instruction source found) are left unchanged.

### 4. Install into XPolicyLab data tree

Target layout:

```
XPolicyLab/data/{bench_name}/{task_name}/{env_cfg_type}/data/episode_0000000.hdf5
```

Example:

```bash
cp data/my_task/data/episode_*.hdf5 \
  ../XPolicyLab/data/RoboDojo/my_task/piper_x/data/
```

### 5. Train in XPolicyLab

Follow the policy README under `XPolicyLab/policy/<POLICY>/`:

```bash
bash install.sh
bash process_data.sh <bench> <task> <env_cfg> <action_type>
bash train.sh <bench> <task> <env_cfg> <action_type> <seed> <gpu>
```

Training scripts expect xspark episodes with a non-empty top-level `instruction` (or `instructions`).

### 6. Offline evaluation

- Driver: `my_robot/replay_robot.py`
- Config: `example/deploy/replay_robot.yml` (generic template; copy and set `env_cfg_type`, `hdf5_paths`, etc.)
- Set `hdf5_paths` to the **`data/`** directory containing `episode_*.hdf5`
- Set `hdf5_format: xspark`
- Replay reads `instruction` from each episode and passes it to the policy as `obs["instruction"]`

### 7. Real-robot deployment

- Robot-side client: `example/deploy/real_env_client.py`
- Import shim: `task_env/real_env_client.py`
- Protocol: `client_server.ws.WsModelClient` (`protocol: ws` in deploy.yml)
- Use XPolicyLab `setup_eval_policy_server.sh` + `setup_eval_env_client.sh` for split-machine runs

## xspark v1.0 fields

Core trajectory groups are always written by `transform2xspark.py`. Instruction fields are **optional during convert** and **required before training / offline eval** (use `--verify` after patching):

```
instruction              # optional at convert; required before train/eval
instructions
data_format_version
additional_info/frequency
vision/...
state/...
action/...
```

EE pose layout: xyz + quaternion **wxyz**. Action tensors are next-state shifted from state.

## Common commands

```bash
pip install -r requirements.txt
python -m my_robot.test_robot
python tools/transform2xspark.py save/test_robot --save-dir ./data --skip-video
python tools/write_xpolicy_instruction.py data/xspark/data --task-name xspark --verify
```

## Commit hygiene

- Keep diffs scoped; match existing naming and import style.
- Do not commit `save/`, `data/`, checkpoints, or NAS paths.
