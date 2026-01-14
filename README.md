[![中文](https://img.shields.io/badge/中文-简体-blue)](./README_CN.md)  
[![English](https://img.shields.io/badge/English-English-green)](./README.md)

[Chinese WIKI](https://tian-nian.github.io/control_your_robot-doc/)

# WECHAT
<p align="center">
  <img src="imgs/Wechat.jpg" alt="wechat_group" width="400">
  <img src="imgs/myWechat.jpg" alt="my_wechat" width="400">
</p>
if the wechat group overdue, you could add my wechat to join in.

# Data collect pipeline now can choose to save the data into the format you want!
You can try it by switch to the branch--`newest` to have a try.  
A function added at `CollectAny` called `_add_data_transform_pipeline()`, add we provide two choice under `utils/data_transofrm_pipeline.py`:
1. `image_rgb_encode_pipeline`  
this will encode every egb image in your data, then save it, it will cost lower memory for data saving by using jepg.
2. `general_hdf5_rdt_format_pipeline`  
this will save the data into rdt_hdf5 format without instruction.

# RoboTwin depoly pipeline support!
Now we already support RoboTwin pipeline! You can refer to under step for your Sim2Real experiment!
1. link `RoboTwin` policy to `control_your_robot` policy
```bash
# example
ln -s path/to/RoboTwin/polciy/pi0 path/to/control_your_robot/policy/
```

2. modify `deploy.sh`
By using `RoboTwin` pipeline, you should set `--robotwin` first, then extra info should be set like:
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
```bash
# Multi-process (strict time-synchronized collection using TimeScheduler)
python example/collect/collect_mp_robot.py
# Multi-process (separate process for each component)
python example/collect/collect_mp_component.py
# Multi-process (separate process for each component, and have diffrent save_freq for each, this will save timestamp)
python example/collect/collect_mp_component_different_time_freq.py
# Single-threaded (may have accumulated delays due to function execution)
python example/collect/collect.py
```

2. Model Deployment Tests
```bash
# Run a straightforward deployment test
python example/deploy/robot_on_test.py
# General deployment script
bash deploy.sh
# Offline data replay consistency test
# you should generate data at least 3 data first by running: python example/collect/collect.py
bash eval_offline.sh
```

3. Remote Deployment and Data Transfer
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
# SAPIEN simulation, see planner/README.md for details
```

5. Debug Scripts
```bash
# Because controller and sensor packages have __init__.py, execute with -m
python -m src.robot.controller.TestArm_controller
python -m src.robot.sensor.TestVision_sensor
python -m my_robot.test_robot
```

6. Data Conversion Scripts
```bash
# After running python example/collect/collect.py and obtaining trajectories
python scripts/convert2rdt_hdf5.py save/test_robot/ save/rdt/
```

7. upload data
**Important!!!**

sensor.vision_sensor already set encode_rgb=True by DEFAULT, you do not need to use encode and decode.

```bash
# In the original dataset, image files occupy a large amount of storage space, which is unfavorable for data transmission. Therefore, a compression and decompression script is provided. It performs JPEG processing on the images to enable faster transfer. The script is configured by default for a dual-arm, three-view setup, but it can be adjusted according to specific needs.
# compress. will make a new floder: path/to/floder/_zip/
python scripts/upload_zip.py path/to/floder --encode

# decompress.
python scripts/upload_zip.py path/to/floder
```

8. telop by joint/eef
```bash
# We provide two general architectures for teleoperation-based data collection. The first one is relatively simple, where the teleoperation control frequency is aligned with the data collection frequency. The second one is slightly more complex, allowing the teleoperation frequency to far exceed the data recording frequency. Both architectures have been validated through real-world robot experiments.
# same freq
python example/teleop/master_slave_arm_teleop.py 
# diff freq
python example/teleop/master_slave_arm_teleop_fs.py
```

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
    Note: key_banned must be set. If you are replaying based on joints, ban 'qpos'; otherwise, ban the other key.
    is_collect and episode_id are linked: they indicate that the replayed trajectory data is being collected and saved under the given episode_id.
    If episode_id is not set, you might overwrite the originally collected trajectories.

show_pic(self, data_path, pic_name):
    Replay video from a specified camera.
    Can be used directly and will automatically decode compressed images in the raw data.
```

### 🤖 Supported Devices

#### 🎛️ Controllers
**✅ Implemented**
| Robotic Arm       | Mobile Base        | Dexterous Hand  | Others     |
|------------------|------------------|----------------|------------|
| Agilex Piper     | Agilex Tracer2.0 | 🚧 In Development | 📦 To Be Added |
| RealMan 65B      | Agilex bunker     | 📦 To Be Added    | 📦 To Be Added |
| Daran ALOHA      | 📦 To Be Added     | 📦 To Be Added    | 📦 To Be Added |
| Y1 ALOHA      | 📦 To Be Added     | 📦 To Be Added    | 📦 To Be Added |

**🚧 Planned Support**
| Robotic Arm      | Mobile Base       | Dexterous Hand | Others     |
|-----------------|-----------------|----------------|------------|
| JAKA             | 📦 To Be Added    | 📦 To Be Added | 📦 To Be Added |
| Franka           | 📦 To Be Added    | 📦 To Be Added | 📦 To Be Added |
| UR5e             | 📦 To Be Added    | 📦 To Be Added | 📦 To Be Added |

#### 📡 Sensors
**✅ Implemented**
| Vision Sensors   | Tactile Sensors | Other Sensors |
|-----------------|----------------|---------------|
| RealSense Series | Vitac3D        | 📦 To Be Added |

**🚧 Planned Support**
For new sensor support requests, please open an issue, or submit a PR with your sensor configuration!

## Directory Overview
| Directory       | Description                  | Main Content |
|----------------|-----------------------------|--------------|
| **📂 controller** | Robot controller wrappers  | Classes for controlling arms, mobile bases, etc. |
| **📂 sensor**    | Sensor wrappers            | Currently only RealSense cameras |
| **📂 utils**     | Utility functions          | Math, logging, and other helper functions |
| **📂 data**      | Data collection module     | Classes for data recording and processing |
| **📂 my_robot**  | Robot integration wrappers | Full robot system composition classes |
| **📂 policy**    | VLA model policies         | Vision-Language-Action model implementations |
| **📂 scripts**   | Example scripts            | Main entry points and test scripts |
| **📂 third_party** | Third-party dependencies | External libraries requiring compilation |
| **📂 planner**   | Motion planning module     | `curobo` planner wrappers + simulated robot code |
| **📂 example**   | Example workflows          | Data collection, model deployment examples |
