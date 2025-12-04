[![中文](https://img.shields.io/badge/中文-简体-blue)](./README.md)  
[![English](https://img.shields.io/badge/English-English-green)](./README_EN.md)

## 1. setup environment
Refer to the conda environment from the official RDT documentation: ([RDT official documentation](https://github.com/thu-ml/RoboticsDiffusionTransformer)).

```bash
# Make sure python version == 3.10
conda create -n RDT python==3.10
conda activate RDT

# Install pytorch
# Look up https://pytorch.org/get-started/previous-versions/ with your cuda version for a correct command
pip install torch==2.1.0 torchvision==0.16.0  --index-url https://download.pytorch.org/whl/cu121

# Install packaging
pip install packaging==24.0
pip install ninja
# Verify Ninja --> should return exit code "0"
ninja --version; echo $?
# Install flash-attn
pip install flash-attn==2.7.2.post1 --no-build-isolation

# Install other prequisites
pip install -r requirements.txt
# If you are using a PyPI mirror, you may encounter issues when downloading tfds-nightly and tensorflow. 
# Please use the official source to download these packages.
# pip install tfds-nightly==4.9.4.dev202402070044 -i  https://pypi.org/simple
# pip install tensorflow==2.15.0.post1 -i  https://pypi.org/simple
```
## 2. download models

```bash
# In the RoboTwin/policy directory
cd ../weights
mkdir RDT && cd RDT
# Download the models used by RDT
huggingface-cli download google/t5-v1_1-xxl --local-dir t5-v1_1-xxl
huggingface-cli download google/siglip-so400m-patch14-384 --local-dir siglip-so400m-patch14-384
huggingface-cli download robotics-diffusion-transformer/rdt-1b --local-dir rdt-1b
```

## 3. data transform

If you've used the CollectAny class to store your data, you can use the provided script to convert it to the HDF5 format required by RDT (not the default saved HDF5 format!).

It is recommended to pre-encode the instructions. The following command will handle the language encoding:
```bash
python scripts/encode_lang_batch_once.py task_name output_dir gpu_id
```
`task_name`: The prefix of the .json file in the `task_config/` directory  
`output_dir`: The directory where the encoded instructions will be saved (an `instructions/` folder will be created inside it)  
`gpu_id`: The GPU to use for encoding (set to 0 for single-GPU systems)  

## 4.  Generate Training Config Files
 `$model_name` determines the model's name during training. You can name it freely.
```bash
cd policy/RDT
bash generate.sh ${model_name}
```

This command will create a folder `training_data/${model_name}/ `for your data and a config file `model_config/${model_name}.yml`.

**IMPORTANT!!!**    
This project is configured by default for a dual-arm robot with 6-DOF arms and 1-DOF grippers. Therefore, if:  
1. **You're using a single-arm robot:**  
   According to the RDT official guide, single-arm data should always be filled into the **right arm**, regardless of whether the arm is placed on the left or right side of the workspace.

2. **Your robot has a different number of degrees of freedom (DOF):**  
   You need to modify the `UNI_STATE_INDICES` in lines **175** and **285** of `./data/hdf5_vla_dataset.py` to map your data to the correct joint angle indices.  
   For example, for a **7-DOF arm + 1-DOF gripper** (single-arm setup):
```
UNI_STATE_INDICES = [
    STATE_VEC_IDX_MAPPING[f"right_arm_joint_{i}_pos"] for i in range(7)
] + [
    STATE_VEC_IDX_MAPPING["right_gripper_open"]
]
```
3. You want to use EEF (End-Effector) control:
Refer to configs/state_vec.py and select the data fields you want. For EEF, use the following:
```python
# Without "right" prefix is for single-arm use.
# Both "eef_angle_*" and "right_eef_angle_*" map to the same indices, so they are interchangeable.
'eef_angle_0': 33,
'right_eef_angle_0': 33,
'eef_angle_1': 34,
'right_eef_angle_1': 34,
'eef_angle_2': 35,
'right_eef_angle_2': 35,
'eef_angle_3': 36,
'right_eef_angle_3': 36,
'eef_angle_4': 37,
'right_eef_angle_4': 37,
'eef_angle_5': 38,
'right_eef_angle_5': 38,
...
'left_eef_angle_0': 83,
'left_eef_angle_1': 84,
'left_eef_angle_2': 85,
'left_eef_angle_3': 86,
'left_eef_angle_4': 87,
'left_eef_angle_5': 88,
```
Data units to note:  
`Distance:m, Angles:rad, Gripper: openness level 0~1,0 close, 1 open`

### 4.1 Organizing Training Data
Place your data in the following structure.
If your training set involves multiple tasks, each task should have its own subfolder containing the HDF5 files and encoded instructions.

**Example Folder Structure:**
```bash
training_data/${model_name}
├── ${task_1}
│   ├── instructions
│   │   ├── lang_embed_0.pt
│   │   ├── ...
│   ├── episode_0.hdf5
│   ├── episode_1.hdf5
│   ├── ...
├── ${task_2}
│   ├── instructions
│   │   ├── lang_embed_0.pt
│   │   ├── ...
│   ├── episode_0.hdf5
│   ├── episode_1.hdf5
│   ├── ...
├── ...
```

### 4.2 Modify Training Config `model_config`
In `model_config/${model_name}.yml`, set the GPU(s) to use via `cuda_visible_device`.
1. For a single-GPU setup, set it to:`cuda_visible_device: 0`
2. For multi-GPU training, set it like:`cuda_visible_device: 0,1,4`

## 5. Fine-tuning the Model

Simply run the following command to start training:

```bash
bash finetune.sh ${model_name}
```
**IMPORTANT!!!**

1. If you are using a single GPU, DeepSpeed will not be enabled, and therefore the model will not save the file: `pytorch_model/mp_rank_00_model_states.pt`. 
2. If you wish to continue training from a pre-trained model, set the`pretrained_model_name_or_path` field to the folder of the model checkpoint you want to load e.g.: `./checkpoints/${model_name}/checkpoint-${ckpt_id}`. 

The model will be loaded in Hugging Face format.  
This is the same format used by the official RDT checkpoint:  
`../weights/RDT/rdt-1b`

## 6. Model Deployment
If you're not using the default training configuration (e.g., different DOF or EEF-based training), you need to update the deployment configuration in `scripts/agilex_model.py`:
```python
AGILEX_STATE_INDICES = [
    STATE_VEC_IDX_MAPPING[f"left_arm_joint_{i}_pos"] for i in range(6)
] + [
    STATE_VEC_IDX_MAPPING["left_gripper_open"]
] + [
    STATE_VEC_IDX_MAPPING[f"right_arm_joint_{i}_pos"] for i in range(6)
] + [
    STATE_VEC_IDX_MAPPING[f"right_gripper_open"]
]
```
