[![中文](https://img.shields.io/badge/中文-简体-blue)](./README.md)  
[![English](https://img.shields.io/badge/English-English-green)](./README_EN.md)

[official][](./README_official.md)

## 配置环境
配置环境请参考[SETUP.md](./SETUP.md)
```bash
# Create and activate conda environment
conda create -n openvla-oft python=3.10 -y
conda activate openvla-oft

# Install PyTorch
# Use a command specific to your machine: https://pytorch.org/get-started/locally/
pip3 install torch torchvision torchaudio

# Clone openvla-oft repo and pip install to download dependencies
git clone https://github.com/moojink/openvla-oft.git
cd openvla-oft
pip install -e .

# Install Flash Attention 2 for training (https://github.com/Dao-AILab/flash-attention)
#   =>> If you run into difficulty, try `pip cache remove flash_attn` first
pip install packaging ninja
ninja --version; echo $?  # Verify Ninja --> should return exit code "0"
pip install "flash-attn==2.5.5" --no-build-isolation
```

## 转化数据
转化数据请参考[README.md](../../README.md)中关于TFDS数据集转化部分的描述, 根据自己的数据集选择数据进行填充, 双臂请模仿ALOHA, 单臂请模仿LIBERO,
对于训练的state根据自己的需求选择基于[ joint / EEF(xyz+rpy) + gripper ]进行设置.
```python
# 单臂, DoFs=7, 使用joint
state = [joint_1, joint_2, ..., joint7, gripper]
# 单臂, 使用EEF
state = [x, y, z, roll, pitch, yaw, gripper]

# 双臂, DoF=7, 使用joint
state = [left_joint_1, left_joint_2, ..., left_joint7, left_gripper,\
         right_joint_1, right_joint_2, ..., right_joint7, right_gripper]
# 双臂, 使用EEF
state = [left_x, left_y, left_z, left_roll, left_pitch, left_yaw, left_gripper, \
        right_x, right_y, right_z, right_roll, right_pitch, right_yaw, right_gripper]
```
对于图像而言, 请先按照您当前数据集默认图像尺寸来转化, 然后使用openVLA-oft的脚本进行图像降采样,参考[ALOHA.md line ](./ALOHA.md#fine-tuning-on-aloha-robot-data)的`Fine-Tuning on ALOHA Robot Data`部分.

然后您需要参考修改下面几个文件:
1. [constants.py](prismatic/vla/constants.py)  :  
请根据您实际state维度对照, 修改或添加您的训练信息.  
`NUM_ACTIONS_CHUNK`: 官方建议为您机械臂操控频率(可以和您采样频率一致)
`ACTION_DIM, PROPRIO_DIM`: 对于使用EEF和joint操控, 二者的维度应该是一致的,只有LIBERO比较特殊.
`ACTION_PROPRIO_NORMALIZATION_TYPE`: 官方提供了三种归一化, 如果您数据比较稳定, 可以使用`BOUNDS`, 默认建议设置为`BOUNDS_Q99`

2. [config.py](prismatic/vla/datasets/rlds/oxe/configs.py#L680)  :  
实现一个你自己的数据声明:
```python
'''
比较有意思的是state_encoding参数没有任何作用
image_obs_keys: 决定你的图像填充顺序, None则padding一个默认值
depth_obs_keys: 默认都是不用于训练的, 不用管, 全设置None就行
state_obs_keys: 按照顺序读取对应key数据, 拼接成完整的tarjectory,如果是None则填充一列0
action_encoding: 决定掩码位置, 只在EEF情况下区分gripper与坐标,joint情况下不区分
'''

# 双臂
# joint, 如果你的state[:14],则不用修改, 否则修改 prismatic/vla/datasets/rlds/oxe/materialize.py#L43, 变成你自己的state维度!
"dual_joint_task_sample": {
    "image_obs_keys": {"primary": "image", "left_wrist": "left_wrist_image", "right_wrist": "right_wrist_image"},
    "depth_obs_keys": {"primary": None, "secondary": None, "wrist": None},
    "state_obs_keys": ["state"],
    "state_encoding": StateEncoding.JOINT_BIMANUAL,
    "action_encoding": ActionEncoding.JOINT_POS_BIMANUAL,
},
# EEF, 官方没有支持, 如果想要自己支持, 那么可以用我修改好的ActionEncoding.DUAL_EEF_POS
"dual_eef_task_sample": {
    "image_obs_keys": {"primary": "image", "left_wrist": "left_wrist_image", "right_wrist": "right_wrist_image"},
    "depth_obs_keys": {"primary": None, "secondary": None, "wrist": None},
    "state_obs_keys": ["state"],
    "state_encoding": StateEncoding.NONE,
    "action_encoding": ActionEncoding.DUAL_EEF_POS,
},

# 单臂
# joint, 如果你的state维度 < 8, 请后面填充None, 示例: 4自由度+1gripper总共5维的state ,要填充三个None到8维度
"single_joint_task_sample": {
    "image_obs_keys": {"primary": "image", "secondary": "image_1", "wrist": None},
    "depth_obs_keys": {"primary": None, "secondary": None, "wrist": None},
    "state_obs_keys": ["state", None, None, None],
    "state_encoding": StateEncoding.JOINT,
    "action_encoding": ActionEncoding.JOINT_POS,
},

# EEF, 由于我们已经拼接好了, 不需要重新拼接参数, 所以直接设置为state就行
"single_eef_task_sample": {
    "image_obs_keys": {"primary": "image", "secondary": "image_1", "wrist": None},
    "depth_obs_keys": {"primary": None, "secondary": None, "wrist": None},
    "state_obs_keys": ["state"],
    "state_encoding": StateEncoding.POS_EULER,
    "action_encoding": ActionEncoding.EEF_POS,
},
```

3. [transforms.py](prismatic/vla/datasets/rlds/oxe/transforms.py#L928)
有些数据集由于构成比较特殊, 因此用于训练的state需要重新提取特征并拼接, 如果你的state直接按照我推荐的进行, 那么就直接使用`aloha_dataset_transform`来return tarj就行.

4. [mixtures.py](prismatic/vla/datasets/rlds/oxe/mixtures.py#L230)
进行训练的数据配比,参考一下就行.

## 训练模型
```bash
torchrun --standalone --nnodes 1 --nproc-per-node X vla-scripts/finetune.py \
  --vla_path openvla/openvla-7b \
  --data_root_dir /PATH/TO/RLDS/DATASETS/DIR/ \
  --dataset_name aloha1_put_X_into_pot_300_demos \
  --run_root_dir /YOUR/CHECKPOINTS/AND/LOG/DIR/ \
  --use_l1_regression True \
  --use_diffusion False \
  --use_film True \
  --num_images_in_input 3 \
  --use_proprio True \
  --batch_size 4 \
  --learning_rate 5e-4 \
  --num_steps_before_decay 50000 \
  --max_steps 100005 \
  --use_val_set True \
  --val_freq 10000 \
  --save_freq 10000 \
  --save_latest_checkpoint_only False \
  --image_aug True \
  --lora_rank 32 \
  --wandb_entity "YOUR_WANDB_ENTITY" \
  --wandb_project "YOUR_WANDB_PROJECT" \
  --run_id_note parallel_dec--25_acts_chunk--continuous_acts--L1_regression--3rd_person_img--left_right_wrist_imgs--proprio_state--film
```
