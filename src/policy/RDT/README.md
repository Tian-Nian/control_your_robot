[![中文](https://img.shields.io/badge/中文-简体-blue)](./README.md)  
[![English](https://img.shields.io/badge/English-English-green)](./README_EN.md)

## 1. 配置环境
conda环境参考RDT官方环境:([RDT official documentation](https://github.com/thu-ml/RoboticsDiffusionTransformer)).

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
## 2. 下载模型

```bash
# In the RoboTwin/policy directory
cd ../weights
mkdir RDT && cd RDT
# Download the models used by RDT
huggingface-cli download google/t5-v1_1-xxl --local-dir t5-v1_1-xxl
huggingface-cli download google/siglip-so400m-patch14-384 --local-dir siglip-so400m-patch14-384
huggingface-cli download robotics-diffusion-transformer/rdt-1b --local-dir rdt-1b
```

## 3. 数据转化

如果你使用了CollectAny类进行了数据存储, 那么可以使用提供的脚本转化数据格式为RDT所使用的hdf5格式, 注意不是默认保存的hdf5!
建议对指令进行提前编码,使用下面指令会进行语言编码:
```bash
python scripts/encode_lang_batch_once.py task_name output_dir gpu_id
```
`task_name`对应`task_config/`中的.json文件前缀
`output_dir`对应你希望保存的文件夹, 会在该文件夹下新建一个`instructions/`保存编码文件
`gpu_id`对应你想使用那个gpu编码, 单卡请设置0

## 4. 生成训练配置文件
 `$model_name` 该名称会决定训练模型的名称, 可以按照自己喜欢来取名
```bash
cd policy/RDT
bash generate.sh ${model_name}
```

该指令将会在`training_data`路径下新建文件夹`/${model_name}/` , 并且在`model_config/`路径下建立`${model_name}.yml`.

**注意!!!**
由于本项目默认适配为6自由度机械臂+1自由度夹爪的双臂机器人, 因此如果:
1. 我使用的是单臂
按照RDT官方, 单臂数据请默认填充在右臂上, 无论机械臂摆放在事业左侧或者右侧
2. 我的机械臂自由度不同
请求改`./data/hdf5_vla_dataset.py`中的line 175与285的`UNI_STATE_INDICES`设置了将数据映射到对应关节角的参数,如7自由度机械臂+1自由度夹爪,单臂:
```
UNI_STATE_INDICES = [
    STATE_VEC_IDX_MAPPING[f"right_arm_joint_{i}_pos"] for i in range(7)
] + [
    STATE_VEC_IDX_MAPPING["right_gripper_open"]
]
```
3. 我想使用EEF控制机械臂
根据`configs/state_vec.py`,选择你想使用的数据类型, 如果是EEF, 请选择:
```python
# 没有right前缀的意思是给单臂使用的, 但是这二者映射的区间相同, 所以可以直接使用right_eef_angle_*
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
需要注意数据的单位:  
`距离:m, 角度:rad, 夹渣:张合度0~1,0 完全关闭, 1 完全打开`

### 4.1 放置训练数据
将数据按照下面格式进行放置, 如果你的训练数据集是联合任务, 每个任务要有一个文件夹, 在文件夹中放置对应的hdf5文件与编码后的指令.

**文件夹示例:**
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

### 4.2 修改对应的`model_config`
在`model_config/${model_name}.yml`中, 你需要设定你想要用来训练的的GPU (modify `cuda_visible_device`). 对于单卡机器, 请设置为 `0`. 对于多卡几环境, 格式设置如 `0,1,4`. 其余的参数可以根据需求自由调整.

## 5. 模型微调

只需要运行下面这条指令就会开始啦:
```bash
bash finetune.sh ${model_name}
```
**注意!!!**

如果你使用单卡训练, DeepSpeed 将不会开启, 因此模型不会保存 `pytorch_model/mp_rank_00_model_states.pt`. 
如果你想接着白村模型继续训练, 请设置 `pretrained_model_name_or_path` 到你想要使用的模型权重的文件夹 `./checkpoints/${model_name}/checkpoint-${ckpt_id}`. 

这将会按照huggingface格式读入模型, 如官方默认模型 `../weights/RDT/rdt-1b`也是这样存储的.

## 6. 模型部署
如果你不是默认的训练设置(关节自由度不同/使用了EEF训练), 那么请按照你的设置, 修改`scripts/agilex_model.py`中的:
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
