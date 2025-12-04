[![中文](https://img.shields.io/badge/中文-简体-blue)](./README.md)  
[![English](https://img.shields.io/badge/English-English-green)](./README_EN.md)

## 如何使用openpi进行训练
1. 采集数据  
可以使用提供的api进行数据采集, 使用CollectAny进行数据存储, 或者用自己的采集的数据.  

2. 转化数据格式  
首先转化为通用hdf5格式, 方便转为lerobotdataset:
``` bash
python scripts/convert2hdf5.py input_path output_path
# example: python scripts/convert2hdf5.py ../../save/task_1/ processed_data/task_1/
```
然后将对应任务的instructions.json移动到对应任务的文件夹中, json文件格式参考`task_instructions/*.json`.

最后根据需求, 转化为lerobotdataset格式, 支持多个任务同时转化, 或者只转化指定任务:
``` bash
python scripts/convert2lerobot.py --raw_dir data_dir --repo_id your_repo_id # --is_multi
# 如果你是单数据集转化
python scripts/convert2lerobot.py --raw_dir processed_data/task_1/ --repo_id my_task_1
# 如果你输多数据集转化 
python scripts/convert2lerobot.py --raw_dir processed_data/ --repo_id union_task --is_multi
```

3. 选择你的config  
在`src/openpi/training/config.py`中, 选择你要使用的训练形式:单臂/双臂, base/fast, full/lora  
修改repo_id为你转化数据时使用的repo_id  
修改一些你想设置的参数如:  
`batch_size`: 训练的总batch size.越大需要的显存越高, 建议开32, 效果不错  
`num_train_steps`: 30000步基本都能收敛, 不放心可以开高点  
`fsdp_devices`: 如果你单卡显存不够, 你可以开多卡, 注意!fsdp是将单个模型平均分到多卡上,不是每张卡一个完整模型  

### 注意!!!  
请认真校对你机械臂所需要的action维度, 修改对应policy的output,将输出维度修改对齐你的机械臂维度  
如你是7+1的机械臂, 那么单臂使用的libero的output是[:8], 双臂使用的aloha是[:16]  

4. 运行`finetune.sh`开启训练  
`your_train_config_name`是`config.py`的`_CONFIGS`中的配置信息   
`your_model_name`可以随便起, 会决定wandb的模型名称与保存的模型名称,  
`gpu_id`是你要使用的gpu对应的id,单卡填0  
```bash
# 如bash finetune.sh pi0_single_base_full my_model 0,1,2
bash finetune.sh your_train_config_name your_model_name gpu_id
```

## 如何使用openpi进行推理  
在`inference_model.py`中给出了单臂和双臂的部署封装类, 请结合自己的机器人进行数据对齐. 需要修改两个部分:
1. `train_config_name`要设置为你训练时使用的`train_config`
2. 要在`src/openpi/training/config.py`对应的`train_config`中将`repo_id`改为你模型训练时使用的数据集的`repo_id`
