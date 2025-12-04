[![中文](https://img.shields.io/badge/中文-简体-blue)](./README.md)  
[![English](https://img.shields.io/badge/English-English-green)](./README_EN.md)

## How to train with openpi

1. Data collection  
You can collect data using the provided API and store it with `CollectAny`, or use your own collected data.

2. Convert data formats  
First convert to the unified HDF5 format so it can be turned into a LeRobotDataset:

```bash
python scripts/convert2hdf5.py input_path output_path
# example:
# python scripts/convert2hdf5.py ../../save/task_1/ processed_data/task_1/
```
Then move the corresponding `instructions.json` for the task into the task folder. See `task_instructions/*.json` for the expected JSON format.

Finally, convert to the LeRobotDataset format as needed. The converter supports converting multiple tasks at once, or converting a single specified task:
``` bash
python scripts/convert2lerobot.py --raw_dir data_dir --repo_id your_repo_id # --is_multi
# Single-dataset example:
python scripts/convert2lerobot.py --raw_dir processed_data/task_1/ --repo_id my_task_1
# Multi-dataset example:
python scripts/convert2lerobot.py --raw_dir processed_data/ --repo_id union_task --is_multi
```
3. Pick your config
In `src/openpi/training/config.py`, choose the training configuration you want (single-arm / dual-arm, base / fast, full / lora).
Set repo_id to the same repo_id you used when converting data.
Adjust other training parameters as needed, for example:

`batch_size`: total training batch size. Larger values require more GPU memory; 32 is a reasonable starting point.

`num_train_steps`: 30k steps typically suffice for convergence; increase if you want more training.

`fsdp_devices`: enable multi-GPU FSDP if a single GPU lacks memory. Note: FSDP shards a single model across GPUs (it does not create full model copies per GPU).

**IMPORTANT!!!**

Make sure the action output dimension of the policy matches your robot's action dimension. Edit the policy output sizes accordingly.

For example, if your robot uses 7 joint values + 1 gripper value (7+1):

Single-arm (libero) output should be [:8]

Dual-arm (aloha) output should be [:16]

4. Run finetune.sh to start training

your_train_config_name should match a config key in `config.py` (_CONFIGS).

your_model_name is an arbitrary name used for WandB and saved model names.

gpu_id is the GPU id(s) to use (single-GPU: 0).

## How to run inference with openpi
inference_model.py contains deployment wrappers for single-arm and dual-arm setups. Align the inputs/outputs to your robot hardware. You must update two parts:

Set `train_config_name` to the `train_config` used for the model.

In `src/openpi/training/config.py`, set the `repo_id` of the train_config to the dataset `repo_id` you used during training.