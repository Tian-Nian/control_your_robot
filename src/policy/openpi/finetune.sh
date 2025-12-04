train_config_name=$1
model_name=$2
gpu_use=$3

uv run scripts/compute_norm_stats.py --config-name $train_config_name

export CUDA_VISIBLE_DEVICES=$gpu_use

XLA_PYTHON_CLIENT_MEM_FRACTION=0.9 uv run scripts/train.py $train_config_name --exp-name=$model_name --overwrite