#!/usr/bin/env python3
"""Run a JAX pi05 policy behind the simple TCP ModelServer.

The script can also run a random-input smoke test:

    python example/deploy/pi05_jax_model_server.py --smoke-test
"""

from __future__ import annotations

import argparse
import dataclasses
from pathlib import Path
import sys
from typing import Any

import numpy as np


THIS_FILE = Path(__file__).resolve()
DEPLOY_DIR = THIS_FILE.parent
REPO_ROOT = DEPLOY_DIR.parent.parent
OPENPI_DIR = REPO_ROOT / "src" / "robot" / "policy" / "openpi"

for path in (DEPLOY_DIR, REPO_ROOT, OPENPI_DIR / "src"):
    path_str = str(path)
    if path_str not in sys.path:
        sys.path.insert(0, path_str)

from robot_client_server.model_server import ModelServer

import jax
import jax.numpy as jnp
import openpi.models.model as _model
import openpi.policies.policy as _policy
from openpi.policies import policy_config as _policy_config
from openpi.shared import download
from openpi.training import checkpoints as _checkpoints
from openpi.training import config as _config
import openpi.transforms as transforms


DEFAULT_CHECKPOINT_DIR = OPENPI_DIR / "checkpoints" / "35000"
DEFAULT_CONFIG = "pi05_full_base"
DEFAULT_ASSET_ID = "charge_insert_norm"
DEFAULT_PROMPT = "charge insert"
_RESTORE_DTYPES = {
    "bfloat16": jnp.bfloat16,
    "float32": jnp.float32,
}


def _load_train_config(config_name: str, asset_id: str, model_dtype: str) -> _config.TrainConfig:
    """Load the pi05 train config and point its norm-stat asset id at this checkpoint."""
    train_config = _config.get_config(config_name)
    model_config = train_config.model
    if hasattr(model_config, "dtype"):
        model_config = dataclasses.replace(model_config, dtype=model_dtype)
    data_config = dataclasses.replace(
        train_config.data,
        assets=_config.AssetsConfig(asset_id=asset_id),
    )
    return dataclasses.replace(train_config, model=model_config, data=data_config)


def _create_jax_policy(
    train_config: _config.TrainConfig,
    checkpoint_dir: str | Path,
    *,
    restore_dtype: str,
    sample_num_steps: int | None,
) -> _policy.Policy:
    """Create a JAX policy while making the checkpoint restore dtype configurable."""
    checkpoint_dir = download.maybe_download(str(checkpoint_dir))
    weight_path = Path(checkpoint_dir) / "model.safetensors"
    if weight_path.exists():
        raise ValueError(f"This server expects a JAX checkpoint, but found a PyTorch checkpoint: {weight_path}")

    dtype = _RESTORE_DTYPES[restore_dtype]
    model = train_config.model.load(_model.restore_params(checkpoint_dir / "params", dtype=dtype))
    data_config = train_config.data.create(train_config.assets_dirs, train_config.model)
    if data_config.asset_id is None:
        raise ValueError("Asset id is required to load norm stats.")
    norm_stats = _checkpoints.load_norm_stats(checkpoint_dir / "assets", data_config.asset_id)

    sample_kwargs = {}
    if sample_num_steps is not None:
        sample_kwargs["num_steps"] = sample_num_steps

    return _policy.Policy(
        model,
        transforms=[
            transforms.InjectDefaultPrompt(None),
            *data_config.data_transforms.inputs,
            transforms.Normalize(norm_stats, use_quantiles=data_config.use_quantile_norm),
            *data_config.model_transforms.inputs,
        ],
        output_transforms=[
            *data_config.model_transforms.outputs,
            transforms.Unnormalize(norm_stats, use_quantiles=data_config.use_quantile_norm),
            *data_config.data_transforms.outputs,
        ],
        sample_kwargs=sample_kwargs,
        metadata=train_config.policy_metadata,
        is_pytorch=False,
        pytorch_device=None,
    )


def make_random_pi05_observation(
    *,
    prompt: str = DEFAULT_PROMPT,
    image_height: int = 224,
    image_width: int = 224,
    seed: int | None = 0,
) -> dict[str, Any]:
    """Create one unbatched observation in the format expected by the Aloha pi05 policy."""
    rng = np.random.default_rng(seed)
    image_shape = (3, image_height, image_width)
    return {
        "state": rng.normal(size=(14,)).astype(np.float32),
        "images": {
            "cam_high": rng.integers(0, 256, size=image_shape, dtype=np.uint8),
            "cam_left_wrist": rng.integers(0, 256, size=image_shape, dtype=np.uint8),
            "cam_right_wrist": rng.integers(0, 256, size=image_shape, dtype=np.uint8),
        },
        "prompt": prompt,
    }


class Pi05JaxModel:
    """Small RPC-facing wrapper around openpi's JAX Policy."""

    def __init__(
        self,
        checkpoint_dir: str | Path,
        *,
        config_name: str = DEFAULT_CONFIG,
        asset_id: str = DEFAULT_ASSET_ID,
        default_prompt: str = DEFAULT_PROMPT,
        restore_dtype: str = "float32",
        model_dtype: str = "float32",
        sample_num_steps: int | None = 1,
    ) -> None:
        self.checkpoint_dir = Path(checkpoint_dir).expanduser().resolve()
        self.config_name = config_name
        self.asset_id = asset_id
        self.default_prompt = default_prompt
        self.restore_dtype = restore_dtype
        self.model_dtype = model_dtype
        self.sample_num_steps = sample_num_steps

        if not self.checkpoint_dir.exists():
            raise FileNotFoundError(f"Checkpoint directory does not exist: {self.checkpoint_dir}")

        print(f"[pi05-server] JAX devices: {jax.devices()}")
        print(f"[pi05-server] Loading config={config_name!r}, asset_id={asset_id!r}")
        print(
            f"[pi05-server] restore_dtype={restore_dtype!r}, "
            f"model_dtype={model_dtype!r}, sample_num_steps={sample_num_steps!r}"
        )
        print(f"[pi05-server] Loading checkpoint: {self.checkpoint_dir}")

        train_config = _load_train_config(config_name, asset_id, model_dtype)
        if restore_dtype == "bfloat16":
            sample_kwargs = {}
            if sample_num_steps is not None:
                sample_kwargs["num_steps"] = sample_num_steps
            self.policy = _policy_config.create_trained_policy(
                train_config,
                self.checkpoint_dir,
                sample_kwargs=sample_kwargs,
            )
        else:
            self.policy = _create_jax_policy(
                train_config,
                self.checkpoint_dir,
                restore_dtype=restore_dtype,
                sample_num_steps=sample_num_steps,
            )

        print("[pi05-server] Model loaded.")

    def infer(self, obs: dict[str, Any]) -> np.ndarray:
        """RPC method: return an action chunk for one observation."""
        obs = dict(obs)
        obs.setdefault("prompt", self.default_prompt)
        result = self.policy.infer(obs)
        return np.asarray(result["actions"])

    def infer_random(self, obs: dict[str, Any] | None = None) -> np.ndarray:
        """RPC/smoke-test method: run inference on a generated random observation."""
        seed = None
        prompt = self.default_prompt
        if isinstance(obs, dict):
            seed = obs.get("seed", seed)
            prompt = obs.get("prompt", prompt)
        return self.infer(make_random_pi05_observation(prompt=prompt, seed=seed))

    def reset_episode(self, obs: dict[str, Any] | None = None) -> bool:
        """RPC placeholder so clients can mark episode boundaries."""
        del obs
        return True


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="JAX pi05 ModelServer entrypoint.")
    parser.add_argument("--host", default="0.0.0.0", help="TCP bind host.")
    parser.add_argument("--port", type=int, default=9999, help="TCP bind port.")
    parser.add_argument("--checkpoint-dir", default=str(DEFAULT_CHECKPOINT_DIR), help="pi05 checkpoint directory.")
    parser.add_argument("--config", default=DEFAULT_CONFIG, help="OpenPI train config name.")
    parser.add_argument("--asset-id", default=DEFAULT_ASSET_ID, help="Norm stats asset id under checkpoint/assets.")
    parser.add_argument("--prompt", default=DEFAULT_PROMPT, help="Default prompt for inference.")
    parser.add_argument(
        "--restore-dtype",
        default="float32",
        choices=sorted(_RESTORE_DTYPES),
        help="Dtype used when restoring JAX checkpoint params. float32 is safer on this GB10 box.",
    )
    parser.add_argument(
        "--model-dtype",
        default="float32",
        choices=sorted(_RESTORE_DTYPES),
        help="Dtype used by the pi05 model config. float32 is safer on this GB10 box.",
    )
    parser.add_argument(
        "--num-steps",
        type=int,
        default=1,
        help="Number of pi05 sampling steps. Use 1 for smoke tests; increase for real deployment.",
    )
    parser.add_argument("--load-only", action="store_true", help="Load the model and exit without inference.")
    parser.add_argument("--smoke-test", action="store_true", help="Load the model, run one random inference, then exit.")
    parser.add_argument("--seed", type=int, default=0, help="Random seed for --smoke-test.")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    model = Pi05JaxModel(
        args.checkpoint_dir,
        config_name=args.config,
        asset_id=args.asset_id,
        default_prompt=args.prompt,
        restore_dtype=args.restore_dtype,
        model_dtype=args.model_dtype,
        sample_num_steps=args.num_steps,
    )

    if args.load_only:
        print("[pi05-server] Load-only check passed.")
        return

    if args.smoke_test:
        actions = model.infer_random({"seed": args.seed, "prompt": args.prompt})
        print(f"[pi05-server] Smoke-test actions shape: {actions.shape}")
        print(f"[pi05-server] Smoke-test actions dtype: {actions.dtype}")
        print("[pi05-server] First action:")
        print(np.array2string(actions[0], precision=5, suppress_small=False))
        return

    ModelServer(model, host=args.host, port=args.port).start()


if __name__ == "__main__":
    main()
