---
license: mit
language:
- en
pipeline_tag: robotics
library_name: transformers
tags:
- robotics
- pytorch
- multimodal
- pretraining
- vla
- diffusion
- rdt
---
# RDT-1B

![](head.mp4)
 RDT-1B is a 1B-parameter imitation learning Diffusion Transformer pre-trained on 1M+ multi-robot episodes. Given language instruction and RGB images of up to three views, RDT can predict the next 
64 robot actions. RDT is compatible with almost all modern mobile manipulators, from single-arm to dual-arm, joint to EEF, position to velocity, and even wheeled locomotion.

 All the [code](https://github.com/thu-ml/RoboticsDiffusionTransformer/tree/main?tab=readme-ov-file), pre-trained model weights, and [data](https://huggingface.co/datasets/robotics-diffusion-transformer/rdt-ft-data) are licensed under the MIT license.

 Please refer to our [project page](https://rdt-robotics.github.io/rdt-robotics/) and [paper](https://arxiv.org/pdf/2410.07864) for more information.

 ## Model Details

 - **Developed by:** The RDT team consisting of researchers from the [TSAIL group](https://ml.cs.tsinghua.edu.cn/) at Tsinghua University
- **Task Type:** Vision-Language-Action (language, image => robot actions)
- **Modle Type:** Diffusion Policy with Transformers
- **License:** MIT
- **Language(s) (NLP):** en
- **Multi-Modal Encoders:**
  - **Vision Backbone:** [siglip-so400m-patch14-384](https://huggingface.co/google/siglip-so400m-patch14-384)
  - **Language Model:** [t5-v1_1-xxl](https://huggingface.co/google/t5-v1_1-xxl)
- **Pre-Training Datasets:** 46 datasets consisting of [RT-1 Dataset](https://robotics-transformer1.github.io/), [RH20T](https://rh20t.github.io/), [DROID](https://droid-dataset.github.io/), [BridgeData V2](https://rail-berkeley.github.io/bridgedata/), [RoboSet](https://robopen.github.io/roboset/), and a subset of [Open X-Embodiment](https://robotics-transformer-x.github.io/). See [this link](https://github.com/thu-ml/RoboticsDiffusionTransformer/blob/main/docs/pretrain.md#download-and-prepare-datasets) for a detailed list.
- **Repository:** https://github.com/thu-ml/RoboticsDiffusionTransformer
- **Paper :** https://arxiv.org/pdf/2410.07864
- **Project Page:** https://rdt-robotics.github.io/rdt-robotics/

 ## Uses

RDT takes language instruction, RGB images (of up to three views), control frequency (if any), and proprioception as input and predicts the next 64 robot actions.
RDT supports control of almost all robot manipulators with the help of the unified action space, which 
includes all the main physical quantities of the robot manipulator (e.g., the end-effector and joint, position and velocity, and the wheeled locomotion). 
To deploy on your robot platform, you need to fill the relevant quantities of the raw action vector into the unified space vector. See [our repository](https://github.com/thu-ml/RoboticsDiffusionTransformer) for more information.

 **Out-of-Scope**: Due to the embodiment gap, RDT cannot yet generalize to new robot platforms (not seen in the pre-training datasets). 
In this case, we recommend collecting a small dataset of the target robot and then using it to fine-tune RDT.
See [our repository](https://github.com/thu-ml/RoboticsDiffusionTransformer) for a tutorial.

Here's an example of how to use the RDT-1B model for inference on a robot:
```python
# Please first clone the repository and install dependencies
# Then switch to the root directory of the repository by "cd RoboticsDiffusionTransformer"

# Import a create function from the code base
from scripts.agilex_model import create_model

# Names of cameras used for visual input
CAMERA_NAMES = ['cam_high', 'cam_right_wrist', 'cam_left_wrist']
config = {
    'episode_len': 1000,  # Max length of one episode
    'state_dim': 14,      # Dimension of the robot's state
    'chunk_size': 64,     # Number of actions to predict in one step
    'camera_names': CAMERA_NAMES,
}
pretrained_vision_encoder_name_or_path = "google/siglip-so400m-patch14-384" 
# Create the model with the specified configuration
model = create_model(
    args=config,
    dtype=torch.bfloat16, 
    pretrained_vision_encoder_name_or_path=pretrained_vision_encoder_name_or_path,
    pretrained='robotics-diffusion-transformer/rdt-1b',
    control_frequency=25,
)

# Start inference process
# Load the pre-computed language embeddings
# Refer to scripts/encode_lang.py for how to encode the language instruction
lang_embeddings_path = 'your/language/embedding/path'
text_embedding = torch.load(lang_embeddings_path)['embeddings']  
images: List(PIL.Image) = ... #  The images from last 2 frames
proprio = ... # The current robot state
# Perform inference to predict the next `chunk_size` actions
actions = policy.step(
    proprio=proprio,
    images=images,
    text_embeds=text_embedding 
)
```

 <!-- RDT-1B supports finetuning on custom datasets, deploying and inferencing on real robots, and retraining the model.
Please refer to [our repository](https://github.com/GeneralEmbodiedSystem/RoboticsDiffusionTransformer/blob/main/docs/pretrain.md) for all the above guides. -->


## Citation

If you find our work helpful, please cite us:
```bibtex
@article{liu2024rdt,
  title={RDT-1B: a Diffusion Foundation Model for Bimanual Manipulation},
  author={Liu, Songming and Wu, Lingxuan and Li, Bangguo and Tan, Hengkai and Chen, Huayu and Wang, Zhengyi and Xu, Ke and Su, Hang and Zhu, Jun},
  journal={arXiv preprint arXiv:2410.07864},
  year={2024}
}
```
Thank you!
