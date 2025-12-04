import torch
import torchvision


def get_resnet(name, weights=None, **kwargs):
    """
    name: resnet18, resnet34, resnet50
    weights: "IMAGENET1K_V1", "r3m"
    """
    # load r3m weights
    if (weights == "r3m") or (weights == "R3M"):
        return get_r3m(name=name, **kwargs)

    func = getattr(torchvision.models, name)
    resnet = func(weights=weights, **kwargs)
    resnet.fc = torch.nn.Identity()
    # resnet_new = torch.nn.Sequential(
    #     resnet,
    #     torch.nn.Linear(512, 128)
    # )
    # return resnet_new
    return resnet


def get_r3m(name, **kwargs):
    """
    name: resnet18, resnet34, resnet50
    """
    import r3m

    r3m.device = "cpu"
    model = r3m.load_r3m(name)
    r3m_model = model.module
    resnet_model = r3m_model.convnet
    resnet_model = resnet_model.to("cpu")
    return resnet_model

def get_dinov2(model_name: str = "dinov2_vitb14", **kwargs) -> torch.nn.Module:
    """
    加载 DINOv2 视觉Transformer模型
    model_name: 可选 dinov2_vits14(小), dinov2_vitb14(基础), 
                dinov2_vitl14(大), dinov2_vitg14(超大)
    weights: 固定为 None (DINOv2 预训练权重自动加载)
    """
    # 从 PyTorch Hub 加载官方模型
    model = torch.hub.load(
        repo_or_dir='facebookresearch/dinov2',
        model=model_name,
        source='github',
        **kwargs
    )
    
    # 移除分类头（保留特征提取层）
    if hasattr(model, 'head'):
        model.head = torch.nn.Identity()
    
    return model