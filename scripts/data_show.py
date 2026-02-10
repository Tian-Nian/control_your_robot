
from robot.utils.base.data_handler import hdf5_groups_to_dict
import os
import random
import numpy as np
import matplotlib.pyplot as plt
import argparse

import math

def get_random_hdf5(path, n):
    if not os.path.isdir(path):
        raise ValueError(f"{path} 不是文件夹")

    # 获取路径下所有 .hdf5 的相对路径
    hdf5_files = [
        os.path.join(path, f)  # 拼接完整路径
        for f in os.listdir(path)
        if os.path.isfile(os.path.join(path, f)) and f.endswith(".hdf5")
    ]

    if not hdf5_files:
        return []

    if len(hdf5_files) <= n:
        return hdf5_files

    return random.sample(hdf5_files, n)

def read_hdf5(hdf5_path):
    episode = hdf5_groups_to_dict(hdf5_path)
    return episode

def plot_6d_dual_episodes(episode_data_list, save_path, required_keys, args, suptitle=None):
    """
    绘制多 episode 的双臂 6D 数据
    左臂虚线，右臂实线
    不同 episode 颜色不同
    子图布局：2x3 (x, y, z, rx, ry, rz)
    图例显示每条线对应的 episode 和左右臂，放右侧

    Parameters
    ----------
    episode_data_list : list of dict
        每个 dict 格式:
        {
            'x': (left_array, right_array),
            'y': ...,
            'z': ...,
            'rx': ...,
            'ry': ...,
            'rz': ...
        }
        左右数组长度可不一致
    save_path : str
        保存路径
    suptitle : str | None
        总标题
    """
    column = 3
    row =  math.ceil(len(required_keys) / column)
    fig, axes = plt.subplots(row, column, figsize=(12, 6), sharex=False, constrained_layout=True)
    axes = axes.ravel()

    num_eps = len(episode_data_list)
    colors = plt.cm.viridis(np.linspace(0, 1, num_eps))

    legend_handles = []

    for i, key in enumerate(required_keys):
        ax = axes[i]
        for ep_idx, ep_data in enumerate(episode_data_list):
            if args.is_dual:
                left, right = ep_data[key]
            else:
                left, right = ep_data[key], None
            
            max_len = max(len(left), len(right)) if right is not None else len(left)
            t = np.linspace(0, max_len-1, max_len)
            left_plot = np.interp(np.arange(max_len), np.arange(len(left)), left)
            right_plot = np.interp(np.arange(max_len), np.arange(len(right)), right) if right is not None else None
            # 绘图
            l_line, = ax.plot(t, left_plot,  linestyle='--', color=colors[ep_idx])
            if right is not None:
                r_line, = ax.plot(t, right_plot, linestyle='-.',  color=colors[ep_idx])
            else:
                r_line = None
            
            # 只在第一个子图收集 legend
            if i == 0:
                legend_handles.append((l_line, f"Left arm ep{ep_idx+1}"))
                legend_handles.append((r_line, f"Right arm ep{ep_idx+1}")) if right is not None else None
        ax.set_title(required_keys[i])
        ax.set_ylabel(required_keys[i])
        ax.grid(True, linestyle='--', alpha=0.3)

    axes[3].set_xlabel('t')
    axes[4].set_xlabel('t')
    axes[5].set_xlabel('t')

    # 删掉多余的图
    for j in range(len(required_keys), row*column):
        fig.delaxes(axes[j])
    
    # 添加图例
    lines, labels = zip(*legend_handles)
    fig.legend(lines, labels, loc='center left', bbox_to_anchor=(1.02,0.5), fontsize=9)

    if suptitle:
        fig.suptitle(suptitle, y=0.98)

    plt.savefig(save_path, dpi=300, bbox_inches='tight')
    plt.close(fig)

def parse_args():
    parser = argparse.ArgumentParser(description="参数读取示例")

    parser.add_argument(
        "--path",
        type=str,
        required=True,
        help="路径，文件或文件夹"
    )

    parser.add_argument(
        "--option",
        type=str,
        nargs="+",  # 接收一个或多个值
        required=False,
        default=["qpos", "joint", "gripper"],
        help="选项列表，空格分隔，例如 --option a b c"
    )

    parser.add_argument(
        "--is_dual",
        type=bool,
        required=False,
        default=False,
        help="是否为双臂"
    )

    parser.add_argument(
        "--joint_dim",
        type=int,
        required=False,
        default=6,
        help="机械臂的关节数(单臂)"
    )

    parser.add_argument(
        "--gripper_dim",
        type=int,
        required=False,
        default=1,
        help="夹爪维度数"
    )

    return parser.parse_args()

# ======= 使用示例 =======
if __name__ == "__main__":
    args = parse_args()
    path = args.path
    my_options = args.option
    is_dual = args.is_dual
    joint_dim = args.joint_dim
    gripper_dim = args.gripper_dim

    if os.path.isdir(path):
        paths = get_random_hdf5(path, 1)
    
    options = {
                "qpos": {
                    "keys": ['x', 'y', 'z', 'rz', 'ry', 'rz'],
                }, 
               "joint": {
                    "keys": [f"joint_{i}" for i in range(joint_dim)],
                }, 
               "gripper": {
                    "keys": [f"gripper_{i}" for i in range(gripper_dim)],
                }, 
            }
    required_keys = []
    for key in options.keys():
        if key in my_options:
            for k in options[key]["keys"]:
                required_keys.append(k)

    data_list = []

    for p in paths:
        episode = read_hdf5(p)
        t = np.linspace(0, len(episode["right_arm"]["qpos"])-1, len(episode["right_arm"]["qpos"]))
        data = {}
        data = {'t': t}
        for opt in my_options:
            keys = options[opt]["keys"]

            for i in range(len(keys)):
                
                if is_dual:
                    if len(keys) > 1:
                        data[keys[i]] = (episode["left_arm"][opt][:,i].flatten(), episode["right_arm"][opt][:,i].flatten())
                    else:
                        data[keys[i]] = (episode["left_arm"][opt][0].flatten(), episode["right_arm"][opt][0].flatten())
                else:
                    if len(keys) > 1:
                        data[keys[i]] = (episode["left_arm"][opt][:,i].flatten())
                    else:
                        data[keys[i]] = (episode["left_arm"][opt][0].flatten())
        
        data_list.append(data)
    plot_6d_dual_episodes(data_list, "save/result_multi.jpg", required_keys, args, suptitle="Dual-arm 6-DOF Trajectory")
