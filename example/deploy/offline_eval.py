
import os
import importlib
import argparse
import numpy as np
import time
import glob
import random
import matplotlib.pyplot as plt
import seaborn as sns
from matplotlib.lines import Line2D

from robot.utils.base.data_handler import debug_print, is_enter_pressed, hdf5_groups_to_dict
from robot.robot.base_robot import dict_to_list

### =========THE PLACE YOU COULD MODIFY=========
# eval setting
DRAW = True
DRAW_DIR = "save/picture/test/"
MODEL_CHUNK_SIZE = 3
SKIP_FRAMWE = 20

def input_transform(data):
    state = np.concatenate([
        np.array(data[0]["left_arm"]["joint"]).reshape(-1),
        np.array(data[0]["left_arm"]["gripper"]).reshape(-1),
        np.array(data[0]["right_arm"]["joint"]).reshape(-1),
        np.array(data[0]["right_arm"]["gripper"]).reshape(-1)
    ])

    img_arr = data[1]["cam_head"]["color"], data[1]["cam_right_wrist"]["color"], data[1]["cam_left_wrist"]["color"]
    return img_arr, state

def compare_transform(data_chunk):
    actions = []

    for data in data_chunk[0]:
        # check if single or dual
        if "left_arm" in data and "right_arm" not in data:
            left_joint = np.array(data["left_arm"]["joint"]).reshape(-1)
            left_gripper = np.array(data["left_arm"]["gripper"]).reshape(-1)
            
            right_joint = np.zeros_like(left_joint)
            right_gripper = np.zeros_like(left_gripper)
            
            action = np.concatenate([
                left_joint,
                left_gripper,
                right_joint,
                right_gripper
            ])
        else:
            action = np.concatenate([
                np.array(data["left_arm"]["joint"]).reshape(-1),
                np.array(data["left_arm"]["gripper"]).reshape(-1),
                np.array(data["right_arm"]["joint"]).reshape(-1),
                np.array(data["right_arm"]["gripper"]).reshape(-1)
            ])
        actions.append(action)

    return np.stack(actions)

def compute_similarity(action_chunk_pred, action_chunk_real):
    dist = np.linalg.norm(action_chunk_pred - action_chunk_real)
    sim = 1 / (1 + dist)
    return sim

def compute_statistics(all_pred_actions, all_real_actions, all_time_chunks):
    """
    Compute statistics for all episodes: mean trajectories, mean differences, variance, etc.
    """
    # Find the maximum number of time steps to align all trajectories
    max_steps = 0
    for pred_actions in all_pred_actions:
        total_steps = sum(chunk.shape[0] for chunk in pred_actions)
        max_steps = max(max_steps, total_steps)
    
    action_dim = all_pred_actions[0][0].shape[1]
    
    all_pred_aligned = []
    all_real_aligned = []
    
    for i, (pred_chunks, real_chunks) in enumerate(zip(all_pred_actions, all_real_actions)):
        pred_traj = np.concatenate(pred_chunks, axis=0)  # (total_steps, action_dim)
        real_traj = np.concatenate(real_chunks, axis=0)  # (total_steps, action_dim)
        
        # Align trajectories to the same length (truncate or pad)
        current_steps = pred_traj.shape[0]
        if current_steps < max_steps:
            pred_pad = np.tile(pred_traj[-1:], (max_steps - current_steps, 1))
            real_pad = np.tile(real_traj[-1:], (max_steps - current_steps, 1))
            pred_traj = np.concatenate([pred_traj, pred_pad], axis=0)
            real_traj = np.concatenate([real_traj, real_pad], axis=0)
        
        elif current_steps > max_steps:
            pred_traj = pred_traj[:max_steps]
            real_traj = real_traj[:max_steps]
        
        all_pred_aligned.append(pred_traj)
        all_real_aligned.append(real_traj)
    
    all_pred_aligned = np.stack(all_pred_aligned)
    all_real_aligned = np.stack(all_real_aligned)
    
    # Compute statistics
    # Mean trajectory over all episodes (max_steps, action_dim)
    mean_pred_traj = np.mean(all_pred_aligned, axis=0)
    mean_real_traj = np.mean(all_real_aligned, axis=0)
    
    # Compute differences between predicted and real actions (num_episodes, max_steps, action_dim)
    differences = all_pred_aligned - all_real_aligned
    
    # Mean and variance of differences per time step and per dimension (max_steps, action_dim)
    mean_diff = np.mean(differences, axis=0)
    var_diff = np.var(differences, axis=0)
    std_diff = np.std(differences, axis=0)
    
    # Mean absolute difference per time step and per dimension (max_steps, action_dim)
    mean_abs_diff = np.mean(np.abs(differences), axis=0)
    
    return {
        'mean_pred_traj': mean_pred_traj,
        'mean_real_traj': mean_real_traj,
        'mean_diff': mean_diff,
        'var_diff': var_diff,
        'std_diff': std_diff,
        'mean_abs_diff': mean_abs_diff,
        'max_steps': max_steps,
        'action_dim': action_dim
    }


### =========THE PLACE YOU COULD MODIFY=========

class Replay:
    def __init__(self, hdf5_path) -> None:
        self.ptr = 0
        self.episode = dict_to_list(hdf5_groups_to_dict(hdf5_path))
    
    def get_data(self):
        try:
            start_ptr = self.ptr
            data = self.episode[self.ptr], self.episode[self.ptr]
            data_chunk_end_ptr = min(len(self.episode), self.ptr+MODEL_CHUNK_SIZE)
            data_chunk = self.episode[self.ptr:data_chunk_end_ptr], self.episode[self.ptr:data_chunk_end_ptr]
            self.ptr += SKIP_FRAMWE
        except:
            return None, None, None
        return data, data_chunk, (start_ptr, data_chunk_end_ptr)

def get_class(import_name, class_name):
    try:
        class_module = importlib.import_module(import_name)
        debug_print("function", f"Module loaded: {class_module}", "DEBUG")
    except ModuleNotFoundError as e:
        raise SystemExit(f"ModuleNotFoundError: {e}")

    try:
        return_class = getattr(class_module, class_name)
        debug_print("function", f"Class found: {return_class}", "DEBUG")

    except AttributeError as e:
        raise SystemExit(f"AttributeError: {e}")
    except Exception as e:
        raise SystemExit(f"Unexpected error instantiating model: {e}")
    return return_class

def eval_once(model, episode):
    replay = Replay(episode)

    similaritys = []
    action_chunk_preds = []
    action_chunk_reals = []
    time_step_chunks = []
    while True:
        # time_step_chunk = (replay.ptr, replay.ptr + MODEL_CHUNK_SIZE)
        data, data_chunk, time_step_chunk = replay.get_data()
        if data is None:
            break

        img_arr, state = input_transform(data)
        model.update_observation_window(img_arr, state)
        action_chunk_pred = model.get_action()
        action_chunk_real = compare_transform(data_chunk)
        if action_chunk_real.shape[0] != action_chunk_pred.shape[0]:
            action_chunk_pred = action_chunk_pred[:action_chunk_real.shape[0]]
        similarity = compute_similarity(action_chunk_pred, action_chunk_real)

        similaritys.append(similarity)
        action_chunk_preds.append(action_chunk_pred)
        action_chunk_reals.append(action_chunk_real)
        time_step_chunks.append(time_step_chunk)

    if DRAW:
        if not os.path.exists(DRAW_DIR):
            os.makedirs(DRAW_DIR)
        
        pic_name = os.path.basename(episode).split(".")[0] + ".png"

        plot_trajectories_subplots(action_chunk_preds, action_chunk_reals, time_step_chunks, os.path.join(DRAW_DIR, pic_name)) 
        # plot_trajectories_subplots([similarity], None, time_step_chunks, save_path = os.path.join(DRAW_DIR, "similarity_" + pic_name)) 

def plot_trajectories_subplots(trajA, trajB, time_intervals, save_path="traj_subplots.png"):
    """
    Plot each dimension of trajectories A and B in subplots, supporting multiple segments
    and different time intervals.
    
    Args:
        trajA, trajB: list of np.ndarray, each segment shape (seg_len, num_dims), can be None
        time_intervals: list of tuples (a, b) indicating the time range of each segment
        save_path: path to save the final figure
    """
    sns.set_style("whitegrid")  # Use seaborn style

    # Check that at least one trajectory exists
    if trajA is None and trajB is None:
        print("No trajectories to plot.")
        return

    # Determine number of dimensions
    sample_traj = trajA if trajA is not None else trajB
    num_dims = sample_traj[0].shape[1]

    # Dynamically adjust figure height based on number of dimensions
    height_per_dim = max(3, 4 - num_dims * 0.1)
    fig, axes = plt.subplots(num_dims, 1, figsize=(16, height_per_dim*num_dims), sharex=True)
    if num_dims == 1:
        axes = [axes]

    # Color scheme: A = blue, B = yellow
    colors = {"A": "#1E90FF", "B": "#FFD700"}

    num_segments = len(time_intervals) if time_intervals is not None else len(sample_traj)

    for seg_idx in range(num_segments):
        a, b = time_intervals[seg_idx] if time_intervals is not None else (0, sample_traj[seg_idx].shape[0]-1)
        t = np.linspace(a, b, sample_traj[seg_idx].shape[0])

        if trajA is not None:
            segA = trajA[seg_idx]
            for dim in range(num_dims):
                # Plot trajectory with bold line, no legend label
                sns.lineplot(x=t, y=segA[:, dim], ax=axes[dim], 
                             color=colors["A"], alpha=0.8, linewidth=10, label='')

        if trajB is not None:
            segB = trajB[seg_idx]
            for dim in range(num_dims):
                sns.lineplot(x=t, y=segB[:, dim], ax=axes[dim], 
                             color=colors["B"], alpha=0.8, linewidth=10, label='')

    # Format each subplot
    for dim in range(num_dims):
        axes[dim].grid(True, alpha=0.3)
        axes[dim].spines['top'].set_visible(False)
        axes[dim].spines['right'].set_visible(False)
        axes[dim].tick_params(axis='both', which='major', labelsize=14)

    # X-axis label
    axes[-1].set_xlabel("Time Step (Frame)", fontsize=16)

    # Adjust layout and save figure
    plt.subplots_adjust(left=0.08, bottom=0.08, right=0.95, top=0.95, hspace=0.3)
    plt.savefig(save_path, dpi=300, bbox_inches='tight', facecolor='white', edgecolor='none')
    plt.close()
    print(f"Saved trajectory figure to {save_path}")

def init():
    parser = argparse.ArgumentParser()
    
    parser.add_argument("--model_name", type=str, required=True, help="Name of the task") 
    parser.add_argument("--model_class", type=str, required=True, help="Name of the model class")
    parser.add_argument("--model_path", type=str, required=True, help="model path, e.g., policy/RDT/checkpoints/checkpoint-10000")
    parser.add_argument("--task_name", type=str, required=True, help="task name, read intructions from task_instuctions/{task_name}.json")
    parser.add_argument("--data_path", type=str, required=True, help="the data you want to eval")
    parser.add_argument("--episode_num", type=int, required=False,default=10, help="how many episode you want to eval")
    
    args = parser.parse_args()
    model_name = args.model_name
    model_class = args.model_class
    model_path = args.model_path
    task_name = args.task_name
    data_path = args.data_path
    episode_num = args.episode_num

    model_class = get_class(f"robot.policy.{model_name}.inference_model", model_class)
    model = model_class(model_path, task_name)

    if os.path.isfile(data_path):
        return model, [data_path]   
    else:
        all_files = glob.glob(os.path.join(data_path, "*.hdf5"))
        if episode_num > len(all_files):
            raise IndexError(f"episode_num > data_num : {episode_num} > len(all_files)")

        # 随机选取
        episodes = random.sample(all_files, episode_num)

        return model, episodes

if __name__ == "__main__":
    os.environ["INFO_LEVEL"] = "INFO" # DEBUG , INFO, ERROR

    model, episodes = init()

    for episode in episodes:
        eval_once(model, episode)
