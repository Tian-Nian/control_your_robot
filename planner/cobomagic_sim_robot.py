import sapien.core as sapien
from sapien.utils.viewer import Viewer
import numpy as np
import cv2
from pynput import keyboard
import time
import sys
sys.path.append("./")
from planner.curobo_planner import CuroboPlanner

x, y, z =0.0, 0.0, 0.0
move_type = 0

class SimRobot:
    def __init__(self, urdf_path: str, fix_root_link=True, balance_passive_force=True):
        # 初始化引擎、场景、渲染器
        self.engine = sapien.Engine()
        self.renderer = sapien.SapienRenderer()
        self.engine.set_renderer(self.renderer)

        self.scene = self.engine.create_scene(sapien.SceneConfig())
        self.scene.set_timestep(1 / 240.0)
        self.scene.add_ground(0)
        self.scene.set_ambient_light([0.5, 0.5, 0.5])
        self.scene.add_directional_light([0, 1, -1], [0.5, 0.5, 0.5])

        self.viewer = Viewer(self.renderer)
        self.viewer.set_scene(self.scene)
        self.viewer.set_camera_xyz(x=-2, y=0, z=1)
        self.viewer.set_camera_rpy(r=0, p=-0.3, y=0)

        # 加载机器人
        loader = self.scene.create_urdf_loader()
        loader.fix_root_link = fix_root_link
        self.robot = loader.load(urdf_path)
        self.balance_passive_force = balance_passive_force

        # 获取关节信息
        self.active_joints = self.robot.get_active_joints()
        self.active_joint_names = [joint.get_name() for joint in self.active_joints]

        # 设置左右臂关节名称
        self.left_arm_names = [f"fl_joint{i}" for i in range(1, 7)]
        self.right_arm_names = [f"fr_joint{i}" for i in range(1, 7)]

        self.left_arm_indices = [i for i, name in enumerate(self.active_joint_names) if name in self.left_arm_names]
        self.right_arm_indices = [i for i, name in enumerate(self.active_joint_names) if name in self.right_arm_names]

        self.left_base_link_name = "fl_base_link"
        self.right_base_link_name = "fr_base_link"
        self.left_end_effort_name = "fl_link6"
        self.right_end_effort_name = "fr_link6"

        self.left_camera_link_name = "left_camera"
        self.right_camera_link_name = "right_camera"
        self.front_camera_link_name = "camera_link2"

        self.left_wrist_camera = self._setup_camera()
        self.right_wrist_camera = self._setup_camera()
        self.front_camera = self._setup_camera()

    def set_planner(self, left_planner, right_planner):
        self.left_planner = left_planner
        self.right_planner = right_planner

    def _get_link_by_name(self, name: str):
        for link in self.robot.get_links():
            if link.get_name() == name:
                return link
        raise ValueError(f"Link '{name}' not found")

    def _update_camera(self):
        left_wrist_pos_link = self._get_link_by_name(self.left_camera_link_name)
        left_wrist_pos = left_wrist_pos_link.get_pose()

        self.left_wrist_camera.set_pose(left_wrist_pos)

        right_wrist_pos_link = self._get_link_by_name(self.right_camera_link_name)
        right_wrist_pos = right_wrist_pos_link.get_pose()

        self.right_wrist_camera.set_pose(right_wrist_pos)

        front_camrea_pos_link = self._get_link_by_name(self.front_camera_link_name)
        right_wrist_pos = front_camrea_pos_link.get_pose()
        self.front_camera.set_pose(right_wrist_pos)

    def _setup_camera(self):
        near, far = 0.1, 100
        width, height, fovy = 640, 480, 37
        camera = self.scene.add_camera(
            name="wrist_camera",
            width=width,
            height=height,
            fovy=np.deg2rad(fovy),
            near=near,
            far=far
        )
        return camera

    def take_picture(self):
        self._update_camera()
        self.left_wrist_camera.take_picture()

        # 获取 RGBA 图像，float32, (H, W, 4), [0, 1]
        rgba = self.left_wrist_camera.get_picture("Color")
        rgb_uint8 = (rgba[:, :, :3] * 255).astype(np.uint8)

        # 获取 Position 图像 (H, W, 3)，取 Z 通道
        position = self.left_wrist_camera.get_picture("Position")
        depth = position[:, :, 2]

        # 归一化深度为可视化图像
        depth_vis = (depth - depth.min()) / (depth.max() - depth.min() + 1e-6)
        depth_vis = (depth_vis * 255).astype(np.uint8)

        return rgb_uint8, depth_vis

    def set_arm_qpos(self, left_angles=None, right_angles=None):
        qpos = self.robot.get_qpos()
        if left_angles:
            for idx, angle in zip(self.left_arm_indices, left_angles):
                qpos[idx] = angle
        if right_angles:
            for idx, angle in zip(self.right_arm_indices, right_angles):
                qpos[idx] = angle
        self.robot.set_qpos(qpos)

    def run_trajectory(self, joint_indices, trajectory, steps_per_target=1):
        
        for target_angles in trajectory:
            qpos = self.robot.get_qpos()
            for idx, angle in zip(joint_indices, target_angles):
                # import pdb;pdb.set_trace()
                qpos[idx] = angle
            self.robot.set_qpos(qpos)

            for _ in range(steps_per_target):
                if self.balance_passive_force:
                    qf = self.robot.compute_passive_force(gravity=True, coriolis_and_centrifugal=True)
                    self.robot.set_qf(qf)
                self.scene.step()

            self.scene.update_render()
            self.viewer.render()

    def get_joint_positions(self, joint_indices):
        qpos = self.robot.get_qpos()
        return np.array([qpos[i] for i in joint_indices])

    def get_relative_pose(self, base_link_name, end_link_name):
        base = self._get_link_by_name(base_link_name)
        end = self._get_link_by_name(end_link_name)
        return base.get_pose().inv() * end.get_pose()

    def loop(self):
        while not self.viewer.closed:
            # 可选：打印末端执行器相对位置
            # left_pose = self.get_relative_pose("fl_base_link", "fl_link6")
            # right_pose = self.get_relative_pose("fr_base_link", "fr_link6")
            # print("Left arm relative pose:", left_pose)
            # print("Right arm relative pose:", right_pose)

            for _ in range(4):
                if self.balance_passive_force:
                    qf = self.robot.compute_passive_force(gravity=True, coriolis_and_centrifugal=True)
                    self.robot.set_qf(qf)
                self.scene.step()

            self.scene.update_render()
            self.viewer.render()

    def left_move(self,delta_move):
        current_left_joint_pose = self.get_joint_positions(self.left_arm_indices)
        current_left_end_effort_pose = self.get_relative_pose(self.left_base_link_name, self.left_end_effort_name)
        current_left_end_effort_pose = np.concatenate([current_left_end_effort_pose.p, current_left_end_effort_pose.q])
        
        left_target_gripper_pose = current_left_end_effort_pose + delta_move
        
        start_time = time.time()
        left_result = self.left_planner.ik(left_target_gripper_pose)
        end_time = time.time()
        print(f"单臂:{end_time - start_time}s")

        if not np.array(left_result.success.cpu())[0][0]:
            left_result = current_left_joint_pose
            print("left ik fail")
            return
        left_result = np.array(left_result.js_solution.position.cpu())[0][0]
        step_n = self.compute_steps(current_left_joint_pose, left_result)
        left_path = np.linspace(current_left_joint_pose, left_result, step_n)
        self.run_trajectory(self.left_arm_indices, left_path)
    
    def right_move(self,delta_move):
        current_right_joint_pose = self.get_joint_positions(self.right_arm_indices)
        current_right_end_effort_pose = self.get_relative_pose(self.right_base_link_name, self.right_end_effort_name)
        current_right_end_effort_pose = np.concatenate([current_right_end_effort_pose.p, current_right_end_effort_pose.q])

        right_target_gripper_pose = current_right_end_effort_pose + delta_move

        start_time = time.time()
        right_result = self.right_planner.ik(right_target_gripper_pose)
        end_time = time.time()
        print(f"单臂:{end_time - start_time}s")
        
        if not np.array(right_result.success.cpu())[0][0]:
            right_result = current_right_joint_pose
            print("right ik fail")
            return
        right_result = np.array(right_result.js_solution.position.cpu())[0][0]
        step_n = self.compute_steps(current_right_joint_pose, right_result)
        right_path = np.linspace(current_right_joint_pose, right_result, step_n)
        self.run_trajectory(self.right_arm_indices, right_path)
    
    def move(self,left_delta_move, right_delta_move):
        current_left_joint_pose = self.get_joint_positions(self.left_arm_indices)
        current_left_end_effort_pose = self.get_relative_pose(self.left_base_link_name, self.left_end_effort_name)
        current_left_end_effort_pose = np.concatenate([current_left_end_effort_pose.p, current_left_end_effort_pose.q])

        left_target_gripper_pose = current_left_end_effort_pose + left_delta_move

        current_right_joint_pose = self.get_joint_positions(self.right_arm_indices)
        current_right_end_effort_pose = self.get_relative_pose(self.right_base_link_name, self.right_end_effort_name)
        current_right_end_effort_pose = np.concatenate([current_right_end_effort_pose.p, current_right_end_effort_pose.q])

        right_target_gripper_pose = current_right_end_effort_pose + right_delta_move
        start_time = time.time()

        left_result = self.left_planner.ik(left_target_gripper_pose)
        right_result = self.right_planner.ik(right_target_gripper_pose)

        if not np.array(left_result.success.cpu())[0][0]:
            left_result = current_left_joint_pose
            print("left ik fail")
        else:
            left_result = np.array(left_result.js_solution.position.cpu())[0][0]
        
        if not np.array(right_result.success.cpu())[0][0]:
            right_result = current_right_joint_pose
            print("right ik fail")
        else:
            right_result = np.array(right_result.js_solution.position.cpu())[0][0]
        
        step_n = max(self.compute_steps(current_left_joint_pose, left_result), self.compute_steps(current_right_joint_pose, right_result))
        left_path = np.linspace(current_left_joint_pose, left_result, step_n)
        right_path = np.linspace(current_right_joint_pose, right_result, step_n)

        end_time = time.time()
        print(f"双臂推理: {end_time - start_time}s, 进行插值{step_n}步")

        full_path = np.hstack([left_path, right_path])

        # _full = np.hstack([left_result, right_result])
        self.run_trajectory((self.left_arm_indices + self.right_arm_indices) , full_path, steps_per_target=4)
    
    def compute_steps(self,q_start, q_target, min_steps=5, max_steps=50, threshold=np.pi/4):
        delta = np.linalg.norm(q_target - q_start)
        ratio = min(delta / threshold, 1.0) # 如果变化超过45度就达到差值上限
        return int(min_steps + (max_steps - min_steps) * ratio)

    def pad_to_same_length(self, traj1, traj2):
        len1, len2 = len(traj1), len(traj2)
        if len1 < len2:
            pad = np.repeat(traj1[-1][None, :], len2 - len1, axis=0)
            traj1 = np.vstack([traj1, pad])
        elif len2 < len1:
            pad = np.repeat(traj2[-1][None, :], len1 - len2, axis=0)
            traj2 = np.vstack([traj2, pad])
        return traj1, traj2

def print_instructions():
    """打印键盘控制说明"""
    print("\n" + "="*50)
    print("双臂运动控制系统 - 使用说明")
    print("="*50)
    print("位移控制:")
    print("  W/S: 前/后移动(X轴)")
    print("  A/D: 左/右移动(Y轴)")
    print("  Q/E: 上/下移动(Z轴)")
    print("\n模式选择:")
    print("  0: 双同时运动")
    print("  1: 仅左臂运动")
    print("  2: 仅右臂运动")
    print("\n其他:")
    print("  ESC: 退出程序")
    print("="*50 + "\n")

def on_release(key):
    if key == keyboard.Key.esc:
        return False  # 停止监听

def on_press(key):
    global x,y,z,move_type
    # x, y, z = 0.0, 0.0, 0.0
    # 位移控制
    delta_move =0.01
    if key.char == 'w': x += delta_move
    elif key.char == 's': x -= delta_move
    elif key.char == 'a': y += delta_move
    elif key.char == 'd': y -= delta_move
    elif key.char == 'q': z += delta_move
    elif key.char == 'e': z -= delta_move
    
    # 模式选择
    elif key.char == '0': move_type = 0
    elif key.char == '1': move_type = 1
    elif key.char == '2': move_type = 2

def test():
    controller = SimRobot(
        urdf_path="/home/niantian/projects/aloha_maniskill_sim/urdf/arx5_description_isaac.urdf",
        fix_root_link=True,
        balance_passive_force=True
    )
    left_planner = CuroboPlanner(active_joints_name=["fl_joint1","fl_joint2","fl_joint3","fl_joint4","fl_joint5","fl_joint6"],\
                                yml_path='/home/niantian/projects/aloha_maniskill_sim/curobo_left.yml')
    
    right_planner = CuroboPlanner(active_joints_name=["fr_joint1","fr_joint2","fr_joint3","fr_joint4","fr_joint5","fr_joint6"],\
                            yml_path='/home/niantian/projects/aloha_maniskill_sim/curobo_right.yml')

    controller.set_planner(left_planner, right_planner)

    # 获取图像
    rgb, depth = controller.take_picture()
    cv2.imwrite("rgb.jpg", rgb)
    cv2.imwrite("depth.jpg",depth)
    
    # 设置初始角度
    controller.set_arm_qpos(left_angles=[0.0]*6, right_angles=[0.0]*6)
    
    # 运动示例:
    # 单臂运动
    # left_move = np.array([0.05, 0.05, 0.1, 0, 0, 0, 0])
    # controller.left_move(left_move)
    # 双臂一同运动
    left_move = np.array([0, -0.05, 0.1, 0, 0, 0, 0])
    right_move = np.array([0.10, -0.05, 0.1, 0, 0, 0, 0])
    
    controller.move(left_move, right_move)
    
    
    '''
    0: 左臂右臂一同运动
    1: 左臂
    2:右臂
    '''
    print_instructions()

    global x,y,z,move_type

    listener = keyboard.Listener(on_press=on_press, on_release=on_release)
    listener.start()
    
    while listener.is_alive():
        while True:
            delata_move = np.array([x, y, z] + [0, 0, 0, 0])
            if move_type == 0:
                controller.move(delata_move,delata_move)
            elif move_type == 1:
                controller.left_move(delata_move)
            elif move_type == 2:
                controller.right_move(delata_move)
    
    # 进入控制循环（可注释掉）
    controller.loop()

if __name__ == '__main__':
    test()
