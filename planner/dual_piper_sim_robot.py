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

class SimSingleArm:
    def __init__(self, viewer, scene, urdf_path: str,pose=None, fix_root_link=True, balance_passive_force=True):
        # # 初始化引擎、场景、渲染器
        # engine = sapien.Engine()
        # renderer = sapien.SapienRenderer()
        # engine.set_renderer(renderer)

        # self.scene = engine.create_scene(sapien.SceneConfig())
        # self.scene.set_timestep(1 / 240.0)
        # self.scene.add_ground(0)
        # self.scene.set_ambient_light([0.5, 0.5, 0.5])
        # self.scene.add_directional_light([0, 1, -1], [0.5, 0.5, 0.5])

        # self.viewer = Viewer(renderer)
        # self.viewer.set_scene(self.scene)
        # self.viewer.set_camera_xyz(x=-2, y=0, z=1)
        # self.viewer.set_camera_rpy(r=0, p=-0.3, y=0)

        self.viewer = viewer
        self.scene = scene

        # 加载机器人
        loader = self.scene.create_urdf_loader()
        loader.fix_root_link = fix_root_link
        robot = loader.load(urdf_path)
        if pose is not None:
            robot.set_root_pose(pose)
        self.robot = robot

        self.balance_passive_force = balance_passive_force

        # 获取关节信息
        active_joints = self.robot.get_active_joints()
        active_joint_names = [joint.get_name() for joint in active_joints]

        # 设置左右臂关节名称, TODO: 修改为你的机械臂的对应映射
        arm_names = [f"joint{i}" for i in range(1, 7)]
        gripper_names = [f"joint{i}" for i in range(6, 8)]

        self.arm_indices = [i for i, name in enumerate(active_joint_names) if name in arm_names]

        self.base_link_name = "base_link"
        self.end_effort_name = "link6"

        self.camera_link_name = "camera"

        self.wrist_camera = self._setup_camera()

    def set_planner(self, planner):
        self.planner = planner

    def _get_link_by_name(self, name: str):
        for link in self.robot.get_links():
            if link.get_name() == name:
                return link
        raise ValueError(f"Link '{name}' not found")

    def _update_camera(self):
        wrist_pos_link = self._get_link_by_name(self.camera_link_name)
        wrist_pos = wrist_pos_link.get_pose()

        self.wrist_camera.set_pose(wrist_pos)

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
        self.wrist_camera.take_picture()

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

    def set_arm_qpos(self, angles=None):
        qpos = self.robot.get_qpos()
        if angles:
            for idx, angle in zip(self.arm_indices, angles):
                qpos[idx] = angle
            self.robot.set_qpos(qpos)

    def run_trajectory(self, joint_indices, trajectory, steps_per_target=1):
        for target_angles in trajectory:
            qpos = self.robot.get_qpos()
            for idx, angle in zip(joint_indices, target_angles):
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

    def move(self,delta_move):
        current_joint_pose = self.get_joint_positions(self.arm_indices)
        current_end_effort_pose = self.get_relative_pose(self.base_link_name, self.end_effort_name)
        current_end_effort_pose = np.concatenate([current_end_effort_pose.p, current_end_effort_pose.q])
        
        target_gripper_pose = current_end_effort_pose + delta_move
        
        start_time = time.time()
        result = self.planner.ik(target_gripper_pose)
        end_time = time.time()
        # print(f"单臂:{end_time - start_time}s")

        if not np.array(result.success.cpu())[0][0]:
            result = current_joint_pose
            print("left ik fail")
            return self.arm_indices, None
        result = np.array(result.js_solution.position.cpu())[0][0]
        step_n = self.compute_steps(current_joint_pose, result)
        path = np.linspace(current_joint_pose, result, step_n)
        return self.arm_indices, path
        # self.run_trajectory(self.arm_indices, path)
    
    def compute_steps(self,q_start, q_target, min_steps=5, max_steps=50, threshold=np.pi/4):
        delta = np.linalg.norm(q_target - q_start)
        ratio = min(delta / threshold, 1.0) # 如果变化超过45度就达到差值上限
        return int(min_steps + (max_steps - min_steps) * ratio)

class SimDualArm:
    def __init__(self):
        self.robots = None
    
    def setup(self, scene, viewer, robots):
        self.scene = scene
        self.viewer = viewer
        self.robots = robots
        self.balance_passive_force = True

    def play_once(self, target_poses, steps_per_target=1):
        indeces, paths, max_len = [], [], 0
        for robot, target_pose in zip(self.robots, target_poses):
            arm_indices, path = robot.move(target_pose)
            indeces.append(arm_indices)
            paths.append(path)
            if path is not None:
                max_len = len(path)  if len(path)>max_len else max_len
        
        for step in range(max_len):
            for robot_index in range(len(self.robots)):
                if paths[robot_index] is not None and len(paths[robot_index]) > step:

                    qpos = self.robots[robot_index].robot.get_qpos()
                    for idx, angle in zip(indeces[robot_index], paths[robot_index][step]):
                        qpos[idx] = angle
                    self.robots[robot_index].robot.set_qpos(qpos)

            for _ in range(steps_per_target):
                if self.balance_passive_force:
                    qf = self.robots[robot_index].robot.compute_passive_force(gravity=True, coriolis_and_centrifugal=True)
                    self.robots[robot_index].robot.set_qf(qf)
                self.scene.step()

            self.scene.update_render()
            self.viewer.render()

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
    engine = sapien.Engine()
    renderer = sapien.SapienRenderer()
    engine.set_renderer(renderer)

    scene = engine.create_scene(sapien.SceneConfig())
    scene.set_timestep(1 / 240.0)
    scene.add_ground(0)
    scene.set_ambient_light([0.5, 0.5, 0.5])
    scene.add_directional_light([0, 1, -1], [0.5, 0.5, 0.5])

    viewer = Viewer(renderer)
    viewer.set_scene(scene)
    viewer.set_camera_xyz(x=-2, y=0, z=1)
    viewer.set_camera_rpy(r=0, p=-0.3, y=0)
    left_controller = SimSingleArm(
        viewer,
        scene,
        urdf_path="/home/niantian/projects/piper/embodiments/piper/piper.urdf",
        pose=sapien.Pose([0,0.2,0],[1,0,0,0]),
        fix_root_link=True,
        balance_passive_force=True
    )
    left_planner = CuroboPlanner(active_joints_name=["joint1","joint2","joint3","joint4","joint5","joint6"],\
                                yml_path="/home/niantian/projects/piper/embodiments/piper/curobo_tmp.yml")

    left_controller.set_planner(left_planner)

    right_controller = SimSingleArm(
        viewer,
        scene,
        urdf_path="/home/niantian/projects/piper/embodiments/piper/piper.urdf",
        pose=sapien.Pose([0,-0.2,0],[1,0,0,0]),
        fix_root_link=True,
        balance_passive_force=True
    )
    right_planner = CuroboPlanner(active_joints_name=["joint1","joint2","joint3","joint4","joint5","joint6"],\
                                yml_path="/home/niantian/projects/piper/embodiments/piper/curobo_tmp.yml")

    right_controller.set_planner(right_planner)

    right_controller.set_arm_qpos(angles=[0.0]*6)
    left_controller.set_arm_qpos(angles=[0.0]*6)

    robot = SimDualArm()
    robot.setup(scene, viewer, [left_controller, right_controller])

    # # 获取图像
    # rgb, depth = controller.take_picture()
    # cv2.imwrite("rgb.jpg", rgb)
    # cv2.imwrite("depth.jpg",depth)
    
    # 设置初始角度
    
    
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
            print("delata_move:", delata_move)
            robot.play_once([delata_move, delata_move])
            # if move_type == 0:
            #     controller.move(delata_move)
            # elif move_type == 1:
            #     controller.left_move(delata_move)
            # elif move_type == 2:
            #     controller.right_move(delata_move)
    
    # 进入控制循环（可注释掉）
    # controller.loop()

if __name__ == '__main__':
    test()
