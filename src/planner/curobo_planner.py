from curobo.types.math import Pose as CuroboPose
import time
import torch
import yaml
from curobo.types.robot import JointState
from curobo.wrap.reacher.motion_gen import MotionGen, MotionGenConfig, MotionGenPlanConfig, PoseCostMetric
# cuRobo
from curobo.types.base import TensorDeviceType
from curobo.types.robot import RobotConfig
from curobo.util_file import get_robot_configs_path, join_path, load_yaml
from curobo.wrap.reacher.ik_solver import IKSolver, IKSolverConfig

import numpy as np
import pdb
import toppra as ta
import transforms3d as t3d

from curobo.util import logger

# import envs._GLOBAL_CONFIGS as CONFIGS

class CuroboPlanner():
    def __init__(self, active_joints_name, yml_path=None):
        super().__init__()
        ta.setup_logging("CRITICAL") # hide logging
        logger.setup_logger(level="error", logger_name="'curobo")
        
        if yml_path != None:    
            self.yml_path = yml_path
        else:
            raise ValueError("[Planner.py]: CuroboPlanner yml_path is None!")
        # self.robot_origion_pose = robot_origion_pose
        self.active_joints_name = active_joints_name
        # self.all_joints = [joint.get_name() for joint in entity.get_active_joints()]
        
        # translate from baselink to arm's base
        with open(self.yml_path, 'r') as f:
            yml_data = yaml.safe_load(f)
        self.frame_bias = yml_data['planner']['frame_bias']
        self.urdf_path = yml_data['robot_cfg']['kinematics']['urdf_path']
        self.base_link = yml_data['robot_cfg']['kinematics']['base_link']
        self.ee_link = yml_data['robot_cfg']['kinematics']['ee_link']
        self.tensor_args = TensorDeviceType()
            
        robot_cfg = RobotConfig.from_basic(self.urdf_path, self.base_link, self.ee_link, self.tensor_args)
        ik_config = IKSolverConfig.load_from_robot_config(
                                            robot_cfg,
                                            None,
                                            num_seeds=20,
                                            self_collision_check=True,
                                            self_collision_opt=True,
                                            tensor_args=self.tensor_args,
                                            use_cuda_graph=True,
                                        )
        self.ik_solver = IKSolver(ik_config)
        # motion generation
        if True: # if not aloha
            world_config = {
                "cuboid": {
                    "table": {
                        "dims": [0.7, 2, 0.04],  # x, y, z
                        "pose": [1000, 0.0, 0.74, 1, 0, 0, 0.0],  # x, y, z, qw, qx, qy, qz
                    },
                }
            }
        
        motion_gen_config = MotionGenConfig.load_from_robot_config(
            self.yml_path,
            # world_config,
            interpolation_dt=1/250,
            num_trajopt_seeds=1
        )
        self.motion_gen = MotionGen(motion_gen_config)
        self.motion_gen.warmup()
        self.motion_gen_batch = MotionGen(motion_gen_config)
        # self.motion_gen_batch.warmup(batch=10) # batch=CONFIGS.ROTATE_NUM = 10
        
        
    def ik(self, target_gripper_pose, current_joint_angle=None):
        # 构造目标位姿，假设target_gripper_pose格式为[x,y,z, ..., qx,qy,qz,qw]
        goal_pose_of_gripper = CuroboPose.from_list(
            list(target_gripper_pose[:3]) + list(target_gripper_pose[-4:])
        )
        
        if current_joint_angle is not None:
            # current_joint_angle 应该是 shape (dof,)，添加 batch 和 seq 维度 (1,1,dof)
            retract_config = torch.tensor(current_joint_angle, device="cuda",dtype=torch.float32).unsqueeze(0)  # (1, dof)
            seed_config = retract_config.unsqueeze(0)  # (1, 1, dof)

            result = self.ik_solver.solve_goalset(
                goal_pose_of_gripper,
                retract_config=retract_config,
                seed_config=seed_config
            )
        else:
            result = self.ik_solver.solve_single(goal_pose_of_gripper)
        return result

    # target_gripper_pose np array [:7] x y z qw qx qy qz
    def plan_path(self, curr_joint_pos, target_gripper_pose, constraint_pose=None, arms_tag=None):
        goal_pose_of_gripper = CuroboPose.from_list(list(target_gripper_pose[:3]) + list(target_gripper_pose[-4:]))

        # print('[debug]: joint_angles: ', joint_angles)
        start_joint_states = JointState.from_position(
                                                torch.tensor(curr_joint_pos).float().cuda().reshape(1, -1),
                                                joint_names=self.active_joints_name)
        # plan
        c_start_time = time.time()
        plan_config = MotionGenPlanConfig(max_attempts=10)
        if constraint_pose is not None:
            pose_cost_metric = PoseCostMetric(
                hold_partial_pose=True,
                hold_vec_weight=self.motion_gen.tensor_args.to_device(constraint_pose)
            )
            plan_config.pose_cost_metric = pose_cost_metric
        
        self.motion_gen.reset(reset_seed=True)  # 运行的代码

        result = self.motion_gen.plan_single(
            start_joint_states, 
            goal_pose_of_gripper, 
            plan_config
        )
        # traj = result.get_interpolated_plan()
        c_time = time.time() - c_start_time
        # print(f"[Planner.py] Func(plan_curobo): Planning time: {c_time:.3f}s")
        
        # output
        res_result = dict()
        if result.success.item() == False:
            print(f"[Planner.py] Func(plan_curobo): {arms_tag} plan failed, status: \033[31m{result.status}\033[0m")
            # print("[Planner.py] target_pose: ", target_pose_p, target_pose_q)
            res_result['status'] = "Fail"
            return res_result
        else:
            res_result['status'] = "Success"
            res_result['position'] = np.array(result.interpolated_plan.position.to('cpu'))
            res_result['velocity'] = np.array(result.interpolated_plan.velocity.to('cpu'))
            return res_result
    
    def plan_grippers(self, now_val, target_val):
        step_n = 200
        dis_val = target_val - now_val
        step = dis_val / step_n
        res={}
        vals = np.linspace(now_val, target_val, step_n)
        res['step_n'] = step_n
        res['step'] = step
        res['result'] = vals
        return res
    
    def _trans_from_world_to_base(self, base_pose, target_pose):
        # transform target pose from world frame to base frame
        # base_pose: np.array([x, y, z, qw, qx, qy, qz])
        # target_pose: np.array([x, y, z, qw, qx, qy, qz])
        base_p, base_q = base_pose[0:3], base_pose[3:]
        target_p, target_q = target_pose[0:3], target_pose[3:]
        rel_p = target_p - base_p
        wRb= t3d.quaternions.quat2mat(base_q)
        wRt = t3d.quaternions.quat2mat(target_q)
        result_p = wRb.T @ rel_p
        result_q = t3d.quaternions.mat2quat(wRb.T @ wRt)
        return result_p, result_q

if __name__ == "__main__":
    
    planner = CuroboPlanner(active_joints_name=["fl_joint1","fl_joint2","fl_joint3","fl_joint4","fl_joint5","fl_joint6"], yml_path='/home/niantian/projects/aloha_maniskill_sim/curobo_left.yml')
    curr_joint_pos = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    target_gripper_pose = np.array([0.0506, -0.0018,  0.3029, 0.7600,  0.4181, -0.4921,  0.0737])
    import time
    for i in range(10):
        start  = time.time()
        result = planner.ik(target_gripper_pose, current_joint_angle=curr_joint_pos)
        # result2 = planner.ik(target_gripper_pose)
        end = time.time()
        # import pdb;pdb.set_trace()
        print(result)
        # print(result2)
        print(f"time cost:{end - start}s")
    # print(result)