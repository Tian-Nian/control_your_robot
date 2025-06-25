import sys
sys.path.append("./")

from Robotic_Arm.rm_robot_interface import *
import time
import numpy as np
from controller.arm_controller import ArmController

'''
RealMan base code from:
https://develop.realman-robotics.com/robot/apipython/getStarted/
'''

class RealmanController(ArmController):
    def __init__(self, name):
        super().__init__()
        self.name = name
        self.controller_type = "user_controller"
        self.controller = None

    def set_up(self,rm_ip, thread_mode=None,connection_level=3):
        try:
            result = initialize_robot(rm_ip, thread_mode,connection_level)
            if result is None:
                raise ConnectionError(f"Failed to connect to robot arm at {rm_ip}")
            self.controller, self.handle = result
        except Exception as e:
            raise ConnectionError(f"Failed to initialize robot arm: {str(e)}")
        self.prev_tech_state = None
        self.gripper_close = False

    def reset(self, start_angles):
        """Move robot to the specified start position"""
        print(f"\nMoving {self.name} arm to start position...")
        
        # Check if robot is still connected
        succ, _ = self.controller.rm_get_current_arm_state()
        if succ != 0:
            print(f"Error: {self.name} arm is not connected or responding")
            return False
        
        # Get current joint positions
        succ, state = self.controller.rm_get_current_arm_state()
        if succ == 0:
            current_joints = state['joint']
            print(f"Current {self.name} arm position: {current_joints}")
        
        # release gripper
        self.controller.rm_set_gripper_release(100, 0, 0)

        # Move to start position with error handling
        try:
            print(f"Target {self.name} arm position: {start_angles}")
            result = self.controller.rm_movej(start_angles, 20, 0, 0, 1)  # v=20%, blocking=True
            if result == 0:
                print(f"Successfully moved {self.name} arm to start position")
                # Verify current position
                succ, state = self.controller.rm_get_current_arm_state()
                if succ == 0:
                    current_joints = state['joint']
                    print(f"New {self.name} arm position: {current_joints}")
                    max_diff = max(abs(np.array(current_joints) - np.array(start_angles)))
                    if max_diff > 0.01:  # Allow small tolerance of 0.01 radians
                        print(f"Warning: {self.name} arm position differs from target by {max_diff} radians")
                else:
                    print(f"Warning: Could not verify {self.name} arm position")
                # Wait for system to stabilize
                print(f"Waiting for {self.name} arm to stabilize...")
                time.sleep(2)
                return True
            else:
                print(f"Failed to move {self.name} arm to start position. Error code: {result}")
                return False
        except Exception as e:
            print(f"Exception while moving {self.name} arm: {str(e)}")
            return False

    def get_state(self):
        # Get arm state
        succ, arm_state = self.controller.rm_get_current_arm_state()

        if succ != 0 or arm_state is None:
            raise RuntimeError("Failed to get arm state")
        state = arm_state.copy()

        state["gripper"] = self.get_gripper()

        return state
    
    # The input gripper value is in the range [0, 1], representing the degree of opening.
    def get_gripper(self):
        try:
            succ, gripper = self.controller.rm_get_gripper_state()
            if succ != 0:
                raise RuntimeError("Failed to get gripper state")
            return gripper
        except Exception as e:
            raise RuntimeError(f"Error getting gripper state: {str(e)}")
        return gripper
    
    '''
    Unlike Libero and Driod, which rely on separate action spaces, 
    RealMan is controlled directly through state-based commands. 
    Therefore, this function is unnecessary.
    '''
    def get_action(self):
        raise NotImplementedError("get_action is not implemented")
    
    def set_joint(self, joint):
        try:
            if len(joint) != 7:
                raise ValueError(f"Invalid joint length: {len(joint)}")
            success = self.controller.rm_movej_canfd(joint, False, 0, 0, 0)
            if success != 0:
                raise RuntimeError("Failed to set joint angles")
        except Exception as e:
            raise RuntimeError(f"Error moving robot: {str(e)}")

    def set_position(self, position):
        try:
            # Validate state length
            if len(position) != 6:  # 6 joints
                raise ValueError(f"Invalid state length: {len(position)}")
            # delta postion, abs angle
            success = self.controller.rm_movej_p_canfd(position, False, 0, 0, 0)
            if success != 0:
                raise RuntimeError("Failed to set joint angles")           
        except Exception as e:
            raise RuntimeError(f"Error moving robot: {str(e)}")
     
    def set_gripper(self, gripper):
        try:
            if gripper < 0.10 and not self.gripper_close:
                success = self.controller.rm_set_gripper_pick(100, 100, 0, 0)
                if success:
                    self.gripper_close = True
                else:
                    raise RuntimeError("Failed to close gripper")
            elif gripper > 0.9 and self.gripper_close:
                success = self.controller.rm_set_gripper_release(100, 0, 0)
                if success:
                    self.gripper_close = False
                else:
                    raise RuntimeError("Failed to open gripper")
        except Exception as e:
            raise RuntimeError(f"Error setting gripper pos: {str(e)}")

    def set_action(self, action):
        raise NotImplementedError("set_action is not implemented")
    
    def __del__(self):
        try:
            if hasattr(self, 'controller'):
                # Add any necessary cleanup for the arm controller
                pass
        except:
            pass

def initialize_robot(robot_ip, thread_mode=None, connection_level=3):
    """Initialize robot arm controller and establish connection"""
    print(f"\nInitializing robot at {robot_ip} with connection level {connection_level}...")
    
    # Create a new instance with the specified thread mode
    if thread_mode is not None:
        print(f"Using thread mode: {thread_mode}")
        robot_controller = RoboticArm(thread_mode)
    else:
        print("Using default thread mode")
        # Default to single thread mode if none specified
        robot_controller = RoboticArm()
    
    # Try to connect with retry logic
    max_retries = 3
    for attempt in range(max_retries):
        print(f"Connecting to robot at {robot_ip}, attempt {attempt+1}/{max_retries}...")
        handle = robot_controller.rm_create_robot_arm(robot_ip, 8080, connection_level)
        
        if handle.id != -1:
            print(f"Successfully connected to robot at {robot_ip}, handle ID: {handle.id}")
            # Verify connection is active
            succ, state = robot_controller.rm_get_current_arm_state()
            if succ == 0:
                print(f"Connection verified for robot at {robot_ip}")
                print(f"Current state: {state}")
                
                # Get robot info for additional verification
                succ, info = robot_controller.rm_get_robot_info()
                if succ == 0:
                    print(f"Robot info: {info}")
                
                return robot_controller, handle
            else:
                print(f"Connection established but couldn't get state from robot at {robot_ip}. Error code: {succ}")
        else:
            print(f"Failed to create robot arm handle for {robot_ip}. Handle ID: {handle.id}")
        
        if attempt < max_retries - 1:
            print(f"Failed to connect to robot at {robot_ip}, retrying in 2 seconds...")
            time.sleep(2)
    
    print(f"Failed to connect to robot at {robot_ip} after {max_retries} attempts")
    return None

if __name__ == "__main__":
    controller = RealmanController("Realman")
    controller.set_up("192.168.1.18", rm_thread_mode_e.RM_TRIPLE_MODE_E)
    controller.reset([0, 0, 0, 0, 0, 0, 0])
    time.sleep(1)
    controller.set_gripper(0.5)
    time.sleep(1)