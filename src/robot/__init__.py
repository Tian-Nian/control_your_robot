import os
current_file = os.path.abspath(__file__)
current_dir = os.path.dirname(current_file)

ROOT_PATH = os.path.join(current_dir, "../../")
ROBOT_PATH = os.path.join(ROOT_PATH, "my_robot")