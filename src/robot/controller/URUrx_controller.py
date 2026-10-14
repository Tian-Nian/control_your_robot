

from robot.utils.base.data_handler import debug_print
from robot.controller.arm_controller import ArmController

import urx
"""
UR base code from:
https://github.com/SintefManufacturing/python-urx?utm_source=chatgpt.com
"""

class URUrxController(ArmController):
    def __init__(self, name):
        self.name = name
        self.controller_type = "user_controller"
        self.controller = None
    
    def set_up(self):
        self.controller = urx.Robot("192.168.0.100")
        self.controller .set_tcp((0, 0, 0.1, 0, 0, 0))
        self.controller .set_payload(2, (0, 0, 0.1))

    def apply_reset(self, start_state):
        joints = [0, 0, 0, 0, 0, 0] if start_state is None else list(start_state)
        self.controller.movej(joints)
