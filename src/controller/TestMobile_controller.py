from controller.mobile_controller import MobileController

import numpy as np
import time

from utils.data_handler import debug_print

class TestMobileController(MobileController):
    def __init__(self, name, INFO="DEBUG"):
        super().__init__()
        self.name = name
        self.INFO = INFO

    def set_up(self):
        self.position =  np.random.rand(6)
        self.position[2] = 0.
        self.position[3] = 0.
        self.position[4] = 0.

        self.velocity = np.random.rand(6)
        self.velocity[2] = 0.
        self.velocity[3] = 0.
        self.velocity[4] = 0.
        debug_print(self.name, f"setup success",self.INFO)
    
    def set_move_velocity(self, velocity):
        self.velocity = velocity
        self.position += self.velocity * 0.1
    
    def set_move_to(self, position):
        self.position = position
    
    def get_position(self):
        return self.position

    def get_move_velocity(self):
        return self.velocity

    def __del__(self):
        try:
            if hasattr(self, 'controller'):
                # Add any necessary cleanup for the arm controller
                pass
        except:
            pass

if __name__ == "__main__":
    import os
    os.environ["INFO_LEVEL"] = "DEBUG" # DEBUG , INFO, ERROR

    controller = TestMobileController("test_mobile")
    controller.set_up()
    controller.set_collect_info(["move_velocity", "position"])

    for i in range(10):
        time.sleep(0.1)
        controller.move({"move_velocity": [0.01, 0.01, 0., 0., 0., 0.]})

        print(controller.get())
    
    controller.move({"move_to": [5., 5., 0., 0., 0., 1.]})
    print(controller.get())