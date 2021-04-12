import os
import numpy as np
from robosuite.models.grippers.gripper_model import GripperModel

class PlateGripper(GripperModel):

    def __init__(self, idn=0):
        cur_dir = os.path.dirname(os.path.realpath(__file__))
        super().__init__(cur_dir + "/../assets/plate.xml", idn=idn)

    def format_action(self, action):
        return action

    @property
    def init_qpos(self):
        return np.array([0.0])

    @property
    def _important_geoms(self):
        return {
            "center": ["plate_collision"]
        }

