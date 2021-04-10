import robosuite as suite
from robosuite.models import MujocoWorldBase
from robosuite.models.robots import UR5e
from robosuite.models.grippers import gripper_factory
from robosuite.models.arenas import EmptyArena
from mujoco_py import MjSim, MjViewer

world = MujocoWorldBase()

ur5 = UR5e()
gripper = gripper_factory("RobotiqThreeFingerDexterousGripper")
ur5.add_gripper(gripper)

ur5.set_base_xpos([0,0,0])
world.merge(ur5)

arena = EmptyArena()
world.merge(arena)

model = world.get_model()
sim = MjSim(model)
viewer = MjViewer(sim)
while True:
    viewer.render()
