import robosuite as suite
from robosuite.controllers.controller_factory import load_controller_config, controller_factory
from fly_env import FlyEnv
from robosuite.wrappers import GymWrapper
import math
import time
import numpy as np

controller_config = load_controller_config(custom_fpath='./vel_control.json')
# user_controller_config = controller_factory(
#     "JOINT_VELOCITY",
#     input
# )

# env = GymWrapper(
#     suite.make(
#         "FlyEnv",
#         robots=['UR5e'],
#         gripper_types='default',
#         controller_configs=controller_config,
#         use_camera_obs=False,
#         has_renderer=True,
#         render_camera=None,
#         has_offscreen_renderer=False,
#         control_freq=500,
#         horizon=10000,
#     )
# )
env  =    suite.make(
        "FlyEnv",
        robots=['UR5e'],
        gripper_types=None,
        controller_configs=controller_config,
        use_camera_obs=False,
        has_renderer=True,
        render_camera=None,
        has_offscreen_renderer=False,
        control_freq=100,
        horizon=10000,
    )

obs = env.reset()
sensor_data_list = []
while True:
    t = time.time()
    env.render()
    # print("render time", time.time() - t)
    # action = env.action_space.sample()
    action = [0, 0,0,0,0,0]
    obs, _,_,_ = env.step(action)
    controller = env.robots[0].controller
    sensor_data = env.robots[0].get_sensor_measurement("gripper0_force_ee")
    # print("goal vel", controller.goal_vel, controller.last_err)
    # print("first joint vel: ",obs['robot0_joint_vel'][0])
    # print("sensor data:", sensor_data)
    # sensor_data_list.append(sensor_data)
    # if len(sensor_data_list) > 350:
    #     np.save("force_data", sensor_data_list)