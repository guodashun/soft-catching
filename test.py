import math
import time
import numpy as np
import robosuite as suite
import matplotlib.pyplot as plt
from robosuite.controllers.controller_factory import load_controller_config, controller_factory
from robosuite.models.grippers.gripper_model import GripperModel
from scipy.spatial.transform import rotation
from fly_env import FlyEnv
from robosuite.models.grippers import GRIPPER_MAPPING
from robosuite.wrappers import GymWrapper
from grippers.plate_gripper import PlateGripper
from utils import *

# add custom gripper
GRIPPER_MAPPING["PlateGripper"] = PlateGripper

# show force data
show_data = False

# params for impedence control
control_freq = 500
# plate_offset = [0,0,10]
# plate_force_arm = [-0.045, -0.045, 0]
M = np.array(
    [1, 0, 0, 0, 0, 0,
     0, 1, 0, 0, 0, 0,
     0, 0, 1, 0, 0, 0,
     0, 0, 0, 1, 0, 0,
     0, 0, 0, 0, 1, 0,
     0, 0, 0, 0, 0, 1,]
).reshape(6,6) * 0.3
D = np.array(
    [1, 0, 0, 0, 0, 0,
     0, 1, 0, 0, 0, 0,
     0, 0, 1, 0, 0, 0,
     0, 0, 0, 1, 0, 0,
     0, 0, 0, 0, 1, 0,
     0, 0, 0, 0, 0, 1,]
).reshape(6,6) * 10
K = np.array(
    [1, 0, 0, 0, 0, 0,
     0, 1, 0, 0, 0, 0,
     0, 0, 1, 0, 0, 0,
     0, 0, 0, 1, 0, 0,
     0, 0, 0, 0, 1, 0,
     0, 0, 0, 0, 0, 1,]
).reshape(6,6) * 0

controller_config = load_controller_config(custom_fpath='./pose_control.json')
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
        gripper_types="PlateGripper", # PlateGripper Robotiq140Gripper
        controller_configs=controller_config,
        use_object_obs=True,
        use_camera_obs=False,
        has_renderer=True,
        render_camera=None,
        has_offscreen_renderer=False,
        control_freq=control_freq,
        horizon=10000,
    )
obs = env.reset()
ur5e = env.robots[0]
tar = list(obs['robot0_eef_pos']) + list(quat2rotvec(obs['robot0_eef_quat']))
tar = np.array(tar)
sensor_data_list = []

if show_data:
    plt.ion()
    
force = np.zeros((6,))
last_force = np.zeros((6,))
force_cal = force.copy()
has_legend = False
while True:
    env.render()
    # print("render time", time.time() - t)
    # action = env.action_space.sample()
    ee_endpos = obs['robot0_eef_pos']
    ee_rotvec = quat2rotvec(obs['robot0_eef_quat'])
    # action =[0, 0,0] +list(robot_rotvec)# [0, 0,0,0,0,0]
    con_set = env.get_contacts(env.fly_obj)
    action = list(ee_endpos) + list(ee_rotvec)
    force = np.array(list(ur5e.ee_force) + list(ur5e.ee_torque))
    # print("cur plate pose", action)
    if not con_set:
        # vel_follow()
        pass
    else:
        # print("start impedence control")
        x_p = tar - np.array(action)
        print("before x_p", x_p)
        force = np.array(list(ur5e.ee_force) + list(ur5e.ee_torque))
        print("force", force)
        print("last_force", last_force)
        force_cal = force.copy()
        if np.linalg.norm(force-last_force) < 0.1:
            force_cal = np.zeros((6,))
        # force_offset = list(quat2matrix(obs['robot0_eef_quat']) @ plate_offset)
        # print("force offset", force_offset)
        # torque_offset = [force_offset[i] * plate_force_arm[i] for i in range(3)]
        # force -= np.array(force_offset + torque_offset)
        v = ur5e._hand_total_velocity
        x_p = impedence_control(force_cal, x_p, v, M, K, D, freq=control_freq)
        print("after  x_p", x_p)
        # action[0] += 0.01
        # action[1] += 0.01
        action += x_p
        # print("after control action is", action)
    
    ur5e._visualize_grippers(True)
    # print("contact", env.get_contacts(env.banana))
    # jacp = ur5e.sim.data.get_body_jacp(ur5e.robot_model.eef_name)
    # print("jacobian:", jacp)
    # print("jacobian shape", jacp.shape)
    # print(env.banana._contact_geoms)

    # action = list(ee_endpos) + list(ee_rotvec)
    # obs, _,_,_ = env.step(tar)
    obs, _,_,_ = env.step(action)


    controller = ur5e.controller
    
    last_force = force.copy()
    

    sensor_data_list.append(force)
    if show_data:
        t = range(len(sensor_data_list))
        plt.plot(t, np.array(sensor_data_list)[:,0], label="x", color='darkorange')
        plt.plot(t, np.array(sensor_data_list)[:,1], label="y", color='limegreen')
        plt.plot(t, np.array(sensor_data_list)[:,2], label="z", color='steelblue')
        plt.pause(1/control_freq)
        if not has_legend:
            plt.legend()
            has_legend = True
    # print(obs)
    # print("goal vel", controller.goal_vel, controller.last_err)
    # print("first joint vel: ",obs['robot0_joint_vel'][0])
    # print("robot eef quat:",obs['robot0_eef_quat'], robot_rotvec)
    # print("sensor data:", sensor_data)
    if len(sensor_data_list) % 100 == 0:
        np.save("force_data", sensor_data_list)
        print("save successfully")
