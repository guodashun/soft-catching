from sys import modules
import numpy as np
import robosuite as suite
from robosuite.environments.manipulation.single_arm_env import SingleArmEnv
from robosuite.models import arenas
from robosuite.models.tasks import ManipulationTask
from robosuite.models.arenas import EmptyArena
from robosuite.models.objects import BallObject
from robosuite.utils.observables import Observable, sensor
from objects.banana import BananaObject
from robosuite.utils.placement_samplers import UniformRandomSampler

class FlyEnv(SingleArmEnv):
    def __init__(
        self,
        robots,
        env_configuration="default",
        controller_configs=None,
        mount_types="default",
        gripper_types="default",
        initialization_noise="default",
        reward_scale=1.0,
        use_object_obs=False,
        use_camera_obs=True,
        has_renderer=False,
        has_offscreen_renderer=True,
        render_camera=None,
        render_collision_mesh=False,
        render_visual_mesh=True,
        render_gpu_device_id=-1,
        control_freq=20,
        horizon=1000,
        ignore_done=False,
        hard_reset=True,
        camera_names="agentview",
        camera_heights=256,
        camera_widths=256,
        camera_depths=False,
    ):
        
        self.reward_scale = reward_scale
        self.use_object_obs = use_object_obs
        

        super().__init__(robots, env_configuration=env_configuration, controller_configs=controller_configs, mount_types=mount_types, gripper_types=gripper_types, initialization_noise=initialization_noise, use_camera_obs=use_camera_obs, has_renderer=has_renderer, has_offscreen_renderer=has_offscreen_renderer, render_camera=render_camera, render_collision_mesh=render_collision_mesh, render_visual_mesh=render_visual_mesh, render_gpu_device_id=render_gpu_device_id, control_freq=control_freq, horizon=horizon, ignore_done=ignore_done, hard_reset=hard_reset, camera_names=camera_names, camera_heights=camera_heights, camera_widths=camera_widths, camera_depths=camera_depths)
    
    def _load_model(self):

        super()._load_model()

        # there may be a fly_obj

        arena = EmptyArena()

        self.fly_obj = BananaObject(
            name="fly_obj",
            # size=[0.04],
            # rgba=[0, 0.5, 0.5, 1]
        )

        self.model = ManipulationTask(
            mujoco_arena=arena,
            mujoco_robots=[robot.robot_model for robot in self.robots],
            mujoco_objects=self.fly_obj,
        )

    def _setup_observables(self):
        observables = super()._setup_observables()

        if self.use_object_obs:
            pf = self.robots[0].robot_model.naming_prefix
            modality = "object"

            @sensor(modality=modality)
            def fly_obj_pos(obs_cache):
                return np.array(self.sim.data.body_xpos[self.fly_obj_body_id])

            @sensor(modality=modality)
            def fly_obj_qpos(obs_cache):
                return np.array(self.sim.data.body_xquat[self.fly_obj_body_id])
            
            @sensor(modality=modality)
            def fly_obj_to_eef_pos(obs_cache):
                return obs_cache["fly_obj_pos"] - obs_cache[f"{pf}eef_pos"] if\
                    "fly_obj_pos" in obs_cache and f"{pf}eef_pos" in obs_cache else np.zeros(3)

            sensors = [fly_obj_pos, fly_obj_qpos, fly_obj_to_eef_pos]
            names = [s.__name__ for s in sensors]

            for name, s in zip(names, sensors):
                observables[name] = Observable(
                    name=name,
                    sensor=s,
                    sampling_rate=self.control_freq
                )

        return observables

    def _reset_internal(self):

        super()._reset_internal()
        
        # reset ur5 joint
        init_qpos = np.array([-0.470, -1.735, 2.480, -2.275, 1.57, 0])
        self.sim.data.qpos[self.robots[0]._ref_joint_pos_indexes] = init_qpos

        # Reset all object positions using initializer sampler if we're not directly loading from an xml
        
        self.fly_obj_body_id = self.sim.model.body_name2id(self.fly_obj.root_body)
        eef_pos_xy = self.sim.data.site_xpos[self.robots[0].eef_site_id][:2]
        self.sim.data.set_joint_qpos(self.fly_obj.joints[0], [eef_pos_xy[0], eef_pos_xy[1]+0.1,2,0,0,0,1])

    def reward(self, action):
        return 1.0
        # super().reward()

    def step(self, action):
        # con_set = self.get_contacts(self.fly_obj)
        
        # if not con_set:
        #     # vel follow
        #     pass
        # else:
        #     impedence_control()
        # # add fly object traj
        return super().step(action)

    def reset(self):

        return super().reset()
        # add fly object traj
