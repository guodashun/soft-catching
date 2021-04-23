import numpy as np
from scipy.spatial.transform import Rotation as R

def quat2rotvec(quat):
    r = R.from_quat(quat)
    return r.as_rotvec()

def quat2matrix(quat):
    r = R.from_quat(quat)
    return r.as_matrix()

def rotvec2quat(vec):
    r = R.from_rotvec(vec)
    return r.as_quat()

# def quat2euler(quat):
#     r = R.from_quat(quat)
#     return r.as_euler('zyx')

# def pose_absolute_control(tar, cur_pos, cur_vel, kp, kd):
#     tar = np.array(tar)
#     cur_pos = np.array(cur_pos)
#     cur_vel = np.array(cur_vel)
#     action = np.zeros((6,1))
    
#     for i in range(len(tar)):
#         action[i] = (tar[i] - cur_pos[i]) * kp - cur_vel[i] * kd

#     return action

def impedence_control(force, x_p, v, M, K, D, freq):
    acc = np.linalg.inv(M) @ ( K @ x_p - force - D @ v)
    next_v = v + acc / freq
    x = (v + next_v) / 2 / freq
    return x

def vel_follow():
    return 
