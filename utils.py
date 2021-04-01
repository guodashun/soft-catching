import numpy as np
from scipy.spatial.transform import Rotation

def RT_(R,t):
    RT = np.zeros((4,4))
    RT[:3,:3]=R
    RT[:3,3]=t.flatten()
    RT[3,3]=1
    return RT

# def RT_I_(RT):
#     R,t = R_T_(RT)
#     R = np.linalg.inv(R)
#     t = -R@t
#     return RT_(R,t)

def R_T_(RT):
    R = RT[:3,:3]
    t = RT[:3,3].reshape(3,1)
    return R,t

def t2f(t):
    return np.array(t.flatten().tolist()+[1]).reshape(4,1)

def np2ur(pose):
    rotv = Rotation.from_dcm(pose[:3, :3]).as_rotvec()
    return np.r_[pose[:3, 3], rotv]

def ur2np(ur_pose):
    dcm = Rotation.from_rotvec(ur_pose[-3:]).as_matrix()
    m = np.eye(4)
    m[:3, :3] = dcm
    m[:3, 3] = ur_pose[:3]
    return m
