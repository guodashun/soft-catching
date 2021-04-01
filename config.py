import numpy as np
from scipy.spatial.transform import Rotation as R

RT_wb = np.eye(4)
RT_wb[:3, :3] = R.from_euler('zyz', [45,45, 90], degrees=True).as_matrix()
RT_wb[:3, 3] = [0, 0.225, 0.4]

RT_104 = RT_wb.copy()
config_104 = {
    'RT_wb':RT_104,
    'arm_ip':"192.168.1.104",
    'force_port': 63351,
}

RT_wb[:3, 3] = [0, 0,0]
RT_104_v = RT_wb.copy()

config_104_v = {
    'RT_wb':RT_104_v,
    'arm_ip':"192.168.1.104",
    'force_port': 63351,
}

config = config_104_v
