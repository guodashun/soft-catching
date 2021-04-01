import numpy as np
import rtde_control
import rtde_receive
import socket
from threading import Thread
from multiprocessing import Queue
from config import config
from utils import *
import time

arm_ip = "192.168.1.104"
force_port = 63351

class Collect():
    def __init__(self, cfg) -> None:
        ip = cfg['arm_ip']
        port = cfg['force_port']
        self.cfg = cfg
        self.rr = rtde_receive.RTDEReceiveInterface(ip)
        self.rc = rtde_control.RTDEControlInterface(ip)
        self.fs = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.fs.connect((ip, port))
        self.info_q = Queue()
        self.info_trd = Thread(target=self.collect)
        self._close = False
        self.info = []

    def calibrate(self, axis):
        # only change eef_R
        RT_we = self.get_RT_we()
        if axis == 'x':
            RT_we[:3,:3] = np.array([
                [0,0,1],
                [0,1,0],
                [-1,0,0],
            ])
        elif axis == 'y':
            RT_we[:3,:3] = np.array([
                [0,0,1],
                [-1,0,0],
                [0,-1,0],
            ])
        elif axis == 'z':
            RT_we[:3,:3] = np.array([
                [0,-1,0],
                [-1,0,0],
                [0,0,-1],
            ])
        self.pos_control(RT_we)

    def get_RT_we(self):
        RT_be = ur2np(self.rr.getActualTCPPose())
        RT_we = self.cfg['RT_wb'] @ RT_be
        return RT_we

    def pos_control(self, RT_we, speed=0.25, acc=0.5):
        RT_be = np.linalg.inv(self.cfg['RT_wb']) @ RT_we
        self.rc.moveL(np2ur(RT_be), speed, acc)

    def collect(self):
        while True:
            if self._close:
                print("close successed")
                break
            ft_data = self.fs.recv(9216).decode()
            ft_wrench = [float(i) for i in ft_data[1:-1].split(")(")[-1].split(" , ")]
            # ft_wrench_stack = np.roll(ft_wrench_stack, shift=-1, axis=0)
            # ft_wrench_stack[-1,...]=ft_wrench
            tcp_speed_b = self.rr.getActualTCPSpeed()
            # print('ft', ft_wrench)
            # print("append", np.r_[ft_wrench, tcp_speed_b])
            # self.info_q.put(np.r_[ft_wrench, tcp_speed_b])
            self.info.append(np.r_[ft_wrench, tcp_speed_b])
    
    def traj_move(self, pos, speed=0.25, acc=0.5):
        rot_vec = np2ur(self.get_RT_we())[-3:]
        self.pos_control(ur2np(np.r_[pos, rot_vec]), speed, acc)

    def save_data(self, filename):
        np.save(filename, self.info)

    def close(self):
        self._close = True

if __name__ == "__main__":
    c = Collect(config)
    # c.calibrate('x')
    # print(np2ur(c.get_RT_we()))
    c.info_trd.start()
    # print("trd start")
    # c.traj_move([-0.16746936,1.08857774,0.64925779], 1, 1) # 
    print("movement finished")
    time.sleep(4)
    c.close()
    # print("closed")
    c.save_data("only_force")
    # print("saved")

    # print(c.rr.getActualToolAccelerometer())