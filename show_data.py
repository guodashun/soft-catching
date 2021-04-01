import numpy as np
import matplotlib.pyplot as plt
from utils import *
from config import config

axis_dic = {
    "x":0,
    "y":1,
    "z":2,
}
cate_dic = {
    "force":0,
    "torque":1,
}
FORCE, TORQUE = 0, 1

def show_data(data, axis='x', cate='force'):
    t = range(len(data))
    # plt.scatter(data[:, 6+axis], data[:,axis+cate*3])
    plt.scatter(t, data[:,axis_dic[axis]+cate_dic[cate]*3], label=cate)
    # plt.scatter(t, data[:, 6+axis_dic[axis]], color='r', label='v')
    plt.title(f"{cate}({axis})")
    plt.legend()
    plt.show()


if __name__ == "__main__":
    axis = 'z'
    # data = np.load(f'{axis}_speed4.npy')
    data = np.load(f'only_force.npy')

    for i in range(len(data)):
        data[i,6:12] = np2ur(config['RT_wb'] @ ur2np(data[i,6:12]))
    show_data(data, axis, 'force')