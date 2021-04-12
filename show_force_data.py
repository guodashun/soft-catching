import numpy as np
import matplotlib.pyplot as plt

data = np.load("force_data.npy")
t = range(len(data))
plt.plot(t, data[:,0], label="x")
plt.plot(t, data[:,1], label="y")
plt.plot(t, data[:,2], label="z")
plt.xlim(000,350)
# plt.ylim(000,10)
plt.legend()
plt.title("Sim Force")
plt.show()
