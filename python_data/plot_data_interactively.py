#%%
import numpy as np
from matplotlib import pyplot as plt

#%%
state = np.loadtxt('../log/state.txt', dtype=np.float32)
cmd = np.loadtxt('../log/cmd.txt', dtype=np.float32)
start = 0
end = -1
# end = -1
offset = 3 + 12*3 + 3*4
offset2 = 12*4 + 3*4
plt.subplot(2, 2, 1)
plt.plot(state[start:end, offset+0])
plt.plot(cmd[start:end, offset2+0])
plt.legend(['real x', 'planned x'])
plt.title("position x")

# plt.subplot(2, 2, 1)
# plt.plot(state[start:end, 12*3+3*3+0])
# plt.plot(cmd[start:end, 12*4+3*2+0])
# plt.legend(['real', 'planned'])
# plt.title("velocity x")

plt.subplot(2, 2, 2)
legends = []
ranges = [0, 1, 2, 3]
for i in ranges :
    plt.plot(state[start:end, 6*i+offset+2])
    plt.plot(cmd[start:end, 12*i+offset2+2])
    legends.append('real leg {}'.format(i))
    legends.append('planned leg {}'.format(i))
plt.legend(legends)
plt.title("position z")

plt.subplot(2, 2, 3)
legends = []
for i in ranges :
    plt.plot(state[start:end, 6*i+offset+3+2])
    plt.plot(cmd[start:end, 12*i+offset2+3+2])
    legends.append('real leg {}'.format(i))
    legends.append('planned leg {}'.format(i))
plt.legend(legends)
plt.title("velocity z")

plt.subplot(2, 2, 4)
legends = []
for i in ranges :
    plt.plot(cmd[start:end, 12*i+offset2+3+3+2])
    legends.append('planned leg {}'.format(i))
plt.legend(legends)
plt.legend(['planned'])
plt.title("acc z")

plt.show()