import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np


# states
#  0-2   : pos
#  3-5   : vel
#  6-8   : angular
#  9-11  : pos_des
#  12-14 : vel_des
#  15-17 : angular_des

class DataPlot:
    def __init__(self, file1, file2):
        self.file1 = file1
        self.file2 = file2
        self.fig = plt.figure()

    def base_pose(self):
        self.data = np.loadtxt(self.file1)
        self.time = np.linspace(0, (len(self.data) - 1) * 0.02, len(self.data))
    
        time = self.time
        data = self.data

        ax1_1 = plt.subplot(3, 1, 1)
        ax1_1.set_title('pos')
        ax1_1.plot(time, data[:, 0], lw='1', label="pos_x")
        ax1_1.plot(time, data[:, 1], lw='1', label="pos_y")
        ax1_1.plot(time, data[:, 2], lw='1', label="pos_z")
        ax1_1.plot(time, data[:, 9], lw='1', label="pos_xd")
        ax1_1.plot(time, data[:, 10], lw='1', label="pos_yd")
        ax1_1.plot(time, data[:, 11], lw='1', label="pos_zd")
        plt.autoscale(enable='true', axis='y')
        plt.legend()
        plt.grid()

        ax1_1 = plt.subplot(3, 1, 2)
        ax1_1.set_title('vel')
        ax1_1.plot(time, data[:, 3], lw='1', label="vel_x")
        ax1_1.plot(time, data[:, 4], lw='1', label="vel_y")
        ax1_1.plot(time, data[:, 5], lw='1', label="vel_z")
        ax1_1.plot(time, data[:, 12], lw='1', label="vel_xd")
        ax1_1.plot(time, data[:, 13], lw='1', label="vel_yd")
        ax1_1.plot(time, data[:, 14], lw='1', label="vel_zd")
        plt.autoscale(enable='true', axis='y')
        plt.legend()
        plt.grid()

        ax1_1 = plt.subplot(3, 1, 3)
        ax1_1.set_title('ang_vel')
        ax1_1.plot(time, data[:, 6], lw='1', label="ang_vel_x")
        ax1_1.plot(time, data[:, 7], lw='1', label="ang_vel_y")
        ax1_1.plot(time, data[:, 8], lw='1', label="ang_vel_z")
        ax1_1.plot(time, data[:, 15], lw='1', label="ang_vel_xd")
        ax1_1.plot(time, data[:, 16], lw='1', label="ang_vel_yd")
        ax1_1.plot(time, data[:, 17], lw='1', label="ang_vel_zd")
        plt.autoscale(enable='true', axis='y')
        plt.legend()
        plt.grid()
    
    def actuators_cmds(self):
        self.data1 = np.loadtxt(self.file1)
        self.data2 = np.loadtxt(self.file2)
        
        time = np.linspace(0, (len(self.data1) - 1) * 0.001, len(self.data1))
        data1 = self.data1[:,1::]
        data2 = self.data2[:,1::]

        ax1_1 = plt.subplot(3, 1, 1)
        ax1_1.set_title('abad')
        ax1_1.plot(time, data1[:, 0], lw='1', label="abad_wbc")
        ax1_1.plot(time + 0.035, data2[:, 0], lw='1', label="abad_atsc")
        plt.autoscale(enable='true', axis='y')
        plt.legend()
        plt.grid()

        ax1_1 = plt.subplot(3, 1, 2)
        ax1_1.set_title('hip')
        ax1_1.plot(time, data1[:, 1], lw='1', label="hip_wbc")
        ax1_1.plot(time + 0.035, data2[:, 1], lw='1', label="hip_atsc")
        plt.autoscale(enable='true', axis='y')
        plt.legend()
        plt.grid()

        ax1_1 = plt.subplot(3, 1, 3)
        ax1_1.set_title('knee')
        ax1_1.plot(time, data1[:, 2], lw='1', label="knee_wbc")
        ax1_1.plot(time  + 0.035, data2[:, 2], lw='1', label="knee_atsc")
        plt.autoscale(enable='true', axis='y')
        plt.legend()
        plt.grid()

    def base_acc(self):
        data1 = np.loadtxt(self.file1)
        data2 = np.loadtxt(self.file2)
        
        n = min(len(data1), len(data2))
        time = np.linspace(0, (n - 1) * 0.001, n)

        ax1_1 = plt.subplot(3, 2, 1)
        ax1_1.set_title('acc x')
        ax1_1.plot(time, data1[:n, 0], lw='1', label="wbc")
        ax1_1.plot(time, data2[:n, 0], lw='1', label="atsc")
        plt.autoscale(enable='true', axis='y')
        plt.legend()
        plt.grid()

        ax1_1 = plt.subplot(3, 2, 3)
        ax1_1.set_title('acc y')
        ax1_1.plot(time, data1[:n, 1], lw='1', label="wbc")
        ax1_1.plot(time, data2[:n, 1], lw='1', label="atsc")
        plt.autoscale(enable='true', axis='y')
        plt.legend()
        plt.grid()

        ax1_1 = plt.subplot(3, 2, 5)
        ax1_1.set_title('acc z')
        ax1_1.plot(time, data1[:n, 2], lw='1', label="wbc")
        ax1_1.plot(time, data2[:n, 2], lw='1', label="atsc")
        plt.autoscale(enable='true', axis='y')
        plt.legend()
        plt.grid()

        ax1_1 = plt.subplot(3, 2, 2)
        ax1_1.set_title('ang acc x')
        ax1_1.plot(time, data1[:n, 3], lw='1', label="wbc")
        ax1_1.plot(time, data2[:n, 3], lw='1', label="atsc")
        plt.autoscale(enable='true', axis='y')
        plt.legend()
        plt.grid()

        ax1_1 = plt.subplot(3, 2, 4)
        ax1_1.set_title('ang acc y')
        ax1_1.plot(time, data1[:n, 4], lw='1', label="wbc")
        ax1_1.plot(time, data2[:n, 4], lw='1', label="atsc")
        plt.autoscale(enable='true', axis='y')
        plt.legend()
        plt.grid()

        ax1_1 = plt.subplot(3, 2, 6)
        ax1_1.set_title('ang acc z')
        ax1_1.plot(time, data1[:n, 5], lw='1', label="wbc")
        ax1_1.plot(time, data2[:n, 5], lw='1', label="atsc")
        plt.autoscale(enable='true', axis='y')
        plt.legend()
        plt.grid()

if __name__ == "__main__":
    # d = DataPlot('./actuator_cmds_log_wbc.txt', './actuator_cmds_log_atsc.txt')
    d = DataPlot('./acc_log_wbc.txt', './acc_log_atsc.txt')
    d.base_acc()
    plt.show()

