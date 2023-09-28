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
    def __init__(self, file):
        self.file = file
        self.data = np.loadtxt(self.file)
        self.time = np.linspace(0, (len(self.data) - 1) * 0.02, len(self.data))
        self.fig = plt.figure(file)

    def main(self):
        self.data_plot()

    def data_plot(self):
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


if __name__ == "__main__":
    d = DataPlot('./data_log_atsc_Sept28.txt')
    d.main()
    plt.show()

