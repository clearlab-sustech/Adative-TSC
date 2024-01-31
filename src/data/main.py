import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np


# states
#  0-2   : pos
#  3-5   : vel
#  6-8   : omega
#  9-11  : pos_des
#  12-14 : vel_des
#  15-17 : omega_des

class DataPlot:
    def __init__(self, file1, file2):
        self.file1 = file1
        self.file2 = file2
        self.file3 = None
        self.fig = plt.figure()
        plt.autoscale(enable='true', axis='y')
        plt.legend()
        plt.grid()
    
    def angularMAE(self):
        data1 = np.loadtxt(self.file1)
        data2 = np.loadtxt(self.file2)
        
        mae_wbc = (np.absolute(data1[:, 6:9]- data1[:, 15:18])).mean(axis=0)
        mae_atsc =(np.absolute(data2[:, 6:9]- data2[:, 15:18])).mean(axis=0)

        species = ("omega x", "omega y", "omega z")
        penguin_means = {
            'feedback-mpc': mae_wbc/0.3,
            'proposed': mae_atsc/0.3,
        }

        x = np.arange(len(species))  # the label locations
        width = 0.25  # the width of the bars
        multiplier = 0

        fig, ax = plt.subplots(layout='constrained')

        for attribute, measurement in penguin_means.items():
            offset = width * multiplier
            rects = ax.bar(x + offset, measurement, width, label=attribute)
            ax.bar_label(rects, padding=3)
            multiplier += 1

        # Add some text for labels, title and custom x-axis tick labels, etc.
        ax.set_title('mean square error')
        ax.set_xticks(x + width, species)
        ax.legend(loc='upper right')
        # ax.set_ylim(0, 250)
        ax.set_title('mean absulute error')

    def errMSE(self):
        data1 = np.loadtxt(self.file1)[10000:25000,:]
        data2 = np.loadtxt(self.file2)[10000:25000,:]

        vel_mse_atsc = (np.linalg.norm(data1[:, 3:6]- data1[:, 12:15], axis=1)).mean(axis=0)
        vel_mse_wbc =(np.linalg.norm(data2[:, 3:6]- data2[:, 12:15], axis=1)).mean(axis=0)

        ang_mse_atsc = (np.linalg.norm(data1[:, 6:9]- data1[:, 15:18], axis=1)).mean(axis=0)
        ang_mse_wbc =(np.linalg.norm(data2[:, 6:9]- data2[:, 15:18], axis=1)).mean(axis=0)

        species = (' ')
        penguin_means1 = {
            'proposed': vel_mse_atsc,
            'user-tuned': vel_mse_wbc,
        }
        penguin_means2 = {
            'proposed': ang_mse_atsc,
            'user-tuned': ang_mse_wbc,
        }

        x = np.arange(len(species))  # the label locations
        width = 0.25  # the width of the bars
        multiplier = 0

        ax = plt.subplot(1, 2, 1)
        for attribute, measurement in penguin_means1.items():
            offset = width * multiplier
            rects = ax.bar(x + offset, measurement, width, label=attribute)
            ax.bar_label(rects, padding=3)
            multiplier += 1

        # Add some text for labels, title and custom x-axis tick labels, etc.
        ax.set_title('mean square error (linear velocity)')
        ax.set_xticks(x + width, species)
        ax.legend(loc='upper left')
        ax.set_ylim(0, 0.3)

        ax = plt.subplot(1, 2, 2)
        for attribute, measurement in penguin_means2.items():
            offset = width * multiplier
            rects = ax.bar(x + offset, measurement, width, label=attribute)
            ax.bar_label(rects, padding=3)
            multiplier += 1

        # Add some text for labels, title and custom x-axis tick labels, etc.
        ax.set_title('mean square error  (angular velocity)')
        ax.set_xticks(x + width, species)
        ax.legend(loc='upper left')
        ax.set_ylim(0, 1.8)

    def errLipMSE(self):
        data1 = np.loadtxt(self.file1)[10000:50000,:]
        data2 = np.loadtxt(self.file2)[10000:50000,:]

        vel_mse_atsc = (np.linalg.norm(data1[:, 3:6]- data1[:, 21:24], axis=1)).mean(axis=0)
        vel_mse_wbc =(np.linalg.norm(data2[:, 3:6]- data2[:, 21:24], axis=1)).mean(axis=0)

        ang_mse_atsc = (np.linalg.norm(data1[:, 6:9]- data1[:, 15:18], axis=1)).mean(axis=0)
        ang_mse_wbc =(np.linalg.norm(data2[:, 6:9]- data2[:, 15:18], axis=1)).mean(axis=0)

        species = (' ')
        penguin_means1 = {
            'proposed': vel_mse_atsc,
            'user-tuned': vel_mse_wbc,
        }
        penguin_means2 = {
            'proposed': ang_mse_atsc,
            'user-tuned': ang_mse_wbc,
        }

        x = np.arange(len(species))  # the label locations
        width = 0.25  # the width of the bars
        multiplier = 0

        ax = plt.subplot(1, 2, 1)
        for attribute, measurement in penguin_means1.items():
            offset = width * multiplier
            rects = ax.bar(x + offset, measurement, width, label=attribute)
            ax.bar_label(rects, padding=3)
            multiplier += 1

        # Add some text for labels, title and custom x-axis tick labels, etc.
        ax.set_title('mean square error (linear velocity)')
        ax.set_xticks(x + width, species)
        ax.legend(loc='upper left')
        ax.set_ylim(0, 0.3)

        ax = plt.subplot(1, 2, 2)
        for attribute, measurement in penguin_means2.items():
            offset = width * multiplier
            rects = ax.bar(x + offset, measurement, width, label=attribute)
            ax.bar_label(rects, padding=3)
            multiplier += 1

        # Add some text for labels, title and custom x-axis tick labels, etc.
        ax.set_title('mean square error  (angular velocity)')
        ax.set_xticks(x + width, species)
        ax.legend(loc='upper left')
        ax.set_ylim(0, 1.1)

    def linearVelMAE(self):
        data1 = np.loadtxt(self.file1)[10000:,:]
        data2 = np.loadtxt(self.file2)[10000:,:]

        n1 = len(data1)
        n2 = len(data2)

        time1 = np.linspace(0, (n1 - 1) * 0.002, n1)
        time2 = np.linspace(0, (n2 - 1) * 0.002, n2)

        ax1_1 = plt.subplot(3, 2, 1)
        ax1_1.set_title('vel x tracking err (feedback-mpc)')
        ax1_1.plot(time1, data1[:n1, 3], lw='1', label="measured")
        ax1_1.plot(time1, data1[:n1, 12], lw='1', color='r', label="reference")
        ax1_1.set_ylabel("vel (m/s)", fontsize=12)
        ax1_1.set_xlim([-1, 45])
        ax1_1.set_ylim([-1, 1])

        ax1_2 = plt.subplot(3, 2, 3)
        ax1_2.set_title('vel y tracking err (feedback-mpc)')
        ax1_2.plot(time1, data1[:n1, 4], lw='1', label="measured")
        ax1_2.plot(time1, data1[:n1, 13], lw='1', color='r', label="reference")
        ax1_2.set_ylabel("vel (m/s)", fontsize=12)
        ax1_2.set_xlim([-1, 45])
        ax1_2.set_ylim([-0.8, 0.8])

        ax1_3 = plt.subplot(3, 2, 5)
        ax1_3.set_title('vel z tracking err (feedback-mpc)')
        ax1_3.plot(time1, data1[:n1, 5], lw='1', label="measured")
        ax1_3.plot(time1, data1[:n1, 14], lw='1', color='r', label="reference")
        ax1_3.set_xlabel("time (s)", fontsize=12)
        ax1_3.set_ylabel("vel (m/s)", fontsize=12)
        ax1_3.set_xlim([-1, 45])
        ax1_3.set_ylim([-0.6, 0.6])

        ax1_4 = plt.subplot(3, 2, 2)
        ax1_4.set_title('vel x tracking err (proposed)')
        ax1_4.plot(time2, data2[:n2, 3], lw='1', label="measured")
        ax1_4.plot(time2, data2[:n2, 12], lw='1', color='r', label="reference")
        ax1_4.set_xlim([-1, 45])
        ax1_4.set_ylim([-1, 1])

        ax1_5 = plt.subplot(3, 2, 4)
        ax1_5.set_title('vel y tracking err (proposed)')
        ax1_5.plot(time2, data2[:n2, 4], lw='1', label="measured")
        ax1_5.plot(time2, data2[:n2, 13], lw='1', color='r', label="reference")
        ax1_5.set_xlim([-1, 45])
        ax1_5.set_ylim([-0.8, 0.8])

        ax1_6 = plt.subplot(3, 2, 6)
        ax1_6.set_title('vel z tracking err (proposed)')
        ax1_6.plot(time2, data2[:n2, 5], lw='1', label="measured")
        ax1_6.plot(time2, data2[:n2, 14], lw='1', color='r', label="reference")
        ax1_6.set_xlabel("time (s)", fontsize=12)
        ax1_6.set_xlim([-1, 45])
        ax1_6.set_ylim([-0.6, 0.6])

    def angularVelMAE(self):
        data1 = np.loadtxt(self.file1)[10000:,:]
        data2 = np.loadtxt(self.file2)[10000:,:]

        n1 = len(data1)
        n2 = len(data2)

        time1 = np.linspace(0, (n1 - 1) * 0.002, n1)
        time2 = np.linspace(0, (n2 - 1) * 0.002, n2)

        ax1_1 = plt.subplot(3, 2, 1)
        ax1_1.set_title('angular vel x tracking err (feedback-mpc)')
        ax1_1.plot(time1, data1[:n1, 6], lw='1', label="measured")
        ax1_1.plot(time1, data1[:n1, 15], lw='1', color='r', label="reference")
        ax1_1.set_ylabel("angular vel (rad/s)", fontsize=12)
        ax1_1.set_xlim([-1, 20])
        ax1_1.set_ylim([-3, 3])

        ax1_2 = plt.subplot(3, 2, 3)
        ax1_2.set_title('angular vel y tracking err (feedback-mpc)')
        ax1_2.plot(time1, data1[:n1, 7], lw='1', label="measured")
        ax1_2.plot(time1, data1[:n1, 16], lw='1', color='r', label="reference")
        ax1_2.set_ylabel("angular vel (rad/s)", fontsize=12)
        ax1_2.set_xlim([-1, 20])
        ax1_2.set_ylim([-3, 3])

        ax1_3 = plt.subplot(3, 2, 5)
        ax1_3.set_title('angular vel z tracking err (feedback-mpc)')
        ax1_3.plot(time1, data1[:n1, 8], lw='1', label="measured")
        ax1_3.plot(time1, data1[:n1, 17], lw='1', color='r', label="reference")
        ax1_3.set_xlabel("time (s)", fontsize=12)
        ax1_3.set_ylabel("angular vel (rad/s)", fontsize=12)
        ax1_3.set_xlim([-1, 20])
        ax1_3.set_ylim([-3, 3])

        ax1_4 = plt.subplot(3, 2, 2)
        ax1_4.set_title('angular vel x tracking err (proposed)')
        ax1_4.plot(time2, data2[:n2, 6], lw='1', label="measured")
        ax1_4.plot(time2, data2[:n2, 15], lw='1', color='r', label="reference")
        ax1_4.set_xlim([-1, 20])
        ax1_4.set_ylim([-3, 3])

        ax1_5 = plt.subplot(3, 2, 4)
        ax1_5.set_title('angular vel y tracking err (proposed)')
        ax1_5.plot(time2, data2[:n2, 7], lw='1', label="measured")
        ax1_5.plot(time2, data2[:n2, 16], lw='1', color='r', label="reference")
        ax1_5.set_xlim([-1, 20])
        ax1_5.set_ylim([-3, 3])

        ax1_6 = plt.subplot(3, 2, 6)
        ax1_6.set_title('angular vel z tracking err (proposed)')
        ax1_6.plot(time2, data2[:n2, 8], lw='1', label="measured")
        ax1_6.plot(time2, data2[:n2, 17], lw='1', color='r', label="reference")
        ax1_6.set_xlabel("time (s)", fontsize=12)
        ax1_6.set_xlim([-1, 20])
        ax1_6.set_ylim([-3, 3])


if __name__ == "__main__":
    path1 = '/home/poplar/Desktop/VF-TSC/src/data/sim/lateral/atsc/data_log.txt'
    path2 = '/home/poplar/Desktop/VF-TSC/src/data/sim/lateral/wbc/data_log.txt'
    d = DataPlot(path1, path2)
    d.errMSE()
    plt.show()
