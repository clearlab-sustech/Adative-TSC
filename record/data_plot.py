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

    def err_mse(self):
        data1 = np.loadtxt(self.file1)
        data2 = np.loadtxt(self.file2)

        mse_wbc = ((data1)**2).mean(axis=0)
        mse_atsc = ((data2)**2).mean(axis=0)

        species = (#"pos x", "pos y", "pos z", "roll", "pitch", "yaw", 
                   "vel x", "vel y", "vel z", "omega x", "omega y", "omega z")
    #    "acc x", "acc y", "acc z", "omega_dot x", "omega_dot y", "omega_dot z"
        penguin_means = {
            'mse_wbc': mse_wbc[6:12]/0.3,
            'mse_atsc': mse_atsc[6:12]/0.3,
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
        ax.legend(loc='upper right', ncols=3)
        # ax.set_ylim(0, 250)

    def err_mean(self):
        data1 = np.loadtxt(self.file1)
        data2 = np.loadtxt(self.file2)

        mse_wbc = (data1).mean(axis=0)
        mse_atsc = (data2).mean(axis=0)

        species = (#"pos x", "pos y", "pos z", "roll", "pitch", "yaw", 
                   "vel x", "vel y", "vel z", "omega x", "omega y", "omega z")
    #    "acc x", "acc y", "acc z", "omega_dot x", "omega_dot y", "omega_dot z"
        penguin_means = {
            'mse_wbc': mse_wbc[6:12]/0.3,
            'mse_atsc': mse_atsc[6:12]/0.3,
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
        ax.legend(loc='upper right', ncols=3)
        # ax.set_ylim(0, 250)

    def base_pose_err(self):
        data1 = np.loadtxt(self.file1)
        data2 = np.loadtxt(self.file2)

        n = min(len(data1), len(data2))
        time = np.linspace(0, (n - 1) * 0.001, n)

        ax1_1 = plt.subplot(3, 2, 1)
        ax1_1.set_title('pos x tracking err')
        ax1_1.plot(time, data1[:n, 0], lw='1', label="wbc")
        ax1_1.plot(time, data2[:n, 0], lw='1', label="atsc")
        plt.autoscale(enable='true', axis='y')
        plt.legend()
        plt.grid()

        ax1_1 = plt.subplot(3, 2, 3)
        ax1_1.set_title('pos y tracking err')
        ax1_1.plot(time, data1[:n, 1], lw='1', label="wbc")
        ax1_1.plot(time, data2[:n, 1], lw='1', label="atsc")
        plt.autoscale(enable='true', axis='y')
        plt.legend()
        plt.grid()

        ax1_1 = plt.subplot(3, 2, 5)
        ax1_1.set_title('pos z tracking err')
        ax1_1.plot(time, data1[:n, 2], lw='1', label="wbc")
        ax1_1.plot(time, data2[:n, 2], lw='1', label="atsc")
        plt.autoscale(enable='true', axis='y')
        plt.legend()
        plt.grid()


        ax1_1 = plt.subplot(3, 2, 2)
        ax1_1.set_title('roll tracking err')
        ax1_1.plot(time, data1[:n, 3], lw='1', label="wbc")
        ax1_1.plot(time, data2[:n, 3], lw='1', label="atsc")
        plt.autoscale(enable='true', axis='y')
        plt.legend()
        plt.grid()

        ax1_1 = plt.subplot(3, 2, 4)
        ax1_1.set_title('pitch tracking err')
        ax1_1.plot(time, data1[:n, 4], lw='1', label="wbc")
        ax1_1.plot(time, data2[:n, 4], lw='1', label="atsc")
        plt.autoscale(enable='true', axis='y')
        plt.legend()
        plt.grid()

        ax1_1 = plt.subplot(3, 2, 6)
        ax1_1.set_title('yaw tracking err')
        ax1_1.plot(time, data1[:n, 5], lw='1', label="wbc")
        ax1_1.plot(time, data2[:n, 5], lw='1', label="atsc")
        plt.autoscale(enable='true', axis='y')
        plt.legend()
        plt.grid()
    
    def base_vel_err(self):
        data1 = np.loadtxt(self.file1)
        data2 = np.loadtxt(self.file2)

        n = min(len(data1), len(data2))
        time = np.linspace(0, (n - 1) * 0.001, n)

        ax1_1 = plt.subplot(3, 2, 1)
        ax1_1.set_title('vel x tracking err')
        ax1_1.plot(time, data1[:n, 6], lw='1', label="wbc")
        ax1_1.plot(time, data2[:n, 6], lw='1', label="atsc")
        plt.autoscale(enable='true', axis='y')
        plt.legend()
        plt.grid()

        ax1_1 = plt.subplot(3, 2, 3)
        ax1_1.set_title('vel y tracking err')
        ax1_1.plot(time, data1[:n, 7], lw='1', label="wbc")
        ax1_1.plot(time, data2[:n, 7], lw='1', label="atsc")
        plt.autoscale(enable='true', axis='y')
        plt.legend()
        plt.grid()

        ax1_1 = plt.subplot(3, 2, 5)
        ax1_1.set_title('vel z tracking err')
        ax1_1.plot(time, data1[:n, 8], lw='1', label="wbc")
        ax1_1.plot(time, data2[:n, 8], lw='1', label="atsc")
        plt.autoscale(enable='true', axis='y')
        plt.legend()
        plt.grid()


        ax1_1 = plt.subplot(3, 2, 2)
        ax1_1.set_title('roll vel tracking err')
        ax1_1.plot(time, data1[:n, 9], lw='1', label="wbc")
        ax1_1.plot(time, data2[:n, 9], lw='1', label="atsc")
        plt.autoscale(enable='true', axis='y')
        plt.legend()
        plt.grid()

        ax1_1 = plt.subplot(3, 2, 4)
        ax1_1.set_title('pitch vel tracking err')
        ax1_1.plot(time, data1[:n, 10], lw='1', label="wbc")
        ax1_1.plot(time, data2[:n, 10], lw='1', label="atsc")
        plt.autoscale(enable='true', axis='y')
        plt.legend()
        plt.grid()

        ax1_1 = plt.subplot(3, 2, 6)
        ax1_1.set_title('yaw vel tracking err')
        ax1_1.plot(time, data1[:n, 11], lw='1', label="wbc")
        ax1_1.plot(time, data2[:n, 11], lw='1', label="atsc")
        plt.autoscale(enable='true', axis='y')
        plt.legend()
        plt.grid()


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

    def push(self):
        data1 = np.loadtxt(self.file1)
        data2 = np.loadtxt(self.file2)
        n = min(len(data1), len(data2))

        time = np.linspace(0, (n - 1) * 0.02, n)
    
        ax1_1 = plt.subplot(3, 1, 1)
        ax1_1.set_title('ang_vel_x')
        ax1_1.plot(time, data1[:n, 6], lw='1', label="wbc")
        ax1_1.plot(time, data2[:n, 6], lw='1', label="atsc")
        plt.autoscale(enable='true', axis='y')
        plt.legend()
        plt.grid()

        ax1_1 = plt.subplot(3, 1, 2)
        ax1_1.set_title('ang_vel_y')
        ax1_1.plot(time, data1[:n, 7], lw='1', label="wbc")
        ax1_1.plot(time, data2[:n, 7], lw='1', label="atsc")
        plt.autoscale(enable='true', axis='y')
        plt.legend()
        plt.grid()

        ax1_1 = plt.subplot(3, 1, 3)
        ax1_1.set_title('ang_vel_z')
        ax1_1.plot(time, data1[:n, 8], lw='1', label="wbc")
        ax1_1.plot(time, data2[:n, 8], lw='1', label="atsc")
        plt.autoscale(enable='true', axis='y')
        plt.legend()
        plt.grid()

if __name__ == "__main__":
    # d = DataPlot('./actuator_cmds_log_wbc.txt', './actuator_cmds_log_atsc.txt')
    # d = DataPlot('./wbc/acc_log.txt', './atsc/acc_log.txt')
    # d = DataPlot('./wbc/push/acc_log.txt', './atsc/push/acc_log.txt')
    # d = DataPlot('./wbc/push/data_log.txt', './atsc/push/data_log.txt')
    d = DataPlot('./wbc/30Hz05/log_stream_wbc.txt', './atsc/30Hz05/log_stream_wbc.txt')

    d.base_vel_err()
    plt.show()

