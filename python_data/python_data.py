#!/usr/bin/env python2
from os import stat
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np


# states
#  0-11   : qpos
#  12-23  : qvel
#  24-35  : tau
#  36-45  : IMU
#  46-48  : floating base pos
#  49-51  : floating base rpy
#  52-63  : foot state pos

# cmd
# 0-11  : tau
# 12-23 : grf
# 24-35 : qdes
# 36-47 : foot cmd pos
# 48-53 : com task


# 41-43: base angular velocity
# 44-47: base quat
# 48-49: height
# 50-51: ddh reference
# 52-54: LQR state(theta, dtheta, vel)
# 55-56: ddt reference
# 57   : length of inverted pendulum
# 58   : control mode
# 59-60: LQR ref(theta, dtheta)
# 61-62: LQR state(theta, dtheta)
# 63   : I_theta
# 64   : wheel_vel

class DataPlot:
    def __init__(self, statefile, cmdfile):
        self.statefile = statefile
        self.cmdfile = cmdfile
        self.states = np.loadtxt(self.statefile)
        self.cmd = np.loadtxt(self.cmdfile)

        self.time = np.linspace(0, (len(self.states) - 1) * 0.002, len(self.states))
        self.fig = plt.figure(statefile)
        # print(np.shape(self.time))
        # self.start = start
        # self.end = end

    def main(self):
        # pass
        # self.plot_foot_state_err()
        self.plot_foot_state()
        # self.plot_com()
        # self.plot_com_vel()
        # self.plot_vetex()

        # self.plot_joint_pos()
        # self.plot_joint_pos_err()
        # # plt.plot(self.time,self.states[:,1])
        # self.plot_joint_vel()
        # self.plot_joint_tau()
        # self.plot_force_ref()

        # self.plot_foot_state()
        # self.plot3d()

    def plot3d(self):
        time = self.time
        states = self.states
        cmd = self.cmd

        ax1_1 = self.fig.gca(projection='3d')
        ax1_1.plot(states[:, 39], states[:, 40], states[:, 41], lw='1', label="com_imu")
        ax1_1.plot(states[:, 97], states[:, 98], states[:, 99], lw='1', label="com_version")
        ax1_1.plot(cmd[:, 48], cmd[:, 49], cmd[:, 50], lw='1', label="cmd_x")

        offset_state = 51
        offset_cmd = 60
        ax1_1.plot(states[:, offset_state], states[:, offset_state + 1], states[:, offset_state + 2], lw='1',
                   label="fl_foot")
        ax1_1.plot(cmd[:, offset_cmd], cmd[:, offset_cmd + 1], cmd[:, offset_cmd + 2], lw='1', label="fl_foot_cmd")
        ax1_1.autoscale(enable='true', axis='y')
        ax1_1.legend()
        ax1_1.grid()

    def plot_vetex(self):
        x = [-0.830000, -0.681000, -0.731000, -0.809000]
        y = [-0.407000, -0.611000, -0.652000, -0.670000]
        plt.plot(x, y)

    def plot_joint_vel(self):
        time = self.time
        states = self.states
        cmd = self.cmd

        offset_state = 12
        ax1_1 = plt.subplot(2, 2, 1)
        ax1_1.set_title('foot_fl')
        ax1_1.plot(time, states[:, offset_state + 0], lw='1', label="abad")
        ax1_1.plot(time, states[:, offset_state + 1], lw='1', label="hip")
        ax1_1.plot(time, states[:, offset_state + 2], lw='1', label="knee")
        # ax1_1.plot(time, cmd[:,24],lw='1', label="cmd_abad")
        # ax1_1.plot(time, cmd[:,25],lw='1', label="cmd_hip")
        # ax1_1.plot(time, cmd[:,26],lw='1', label="cmd_knee")
        # ax1_1.set_xlim(self.start, self.end)
        plt.autoscale(enable='true', axis='y')
        plt.legend()
        plt.grid()

        # Plot 2: theta
        ax1_1 = plt.subplot(2, 2, 2)
        ax1_1.set_title('foot_fr')
        ax1_1.plot(time, states[:, offset_state + 3], lw='1', label="abad")
        ax1_1.plot(time, states[:, offset_state + 4], lw='1', label="hip")
        ax1_1.plot(time, states[:, offset_state + 5], lw='1', label="knee")
        # ax1_1.plot(time, cmd[:,27],lw='1', label="cmd_abad")
        # ax1_1.plot(time, cmd[:,28],lw='1', label="cmd_hip")
        # ax1_1.plot(time, cmd[:,29],lw='1', label="cmd_knee")
        # ax1_1.set_xlim(self.start, self.end)
        plt.autoscale(enable='true', axis='y')
        plt.legend()
        plt.grid()

        # Plot 3: I_theta
        ax1_1 = plt.subplot(2, 2, 3)
        ax1_1.set_title('foot_rl')
        ax1_1.plot(time, states[:, offset_state + 6], lw='1', label="abad")
        ax1_1.plot(time, states[:, offset_state + 7], lw='1', label="hip")
        ax1_1.plot(time, states[:, offset_state + 8], lw='1', label="knee")
        # ax1_1.plot(time, cmd[:,30],lw='1', label="cmd_abad")
        # ax1_1.plot(time, cmd[:,31],lw='1', label="cmd_hip")
        # ax1_1.plot(time, cmd[:,32],lw='1', label="cmd_knee")
        # ax1_1.set_xlim(self.start, self.end)
        plt.autoscale(enable='true', axis='y')
        plt.legend()
        plt.grid()

        # Plot 4: theta_ref
        ax1_1 = plt.subplot(2, 2, 4)
        ax1_1.set_title('foot_rr')
        ax1_1.plot(time, states[:, offset_state + 9], lw='1', label="abad")
        ax1_1.plot(time, states[:, offset_state + 10], lw='1', label="hip")
        ax1_1.plot(time, states[:, offset_state + 11], lw='1', label="knee")
        # ax1_1.plot(time, cmd[:,33],lw='1', label="cmd_abad")
        # ax1_1.plot(time, cmd[:,34],lw='1', label="cmd_hip")
        # ax1_1.plot(time, cmd[:,35],lw='1', label="cmd_knee")
        # ax1_1.set_xlim(self.start, self.end)
        plt.autoscale(enable='true', axis='y')
        plt.legend()
        plt.grid()
        # Plot the Figure 2
        # fig2 = plt.figure(figsize=(15,10))

        # show the figure

    def plot_joint_tau(self):
        time = self.time
        states = self.states
        cmd = self.cmd

        offset_state = 27
        offset_cmd = 24

        ax1_1 = plt.subplot(2, 2, 1)
        ax1_1.set_title('foot_fl')
        ax1_1.plot(time, states[:, offset_state + 0], lw='1', label="abad")
        ax1_1.plot(time, states[:, offset_state + 1], lw='1', label="hip")
        ax1_1.plot(time, states[:, offset_state + 2], lw='1', label="knee")
        ax1_1.plot(time, cmd[:, offset_cmd + 0], lw='1', label="cmd_abad")
        ax1_1.plot(time, cmd[:, offset_cmd + 1], lw='1', label="cmd_hip")
        ax1_1.plot(time, cmd[:, offset_cmd + 2], lw='1', label="cmd_knee")
        # ax1_1.set_xlim(self.start, self.end)
        plt.autoscale(enable='true', axis='y')
        plt.legend()
        plt.grid()

        # Plot 2: theta
        ax1_1 = plt.subplot(2, 2, 2)
        ax1_1.set_title('foot_fr')
        ax1_1.plot(time, states[:, offset_state + 3], lw='1', label="abad")
        ax1_1.plot(time, states[:, offset_state + 4], lw='1', label="hip")
        ax1_1.plot(time, states[:, offset_state + 5], lw='1', label="knee")
        ax1_1.plot(time, cmd[:, offset_cmd + 3], lw='1', label="cmd_abad")
        ax1_1.plot(time, cmd[:, offset_cmd + 4], lw='1', label="cmd_hip")
        ax1_1.plot(time, cmd[:, offset_cmd + 5], lw='1', label="cmd_knee")
        # ax1_1.set_xlim(self.start, self.end)
        plt.autoscale(enable='true', axis='y')
        plt.legend()
        plt.grid()

        # Plot 3: I_theta
        ax1_1 = plt.subplot(2, 2, 3)
        ax1_1.set_title('foot_rl')
        ax1_1.plot(time, states[:, offset_state + 6], lw='1', label="abad")
        ax1_1.plot(time, states[:, offset_state + 7], lw='1', label="hip")
        ax1_1.plot(time, states[:, offset_state + 8], lw='1', label="knee")
        ax1_1.plot(time, cmd[:, offset_cmd + 6], lw='1', label="cmd_abad")
        ax1_1.plot(time, cmd[:, offset_cmd + 7], lw='1', label="cmd_hip")
        ax1_1.plot(time, cmd[:, offset_cmd + 8], lw='1', label="cmd_knee")
        # ax1_1.set_xlim(self.start, self.end)
        plt.autoscale(enable='true', axis='y')
        plt.legend()
        plt.grid()

        # Plot 4: theta_ref
        ax1_1 = plt.subplot(2, 2, 4)
        ax1_1.set_title('foot_rr')
        ax1_1.plot(time, states[:, offset_state + 9], lw='1', label="abad")
        ax1_1.plot(time, states[:, offset_state + 10], lw='1', label="hip")
        ax1_1.plot(time, states[:, offset_state + 11], lw='1', label="knee")
        ax1_1.plot(time, cmd[:, offset_cmd + 9], lw='1', label="cmd_abad")
        ax1_1.plot(time, cmd[:, offset_cmd + 10], lw='1', label="cmd_hip")
        ax1_1.plot(time, cmd[:, offset_cmd + 11], lw='1', label="cmd_knee")
        # ax1_1.set_xlim(self.start, self.end)
        plt.autoscale(enable='true', axis='y')
        plt.legend()
        plt.grid()
        # Plot the Figure 2
        # fig2 = plt.figure(figsize=(15,10))

        # show the figure

    def plot_force_ref(self):
        time = self.time
        states = self.states
        cmd = self.cmd

        offset_state = 36
        ax1_1 = plt.subplot(2, 2, 1)
        ax1_1.set_title('foot_fl')
        ax1_1.plot(time, cmd[:, offset_state + 0], lw='1', label="x")
        ax1_1.plot(time, cmd[:, offset_state + 1], lw='1', label="y")
        ax1_1.plot(time, cmd[:, offset_state + 2], lw='1', label="z")
        # ax1_1.plot(time, cmd[:,24],lw='1', label="cmd_abad")
        # ax1_1.plot(time, cmd[:,25],lw='1', label="cmd_hip")
        # ax1_1.plot(time, cmd[:,26],lw='1', label="cmd_knee")
        # ax1_1.set_xlim(self.start, self.end)
        plt.autoscale(enable='true', axis='y')
        plt.legend()
        plt.grid()

        # Plot 2: theta
        ax1_1 = plt.subplot(2, 2, 2)
        ax1_1.set_title('foot_fr')
        ax1_1.plot(time, cmd[:, offset_state + 3], lw='1', label="x")
        ax1_1.plot(time, cmd[:, offset_state + 4], lw='1', label="y")
        ax1_1.plot(time, cmd[:, offset_state + 5], lw='1', label="z")
        # ax1_1.plot(time, cmd[:,27],lw='1', label="cmd_abad")
        # ax1_1.plot(time, cmd[:,28],lw='1', label="cmd_hip")
        # ax1_1.plot(time, cmd[:,29],lw='1', label="cmd_knee")
        # ax1_1.set_xlim(self.start, self.end)
        plt.autoscale(enable='true', axis='y')
        plt.legend()
        plt.grid()

        # Plot 3: I_theta
        ax1_1 = plt.subplot(2, 2, 3)
        ax1_1.set_title('foot_rl')
        ax1_1.plot(time, cmd[:, offset_state + 6], lw='1', label="x")
        ax1_1.plot(time, cmd[:, offset_state + 7], lw='1', label="y")
        ax1_1.plot(time, cmd[:, offset_state + 8], lw='1', label="z")
        # ax1_1.plot(time, cmd[:,30],lw='1', label="cmd_abad")
        # ax1_1.plot(time, cmd[:,31],lw='1', label="cmd_hip")
        # ax1_1.plot(time, cmd[:,32],lw='1', label="cmd_knee")
        # ax1_1.set_xlim(self.start, self.end)
        plt.autoscale(enable='true', axis='y')
        plt.legend()
        plt.grid()

        # Plot 4: theta_ref
        ax1_1 = plt.subplot(2, 2, 4)
        ax1_1.set_title('foot_rr')
        ax1_1.plot(time, cmd[:, offset_state + 9], lw='1', label="x")
        ax1_1.plot(time, cmd[:, offset_state + 10], lw='1', label="y")
        ax1_1.plot(time, cmd[:, offset_state + 11], lw='1', label="z")
        # ax1_1.plot(time, cmd[:,33],lw='1', label="cmd_abad")
        # ax1_1.plot(time, cmd[:,34],lw='1', label="cmd_hip")
        # ax1_1.plot(time, cmd[:,35],lw='1', label="cmd_knee")
        # ax1_1.set_xlim(self.start, self.end)
        plt.autoscale(enable='true', axis='y')
        plt.legend()
        plt.grid()
        # Plot the Figure 2
        # fig2 = plt.figure(figsize=(15,10))

        # show the figure

    def plot_joint_pos_err(self):
        time = self.time
        states = self.states
        cmd = self.cmd

        plt.title("foot_pos_err_fl")

        plt.plot(time, states[:, 0] - cmd[:, 24], lw='1', label="abad")
        plt.plot(time, states[:, 1] - cmd[:, 25], lw='1', label="hip")
        plt.plot(time, states[:, 2] - cmd[:, 26], lw='1', label="knee")

        # ax1_1.set_xlim(self.start, self.end)
        plt.autoscale(enable='true', axis='y')
        plt.legend()
        plt.grid()

    def plot_joint_pos(self):
        time = self.time
        states = self.states
        cmd = self.cmd

        offset = 3

        ax1_1 = plt.subplot(2, 2, 1)
        ax1_1.set_title('foot_fl')
        ax1_1.plot(time, states[:, 0+offset], lw='1', label="abad")
        ax1_1.plot(time, states[:, 1+offset], lw='1', label="hip")
        ax1_1.plot(time, states[:, 2+offset], lw='1', label="knee")
        ax1_1.plot(time, cmd[:, 0], lw='1', label="cmd_abad")
        ax1_1.plot(time, cmd[:, 1], lw='1', label="cmd_hip")
        ax1_1.plot(time, cmd[:, 2], lw='1', label="cmd_knee")
        # ax1_1.set_xlim(self.start, self.end)
        plt.autoscale(enable='true', axis='y')
        plt.legend()
        plt.grid()

        # Plot 2: theta
        ax1_1 = plt.subplot(2, 2, 2)
        ax1_1.set_title('foot_fr')
        ax1_1.plot(time, states[:, 3+offset], lw='1', label="abad")
        ax1_1.plot(time, states[:, 4+offset], lw='1', label="hip")
        ax1_1.plot(time, states[:, 5+offset], lw='1', label="knee")
        ax1_1.plot(time, cmd[:, 3], lw='1', label="cmd_abad")
        ax1_1.plot(time, cmd[:, 4], lw='1', label="cmd_hip")
        ax1_1.plot(time, cmd[:, 5], lw='1', label="cmd_knee")
        # ax1_1.set_xlim(self.start, self.end)
        plt.autoscale(enable='true', axis='y')
        plt.legend()
        plt.grid()

        # Plot 3: I_theta
        ax1_1 = plt.subplot(2, 2, 3)
        ax1_1.set_title('foot_rl')
        ax1_1.plot(time, states[:, 6+offset], lw='1', label="abad")
        ax1_1.plot(time, states[:, 7+offset], lw='1', label="hip")
        ax1_1.plot(time, states[:, 8+offset], lw='1', label="knee")
        ax1_1.plot(time, cmd[:, 6], lw='1', label="cmd_abad")
        ax1_1.plot(time, cmd[:, 7], lw='1', label="cmd_hip")
        ax1_1.plot(time, cmd[:, 8], lw='1', label="cmd_knee")
        # ax1_1.set_xlim(self.start, self.end)
        plt.autoscale(enable='true', axis='y')
        plt.legend()
        plt.grid()

        # Plot 4: theta_ref
        ax1_1 = plt.subplot(2, 2, 4)
        ax1_1.set_title('foot_rr')
        ax1_1.plot(time, states[:, 9 +offset], lw='1', label="abad")
        ax1_1.plot(time, states[:, 10+offset], lw='1', label="hip")
        ax1_1.plot(time, states[:, 11+offset], lw='1', label="knee")
        ax1_1.plot(time, cmd[:, 9], lw='1', label="cmd_abad")
        ax1_1.plot(time, cmd[:, 10], lw='1', label="cmd_hip")
        ax1_1.plot(time, cmd[:, 11], lw='1', label="cmd_knee")
        # ax1_1.set_xlim(self.start, self.end)
        plt.autoscale(enable='true', axis='y')
        plt.legend()
        plt.grid()
        # Plot the Figure 2
        # fig2 = plt.figure(figsize=(15,10))

        # show the figure

    def plot_foot_state_err(self):
        time = self.time
        states = self.states
        cmd = self.cmd

        plt.title("foot_pos_err_fl")
        # plt.plot(time, states[:,52]-cmd[:,36],lw='1', label="x")
        # plt.plot(time, states[:,53]-cmd[:,37],lw='1', label="y")
        plt.plot(time, states[:, 54] - cmd[:, 38], lw='1', label="z")
        # plt.plot(time, cmd[:,36],lw='1', label="cmd_x")
        # plt.plot(time, cmd[:,37],lw='1', label="cmd_y")
        # plt.plot(time, cmd[:,38],lw='1', label="cmd_z")
        # plt.set_xlim(self.start, self.end)
        plt.autoscale(enable='true', axis='y')
        plt.legend()
        plt.grid()

    def plot_foot_state(self):
        time = self.time
        states = self.states
        cmd = self.cmd

        offset_state = 54-3
        offset_cmd = 69-3
        ax1_1 = plt.subplot(2, 2, 1)
        ax1_1.set_title('foot_fl')
        ax1_1.plot(time, states[:, offset_state], lw='1', label="x")
        # ax1_1.plot(time, states[:, offset_state + 1], lw='1', label="y")
        ax1_1.plot(time, states[:, offset_state + 2], lw='1', label="z")
        ax1_1.plot(time, states[:, 86], lw='1', label="contact_detected")
        ax1_1.plot(time, states[:, 87], lw='1', label="st_remain")
        ax1_1.plot(time, cmd[:, offset_cmd], lw='1', label="cmd_x")
        # ax1_1.plot(time, cmd[:, offset_cmd + 1], lw='1', label="cmd_y")
        ax1_1.plot(time, cmd[:, offset_cmd + 2], lw='1', label="cmd_z")
        ax1_1.plot(time, 0.02 * cmd[:, 38], lw='1', label="force_ref_z")

        # ax1_1.set_xlim(self.start, self.end)
        plt.autoscale(enable='true', axis='y')
        plt.legend()
        plt.grid()

        # Plot 2: theta
        ax1_1 = plt.subplot(2, 2, 2)
        ax1_1.set_title('foot_fr')
        ax1_1.plot(time, states[:, offset_state + 6], lw='1', label="x")
        ax1_1.plot(time, states[:, offset_state + 7], lw='1', label="y")
        ax1_1.plot(time, states[:, offset_state + 8], lw='1', label="z")
        ax1_1.plot(time, states[:, 89], lw='1', label="contact_detected")
        ax1_1.plot(time, states[:, 90], lw='1', label="st_remain")
        ax1_1.plot(time, cmd[:, offset_cmd + 12], lw='1', label="cmd_x")
        ax1_1.plot(time, cmd[:, offset_cmd + 13], lw='1', label="cmd_y")
        ax1_1.plot(time, cmd[:, offset_cmd + 14], lw='1', label="cmd_z")
        # ax1_1.set_xlim(self.start, self.end)
        plt.autoscale(enable='true', axis='y')
        plt.legend()
        plt.grid()

        # Plot 3: I_theta
        ax1_1 = plt.subplot(2, 2, 3)
        ax1_1.set_title('foot_rl')
        ax1_1.plot(time, states[:, offset_state + 12], lw='1', label="x")
        ax1_1.plot(time, states[:, offset_state + 13], lw='1', label="y")
        ax1_1.plot(time, states[:, offset_state + 14], lw='1', label="z")
        ax1_1.plot(time, states[:, 92], lw='1', label="contact_detected")
        ax1_1.plot(time, states[:, 93], lw='1', label="st_remain")
        ax1_1.plot(time, cmd[:, offset_cmd + 24], lw='1', label="cmd_x")
        ax1_1.plot(time, cmd[:, offset_cmd + 25], lw='1', label="cmd_y")
        ax1_1.plot(time, cmd[:, offset_cmd + 26], lw='1', label="cmd_z")
        # ax1_1.set_xlim(self.start, self.end)
        plt.autoscale(enable='true', axis='y')
        plt.legend()
        plt.grid()

        # Plot 4: theta_ref
        ax1_1 = plt.subplot(2, 2, 4)
        ax1_1.set_title('foot_rr')
        ax1_1.plot(time, states[:, offset_state + 18], lw='1', label="x")
        ax1_1.plot(time, states[:, offset_state + 19], lw='1', label="y")
        ax1_1.plot(time, states[:, offset_state + 20], lw='1', label="z")
        ax1_1.plot(time, states[:, 95], lw='1', label="contact_detected")
        ax1_1.plot(time, states[:, 96], lw='1', label="st_remain")
        ax1_1.plot(time, cmd[:, offset_cmd + 36], lw='1', label="cmd_x")
        ax1_1.plot(time, cmd[:, offset_cmd + 37], lw='1', label="cmd_y")
        ax1_1.plot(time, cmd[:, offset_cmd + 38], lw='1', label="cmd_z")
        # ax1_1.set_xlim(self.start, self.end)
        plt.autoscale(enable='true', axis='y')
        plt.legend()
        plt.grid()
        # Plot the Figure 2
        # fig2 = plt.figure(figsize=(15,10))

        # show the figure

    def plot_com_vel(self):
        time = self.time
        states = self.states
        cmd = self.cmd

        offset = 6  # velocity

        ax1_1 = plt.subplot(2, 1, 1)
        ax1_1.set_title('com vel')
        ax1_1.plot(time, states[:, 39 + offset], lw='1', label="x_imu")
        # ax1_1.plot(time, states[:, 40 + offset], lw='1', label="y_imu")
        # ax1_1.plot(time, states[:, 41 + offset], lw='1', label="z_imu")
        # ax1_1.plot(time, states[:, 97], lw='1', label="x_vision")
        # ax1_1.plot(time, states[:, 98], lw='1', label="y_vision")
        # ax1_1.plot(time, states[:, 99], lw='1', label="z_vision")
        ax1_1.plot(time, cmd[:, 48 + offset], lw='1', label="cmd_x")
        ax1_1.plot(time, cmd[:, 60], lw='1', label="footplan Vworld")
        ax1_1.plot(time, cmd[:, 63], lw='1', label="Usercmd Vx")
        # ax1_1.plot(time, cmd[:, 49 + offset], lw='1', label="cmd_y")
        # ax1_1.plot(time, cmd[:, 50 + offset], lw='1', label="cmd_z")
        ax1_1.autoscale(enable='true', axis='y')
        ax1_1.legend()
        ax1_1.grid()

        ax1_1 = plt.subplot(2, 1, 2)
        ax1_1.set_title('com rpy')
        ax1_1.plot(time, states[:, 42 + offset], lw='1', label="r_imu")
        ax1_1.plot(time, states[:, 43 + offset], lw='1', label="p_imu")
        ax1_1.plot(time, states[:, 44 + offset], lw='1', label="y_imu")
        # ax1_1.plot(time, states[:, 100], lw='1', label="r_vision")
        # ax1_1.plot(time, states[:, 101], lw='1', label="p_vision")
        # ax1_1.plot(time, states[:, 102], lw='1', label="y_vision")
        ax1_1.plot(time, cmd[:, 51 + offset], lw='1', label="cmd_x")
        ax1_1.plot(time, cmd[:, 52 + offset], lw='1', label="cmd_y")
        ax1_1.plot(time, cmd[:, 53 + offset], lw='1', label="cmd_z")
        ax1_1.autoscale(enable='true', axis='y')
        ax1_1.legend()
        ax1_1.grid()

    def plot_com(self):
        time = self.time
        states = self.states
        cmd = self.cmd

        offset = 0  # position
        offset = 6  # velocity

        ax1_1 = plt.subplot(2, 1, 1)
        ax1_1.set_title('com pos')
        ax1_1.plot(time, states[:, 39 + offset], lw='1', label="x_imu")
        # ax1_1.plot(time, states[:, 40 + offset], lw='1', label="y_imu")
        # ax1_1.plot(time, states[:, 41 + offset], lw='1', label="z_imu")
        # ax1_1.plot(time, states[:, 97], lw='1', label="x_vision")
        # ax1_1.plot(time, states[:, 98], lw='1', label="y_vision")
        # ax1_1.plot(time, states[:, 99], lw='1', label="z_vision")
        ax1_1.plot(time, cmd[:, 48 + offset], lw='1', label="cmd_x")
        ax1_1.plot(time, cmd[:, 60], lw='1', label="footplan Vworld")

        # ax1_1.plot(time, cmd[:, 49 + offset], lw='1', label="cmd_y")
        # ax1_1.plot(time, cmd[:, 50 + offset], lw='1', label="cmd_z")
        ax1_1.autoscale(enable='true', axis='y')
        ax1_1.legend()
        ax1_1.grid()

        ax1_1 = plt.subplot(2, 1, 2)
        ax1_1.set_title('com rpy')
        ax1_1.plot(time, states[:, 42], lw='1', label="r_imu")
        ax1_1.plot(time, states[:, 43], lw='1', label="p_imu")
        ax1_1.plot(time, states[:, 44], lw='1', label="y_imu")
        ax1_1.plot(time, states[:, 100], lw='1', label="r_vision")
        ax1_1.plot(time, states[:, 101], lw='1', label="p_vision")
        ax1_1.plot(time, states[:, 102], lw='1', label="y_vision")
        ax1_1.plot(time, cmd[:, 51], lw='1', label="cmd_x")
        ax1_1.plot(time, cmd[:, 52], lw='1', label="cmd_y")
        ax1_1.plot(time, cmd[:, 53], lw='1', label="cmd_z")
        ax1_1.autoscale(enable='true', axis='y')
        ax1_1.legend()
        ax1_1.grid()


if __name__ == "__main__":
    # d = DataPlot('state_swing_wbc.txt','cmd_swing_wbc.txt')
    # d = DataPlot('state_swing_pd.txt','cmd_swing_pd.txt')
    # d = DataPlot('state_pf2-4mm.txt','cmd_pf2-4mm.txt')
    # d = DataPlot('state_1217_stable.txt','cmd_1217_stable.txt')
    # d = DataPlot('state_1220.txt','cmd_1220.txt')
    d = DataPlot('../log/state.txt', '../log/cmd.txt')
    # d = DataPlot('state.txt','cmd.txt')
    d.main()

    plt.show()
