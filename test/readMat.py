import scipy.io as scio
import matplotlib.pyplot as plt
import numpy as np
import h5py

if __name__ == '__main__':
    path_data = '/home/nimapng/XiaotianHybrid/test/TestData.mat'
    data = h5py.File(path_data, 'r')
    x = list(data.keys())

    print(data['mpcTable'][:])
