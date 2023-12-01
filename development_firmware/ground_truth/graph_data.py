"""
Looks for two input files in the following formats:
1.) accel_data.txt: 
    [ax1,ay1,az1], [ax2,ay2,az2], [ax3,ay3,az3] . . . [axn,ayn,azn]]
    [[ax1,ay1,az1], [ax2,ay2,az2], [ax3,ay3,az3] . . . [axn,ayn,azn]]
2.) pos_data.txt: 

"""
from scipy.fft import rfft, rfftfreq
import matplotlib
import matplotlib.cm as cm
import numpy as np
import matplotlib.pyplot as plt
from ast import literal_eval
from matplotlib.colors import ListedColormap
import seaborn as sns
from scipy.signal import butter, lfilter, filtfilt

accel_pos = []
accel_test = []
rov_pos = []
arena_pos = []
x_pos = []
y_pos = []
z_pos = []
x_pos_test = []
y_pos_test = []
z_pos_test = []
window_size = 10
T = 60

path = 'tile_data/'

# file = open(path + 'arena_pos.txt')
# for line in file:
#     pos = literal_eval(line)


# file = open(path + 'pos_data.txt')
# for line in file:
#     pos = literal_eval(line)
#     rov_pos.append(pos)
# rov_pos = np.array(rov_pos)

# file = open(path + 'accel_data.txt')
# for line in file:
#     accel = literal_eval(line)
#     accel_pos.append(accel)

# file = open(path + 'accel_data_1_1.txt')
# for line in file:
#     accel = literal_eval(line)
#     accel_test.append(accel)

# for time_index in accel_test: 
#     x_temp = []
#     y_temp = []
#     z_temp = []
#     for data_point in time_index:
#         x_temp.append(data_point[0])
#         y_temp.append(data_point[1])
#         z_temp.append(data_point[2])
    
#     x_pos_test.append(x_temp)
#     y_pos_test.append(y_temp)
#     z_pos_test.append(z_temp)

# fs = len(z_temp)/T
# nyq = 0.5 * fs 

for time_index in accel_pos:
    x_temp = []
    y_temp = []
    z_temp = []
    for data_point in time_index:
        x_temp.append(data_point[0])
        y_temp.append(data_point[1])
        z_temp.append(data_point[2])
    
    x_pos.append(x_temp)
    y_pos.append(y_temp)
    z_pos.append(z_temp)

def butter_bandpass(lowcut, highcut, fs, order=5):
    nyq = 0.5 * fs
    low = lowcut / nyq
    high = highcut / nyq
    b, a = butter(order, [low, high], btype='band')
    return b, a


def butter_bandpass_filter(data, lowcut, highcut, fs, order=5):
    b, a = butter_bandpass(lowcut, highcut, fs, order=order)
    y = lfilter(b, a, data)
    return y

def butter_lowpass_filter(data, cutoff, fs, order):
    normal_cutoff = cutoff / nyq
    # Get the filter coefficients 
    b, a = butter(order, normal_cutoff, btype='low', analog=False)
    y = filtfilt(b, a, data)
    return y

def graph_grid():
    fig, axs = plt.subplots(3, 3)
    fig.tight_layout()
    for i in range(3):
        for j in range(3):
            accel_test = []
            x_pos_test = []
            y_pos_test = []
            z_pos_test = []
            file = open(path + 'accel_data_{0}_{1}.txt'.format(i,j))
            for line in file:
                accel = literal_eval(line)
                accel_test.append(accel)

            for time_index in accel_test: 
                x_temp = []
                y_temp = []
                z_temp = []
                for data_point in time_index:
                    x_temp.append(data_point[0])
                    y_temp.append(data_point[1])
                    z_temp.append(data_point[2])
                
                x_pos_test.append(x_temp)
                y_pos_test.append(y_temp)
                z_pos_test.append(z_temp)

            axs[i, j].plot(z_pos_test[0])
            axs[i, j].set_title('Tile [{0}, {1}]'.format(i,j))
            axs[i, j].set_ylim(9.5, 10.5)
            axs[i, j].set_xlim(500, 1000)
    plt.show()
            


def movingAvg(signal, window_size):
  
    i = 0
    moving_averages = []
    
    # Loop through the array t o
    #consider every window of size 3
    while i < len(signal) - window_size + 1:
    
        # Calculate the average of current window
        window_average = round(np.sum(signal[
        i:i+window_size]) / window_size, 2)
        
        # Store the average of current
        # window in moving average list
        moving_averages.append(window_average)
        
        # Shift window to right by one position
        i += 1
    
    return moving_averages

def graphScatter(x,y,z):
    axis = [x,y,z]
    x_pos_center = []
    y_pos_center = []
    z_pos_center = []

    for time_index in x:
        center_pos = np.array(time_index) 
        zero_pos = center_pos[window_size-1:]-movingAvg(time_index, window_size)
        x_pos_center.append(np.abs(zero_pos))
    for time_index in y:
        center_pos = np.array(time_index) 
        zero_pos = center_pos[window_size-1:]-movingAvg(time_index, window_size)
        y_pos_center.append(np.abs(zero_pos))
    for time_index in y:
        center_pos = np.array(time_index) 
        zero_pos = center_pos[window_size-1:]-movingAvg(time_index, window_size)
        z_pos_center.append(np.abs(zero_pos))

    # average_sum_time = []
    # avg = 0
    # for time_index in accel_pos:
    #     avg = 0
    #     for data_point in time_index:
    #         avg = avg + sum([i**2 for i in data_point])
    #     avg = avg / len(time_index)

    #     average_sum_time.append(avg)

    # ax = plt.axes(projection='3d')
    ax = plt.axes()

    # plt.plot(average_sum_time)
    cmap = ListedColormap([[1,0,0], [0,1,0]])

    c = []
    # print(average_sum_time)
    pos_filter_x = []
    pos_filter_y = []
    accel_filter = []
    for time in z_pos_center:
        accel_filter.append(np.sum(time))
    # index = 0

    # for point in average_sum_time:
    #     if point < 1:
    #         accel_filter.append(point)
    #         pos_filter_x.append(rov_pos[index][0])
    #         pos_filter_y.append(rov_pos[index][1])
    #         index = index + 1

    for d in accel_filter:
        if d > 25:
            c.append('green')
        else: 
            c.append('gray')
    # print(len(average_sum_time))
    # norm_accel = [float(i)/sum(accel_filter) for i in accel_filter]
    # norm_accel = [float(i)/sum(average_sum_time) for i in average_sum_time]
    print(accel_filter)
    ax.scatter(rov_pos[:,0], rov_pos[:,1], c=c)
    # sns.kdeplot(x=rov_pos[:,0], y=rov_pos[:,1], fill=True, zorder = 1, cmap="magma", cbar=True, bw_adjust=0.9)
    # ax.scatter(rov_pos[:,0], rov_pos[:,1], c='w', alpha=0.2)
    # ax.scatter3D(pos_filter_x, pos_filter_y, accel_filter, c=c, cmap=cmap)
    # ax.scatter3D(rov_pos[:,0], rov_pos[:,1], average_sum_time)
    # ax.set_zlim3d(0,1)
    plt.title("Vibration Decision Distribution")
    plt.savefig("vibration_distribution.svg", format='svg', dpi='figure')
    plt.grid()
    plt.show()

def graphProcessing(x, y, z):
    # 96, 405, 685, 822, 1150 => Motor on
    #\
    m_on = [96, 405, 685, 822, 1150]
    axis = [z]
    sns.set_style("ticks")
    fig, axs = plt.subplots(3, figsize=(10,5))
    for dir in axis:
        # print(movingAvg(z_pos[3], window_size))
        axs[0].plot(dir, label='center')
        axs[0].plot(np.arange(window_size-1,len(dir),1), movingAvg(dir, window_size), linestyle='--')
        axs[0].title.set_text("Original Signal with Moving Averge")
        axs[0].set_xticks([])
        axs[0].grid('on')
        # axs[0].plot(movingAvg(z_pos[304], window_size), linestyle='--') 
        # axs[0].plot(movingAvg(z_pos[271], window_size),  linestyle='--') 
        # axs[0].plot(movingAvg(z_pos[281], window_size),  linestyle='--') 
        # axs[0].plot(movingAvg(z_pos[292], window_size), linestyle='--') 

        # axs[0].plot(z_pos[304], label='bottom right')
        # axs[0].plot(z_pos[271], label='top right')
        # axs[0].plot(z_pos[281], label='top left')
        # axs[0].plot(z_pos[292], label='bottom left')


        center_pos = np.array(dir) 
        zero_pos = center_pos[window_size-1:]-movingAvg(dir, window_size)
        axs[1].plot(zero_pos)
        axs[2].plot(np.abs(zero_pos))

    axs[1].set_xticks([])
    axs[1].grid('on')
    axs[1].title.set_text("Center on Zero")
    axs[2].set_xticks([])
    axs[2].grid('on')
    axs[2].hlines(y=0.05, xmin=0, xmax=len(center_pos), color='r', linestyles='dotted')
    axs[2].title.set_text("Absolute Value")
    for value in m_on:
        axs[2].axvline(x = value-5, color = 'yellow', label = 'Motor On')
    # axs[2].legend()
    plt.tight_layout(pad=0.4, w_pad=0.3, h_pad=1.0)
    
    plt.show()
    fig.savefig("magnitude_analysis.svg", format='svg', dpi='figure')


def graphFourier(x,y,z):
    fig, axs = plt.subplots(2, figsize=(10,5))
    signal = np.array(z)
    axs[0].plot(signal)
    axs[0].set_xlabel("Samples")
    axs[0].set_ylabel("Acceleration")
    axs[0].title.set_text("Fourier Analysis in Z Dimension")
    signal = signal[window_size-1:]-movingAvg(signal, window_size)
    # fft_max_pos = []
    N = len(signal)
    # yf = fft(signal)
    # peak = max(2.0/N * np.abs(yf[0:N//2]))
    # fft_max_pos.append(peak)

    # xf = fftfreq(N, fs)[:N//2]
    # # plt.plot(z_pos[3], label='center')
    # # plt.plot(z_pos[304], label='bottom right')
    # # plt.plot(z_pos[271], label='top right')
    # # plt.plot(z_pos[281], label='top left')
    # # plt.plot(z_pos[292], label='bottom left')
    # plt.plot(xf, 2.0/N * np.abs(yf[0:N//2]))
    # plt.grid()
    # print(fft_max_pos)

    # Note the extra 'r' at the front
    yf = rfft(signal)
    xf = rfftfreq(N, 1 / fs)
    # axs[1].set_title("Foureir Transform of Z axis acceleration")
    axs[1].set_xlabel("Frequency (hz)")
    axs[1].set_ylabel("Amplitude")
    axs[1].plot(xf, np.abs(yf))
    
    # ax = plt.axes(projection='3d')
    # ax.scatter3D(rov_pos[:,0], rov_pos[:,1], fft_max_pos)
    # plt.legend()
    plt.tight_layout(pad=0.4, w_pad=0.3, h_pad=1.0)
    plt.show()
    fig.savefig("fourier_analysis.svg", format='svg', dpi='figure')

def graph_placement():
    plt.scatter(rov_pos[:,0], rov_pos[:,1])
    for i in range(len(rov_pos[:,0])):
        plt.text(rov_pos[i,0], rov_pos[i,1], str(i))
    plt.show()


# graph_placement()
sns.set_style("ticks")
# graphProcessing(x_pos_test[0], y_pos_test[0], z_pos_test[0]) 
graph_grid()
# z_filter = butter_lowpass_filter(z_pos_test[0], 2, fs, 2)
# plt.plot(z_filter)
# graphFourier(x_pos_test[0], y_pos_test[0], z_pos_test[0])
# plt.show()
# graphScatter(x_pos, y_pos, z_pos)