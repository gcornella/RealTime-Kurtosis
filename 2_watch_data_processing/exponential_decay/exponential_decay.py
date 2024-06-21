# -----------------------------------------------------------------------------
# Title: Real-time sensing of upper extremity movement diversity using kurtosis implemented on a smartwatch
# Author: Guillem Cornella i Barba
# Affiliation: Department of Mechanical and Aerospace Engineering, University of California Irvine
# Email: cornellg@uci.edu
# Date: 20th June 2024
#
# Description: This code calculates the exponential decay of the kurtosis calculated
# with an increasing window size. We calculate the envelope of the kurtosis, and then
# fit an exponential decay function to its data points, to obtain a tau value.
# ------------------------------------------------------------------------------
#
# Copyright (c) [2024] [Guillem Cornella i Barba]
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.
# -----------------------------------------------------------------------------

import obspy as obspy
from scipy.io import loadmat
import numpy as np
import matplotlib.pyplot as plt
from scipy.signal import hilbert
from scipy.signal import find_peaks
from scipy.optimize import curve_fit
from itertools import cycle

def hl_envelopes_idx(s, dmin=1, dmax=1, split=False):

    # locals min
    lmin = (np.diff(np.sign(np.diff(s))) > 0).nonzero()[0] + 1
    # locals max
    lmax = (np.diff(np.sign(np.diff(s))) < 0).nonzero()[0] + 1

    if split:
        # s_mid is zero if s centered around x-axis or more generally mean of signal
        s_mid = np.mean(s)
        # pre-sorting of locals min based on relative position with respect to s_mid
        lmin = lmin[s[lmin]<s_mid]
        # pre-sorting of local max based on relative position with respect to s_mid
        lmax = lmax[s[lmax]>s_mid]

    # global min of dmin-chunks of locals min
    lmin = lmin[[i+np.argmin(s[lmin[i:i+dmin]]) for i in range(0,len(lmin),dmin)]]
    # global max of dmax-chunks of locals max
    lmax = lmax[[i+np.argmax(s[lmax[i:i+dmax]]) for i in range(0,len(lmax),dmax)]]

    return lmin,lmax

# Fit an exponential decay
def exponential_func(x, A, lambd, C):
    return A * np.exp(-lambd * x) + C

def remove_elements_by_index(arr, indices_to_remove):
    return [elem for index, elem in enumerate(arr) if index not in indices_to_remove]

# Define plot fonts and sizes
plt.rcParams.update({
    'font.size': 20,          # Default font size
    'axes.labelsize': 20,     # Font size of x and y labels
    'xtick.labelsize': 20,    # Font size of x-axis ticks
    'ytick.labelsize': 20,    # Font size of y-axis ticks
})

# Define sampling frequency (Hz)
fs = 50

times_dict_slow = {}
times_dict_fast = {}
for traj in ['shuffling_cards', 'cup_stacking', 'arm_wrestling','handshaking','exploration', 'simulated_normal']:
    fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(8, 6), sharex=True)  # figsize sets the width and height of the figure

    # Load .mat file
    GT_slow = loadmat(traj+'_GT_decay_slow.mat')
    GT_fast = loadmat(traj+'_GT_decay_fast.mat')
    watch_slow = loadmat(traj+'_watch_decay_slow.mat')
    watch_fast = loadmat(traj+'_watch_decay_fast.mat')

    # Access the array and remove nans
    GT_slow = GT_slow['kurt_cut_gt_slow'].flatten() # .flatten() is used to convert MATLAB matrix to 1D array
    GT_slow = GT_slow[~np.isnan(GT_slow)]           # Remove NaNs
    GT_fast = GT_fast['kurt_cut_gt_fast'].flatten() # .flatten() is used to convert MATLAB matrix to 1D array
    GT_fast = GT_fast[~np.isnan(GT_fast)]           # Remove NaNs
    watch_slow = watch_slow['kurt_cut_watch_slow'].flatten() # .flatten() is used to convert MATLAB matrix to 1D array
    watch_slow = watch_slow[~np.isnan(watch_slow)]           # Remove NaNs
    watch_fast = watch_fast['kurt_cut_watch_fast'].flatten() # .flatten() is used to convert MATLAB matrix to 1D array
    watch_fast = watch_fast[~np.isnan(watch_fast)]           # Remove NaNs

    x_slow = np.linspace(0, len(watch_slow)/fs, len(watch_slow))  # Array for x-axis (100 points from 0 to 10)
    x_fast = np.linspace(0, len(watch_fast)/fs, len(watch_fast))

    # Envelope
    lmin_slow, lmax_slow = hl_envelopes_idx(watch_slow)
    lmin_fast, lmax_fast = hl_envelopes_idx(watch_fast)

    # Perform curve fitting
    if traj == 'shuffling_cards':
        p0_slow = (3.0, 0.1, 0.0)
        p0_fast = (6.0, 0.5, 0.0)
        envelope_watch_max_slow = remove_elements_by_index(lmax_slow, [1, 2, 3, 4])
        envelope_watch_max_fast = remove_elements_by_index(lmax_fast, [0, 2])
    elif traj == 'cup_stacking':
        p0_slow = (1.5, 0.1, 0.0)
        p0_fast = (1, 0.1, 0)
        envelope_watch_max_slow = remove_elements_by_index(lmin_slow, [1, 4])
        envelope_watch_max_fast = remove_elements_by_index(lmin_fast, [0, 3])
    elif traj == 'arm_wrestling':
        p0_slow = (2.0, 0.1, 0.0)
        p0_fast = (4.0, 0.1, 0.0)
        envelope_watch_max_slow = remove_elements_by_index(lmax_slow, [0,1,2,3])
        envelope_watch_max_fast = remove_elements_by_index(lmax_fast, [0])
    elif traj == 'handshaking':
        p0_slow = (20.0, 0.1, 0.0)
        p0_fast = (50.0, 0.1, 0.0)
        envelope_watch_max_slow = remove_elements_by_index(lmax_slow, [0, 1, 2, 3, 4, 5,6,7,8])
        envelope_watch_max_fast = remove_elements_by_index(lmax_fast, [0,1,2,3])
    elif traj == 'exploration':
        p0_slow = (3.0, 0.1, 0.0)
        p0_fast = (6.0, 0.1, 0.0)
        envelope_watch_max_slow = remove_elements_by_index(lmax_slow, [0,1,3])
        envelope_watch_max_fast = remove_elements_by_index(lmax_fast, [0,1])
    else:
        p0_slow = (3.0, 0.1, 0.0)
        p0_fast = (6.0, 0.1, 0.0)
        envelope_watch_max_slow = remove_elements_by_index(lmax_slow, [0, 1, 2,3,4,5,6])
        envelope_watch_max_fast = remove_elements_by_index(lmax_fast, [0, 1])

    popt_slow, pcov_slow = curve_fit(exponential_func, x_slow[envelope_watch_max_slow], watch_slow[envelope_watch_max_slow], p0=p0_slow)
    popt_fast, pcov_fast = curve_fit(exponential_func, x_fast[envelope_watch_max_fast], watch_fast[envelope_watch_max_fast], p0=p0_fast)

    # Get the parameters from the fit
    A_fit_slow, lambd_fit_slow, C_fit_slow = popt_slow
    A_fit_fast, lambd_fit_fast, C_fit_fast = popt_fast
    #print(lambd_fit_slow, lambd_fit_fast)


    # Calculate the time to decrease to 10% of the initial value
    decrease = np.linspace(1, 0, num=int((1 - 0) / 0.02) + 1)
    print(decrease)
    time_to_decrease_x_percent_slow = []
    time_to_decrease_x_percent_fast = []
    for i, d in enumerate(decrease):
        print(i,d)
        time_to_decrease_x_percent_slow.append(-np.log(d) * 1 / lambd_fit_slow)
        time_to_decrease_x_percent_fast.append(-np.log(d) * 1 / lambd_fit_fast)

    times_dict_slow[traj] = time_to_decrease_x_percent_slow
    times_dict_fast[traj] = time_to_decrease_x_percent_fast

    # Generate fitted curve for plotting
    signal_fit_slow = exponential_func(x_slow, A_fit_slow, lambd_fit_slow,
                                       C_fit_slow)  # Use the fitted parameters to generate y values
    signal_fit_fast = exponential_func(x_fast, A_fit_fast, lambd_fit_fast,
                                       C_fit_slow)  # Use the fitted parameters to generate y values


    x_slow = np.linspace(0, len(watch_slow)/fs, len(watch_slow))  # Array for x-axis (100 points from 0 to 10)
    x_fast = np.linspace(0, len(watch_fast)/fs, len(watch_fast))

    # Plotting on the same subplot (ax)
    ax1.plot(x_slow, watch_slow, label='watch slow', color='red')
    ax1.plot(x_slow[envelope_watch_max_slow], watch_slow[envelope_watch_max_slow], color = 'm', label='envelope')
    ax1.plot(x_slow, signal_fit_slow, label='Exponential fit', color='green')

    ax2.plot(x_fast, watch_fast, label='watch fast', color='blue')
    ax2.plot(x_fast[envelope_watch_max_fast], watch_fast[envelope_watch_max_fast],  color='m', label='envelope')
    ax2.plot(x_fast, signal_fit_fast, label='Exponential fit', color='green')

    ax3.plot(time_to_decrease_x_percent_slow, decrease , color='red', label='slow decay')
    ax3.plot(time_to_decrease_x_percent_fast, decrease,  color='blue', label='fast decay')
    # Set title and labels
    ax1.set_title(traj)

    ax1.set_ylabel('Kurtosis')
    ax2.set_ylabel('Kurtosis')
    ax3.set_ylabel('% initial kurt')

    # Display legend
    ax1.legend()
    ax2.legend()
    ax3.legend()
    plt.xlabel('Time (s)')

    ax1.grid()
    ax2.grid()
    ax3.grid()

# Create a new figure
plt.figure(figsize=(10, 6))

# Plot each list with its key as the label
colors = cycle(['b', 'g', 'r', 'c', 'm'])
for key, value_list in times_dict_slow.items():
    if key != 'simulated_normal':
        color = next(colors)
        plt.plot(value_list, decrease*100, color = color, label=key,  linewidth=3)
        print(value_list)
        plt.axvline(x = value_list[-2] , color = color, linestyle='--', linewidth=3)

plt.axhline(y=0.02, color='gray', linestyle='--', linewidth=3)

# Add legend
# plt.legend()

# Add titles and labels
plt.title('Slow movement speed')
plt.xlabel('Time (s)')
plt.ylabel('Measured kurtosis decay (%)')
plt.grid()

# Show the plot
plt.show()