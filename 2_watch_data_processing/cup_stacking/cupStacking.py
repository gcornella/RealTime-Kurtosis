# -----------------------------------------------------------------------------
# Title: Real-time sensing of upper extremity movement diversity using kurtosis implemented on a smartwatch
# Author: Guillem Cornella i Barba
# Affiliation: Department of Mechanical and Aerospace Engineering, University of California Irvine
# Email: cornellg@uci.edu
# Date: 20th June 2024
#
# Description: This code imports the accelerometer data extracted from the Samsung watch
# and filters the values from the accelerometers to better estimate the tilt angle.
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

import pandas as pd
import matplotlib.pyplot as plt
from scipy import interpolate
import numpy as np
import math
from scipy.io import savemat
from scipy.signal import butter, filtfilt

####################################################################################
############              Import watch data               ##########################
####################################################################################
speed = 'fast'  # can also be 'fast'
df = pd.read_csv('_cup_stacking_'+speed+'_v1_watchData.csv', delimiter=';')

# Set the sampling frequency (Hz)
fs = 50

angle_array = [math.degrees(r) for r in df['ac'].values]    # Convert from radians to degrees
xs = df['xs'].values
ys = df['ys'].values
zs = df['zs'].values

# Cut the raw data [start:end] to get the experiment data, if needed
raw_signal_cut = angle_array    # [start:end]
raw_xs_cut = xs                 # [start:end]
raw_ys_cut = ys                 # [start:end]
raw_zs_cut = zs                 # [start:end]

n = len(raw_xs_cut)  # Length of the signal

# Perform FFT on the accelerometer data to filter out the accelerometer noise
cutoff_frequency = 1                    # Filter by zeroing out frequencies beyond a certain threshold

def fft_filter(raw_data):
    yf_filtered = np.fft.fft(raw_data)                      # Perform FFT
    cutoff_bin = int(cutoff_frequency / (fs / n))
    yf_filtered[(cutoff_bin + 1):(-cutoff_bin)] = 0         # Zero-out frequencies beyond the cutoff
    filtered_signal_xs = np.fft.ifft(yf_filtered)           # Inverse FFT
    extracted_noise_xs = filtered_signal_xs - raw_xs_cut    # Extract the noise from the raw signal

    return filtered_signal_xs, extracted_noise_xs


[filtered_signal_xs, extracted_noise_xs] = fft_filter(raw_xs_cut)
[filtered_signal_ys, extracted_noise_ys] = fft_filter(raw_ys_cut)
[filtered_signal_zs, extracted_noise_zs] = fft_filter(raw_zs_cut)

# Create a figure and a grid of subplots
fig, axs = plt.subplots(3, 1)  # 3 rows, 1 column
# Plotting on each subplot
axs[0].plot(raw_xs_cut)             # Plot on the first row
axs[0].plot(filtered_signal_xs)
# axs[0].plot(extracted_noise_xs)
axs[0].set_title('accelerometer ax')

axs[1].plot(raw_ys_cut)             # Plot on the second row
axs[1].plot(filtered_signal_ys)
# axs[1].plot(extracted_noise_ys)
axs[1].set_title('accelerometer ay')

axs[2].plot(raw_zs_cut)             # Plot on the third row
axs[2].plot(filtered_signal_zs)
# axs[2].plot(extracted_noise_zs)
axs[2].set_title('accelerometer az')

# Automatically adjust subplot params so that the subplot(s) fits in to the figure area.
plt.tight_layout()

# Calculate the tilt angle using the filtered accelerometer values using the tilt angle estimation function
sq = np.sqrt(filtered_signal_xs**2 + filtered_signal_ys**2 + filtered_signal_zs**2)
filtered_accel_angle_rad = np.arccos(filtered_signal_zs/sq)
filtered_accel_angle = [math.degrees(r) for r in filtered_accel_angle_rad.real]

# Cut the signal to get the experiment's data only, and not the initialization and ending data points
if speed == 'slow':
    filtered_accel_angle_cut = filtered_accel_angle[9500:21440]
else:
    filtered_accel_angle_cut = filtered_accel_angle[1925:5541]

# Create a new figure to plot the tilt angles in raw format
plt.figure(figsize=(8, 6))
plt.plot(angle_array, label='Raw')
plt.plot(filtered_accel_angle, label='filt accel')
plt.title('Raw angles and filtered angles')
plt.legend()
plt.show()

# Interpolate to the length of the cut array
f_angle_filt = interpolate.interp1d(np.linspace(0, len(filtered_accel_angle_cut), len(filtered_accel_angle_cut)), filtered_accel_angle_cut)

# Generate downsampled x values, Interpolate the original array to obtain the downsampled array
new_samples = math.ceil(len(filtered_accel_angle_cut)/15)*15    # 15 is the number of repetitions for each experiment

# Get the resampled angle values
filtered_accel_angle_cut_resampled = f_angle_filt(np.linspace(0, len(filtered_accel_angle_cut), new_samples))

# Plot the final results
plt.figure(figsize=(8, 6))  # Create a new figure
plt.plot(filtered_accel_angle_cut_resampled, label='NEW WatchTilt')
plt.legend()                # Add legend
plt.xlabel('Samples')       # Add labels and title
plt.ylabel('Degree º')
plt.title('Filtered angles resampled')
plt.grid(True)              # Show the grid
plt.show()                  # Show the plot

# Create a dictionary with the arrays and their names
data = {'watch_tiltAngle_filt': filtered_accel_angle_cut_resampled}

# Save the dictionary to a .mat file
savemat('cup_stacking_' + speed + '_results.mat', data)

