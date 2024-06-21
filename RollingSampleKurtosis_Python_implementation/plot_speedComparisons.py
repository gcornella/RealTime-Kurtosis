# -----------------------------------------------------------------------------
# Title: Real-time sensing of upper extremity movement diversity using kurtosis implemented on a smartwatch
# Author: Guillem Cornella i Barba
# Affiliation: Department of Mechanical and Aerospace Engineering, University of California Irvine
# Email: cornellg@uci.edu
# Date: 20th June 2024
#
# Description: This code plots the results of different approaches to calculate kurtosis
# showing that our proposed Rolling Sample Kurtosis algorithm is faster than the others.
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

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.lines as mlines

# Experimental data obtained from our tests (list size and execution time)
list_sizes = np.array([0, 100, 500, 750, 1000, 1500, 2000, 3000, 4000, 5000])

# Standard Kurtosis Approach (values in seconds), (1 code repetition, and 100 code repetitions execution times)
entire1rep = np.array([0, 0.0035103000700473785, 0.10293109994381666, 0.20530409971252084, 0.36622210033237934, 0.926046899985522,
                       1.4716998999938369, 3.3640983002260327, 5.903011799789965, 9.134344600141048])
entire100rep = np.array([0, 0.3820540998131037, 9.212478999979794, 20.840730299707502, 38.761207500007, 82.37241619965062,
                         157.64450010005385, 341.23133700015023, 615.1036628000438, 950.5178276002407])
# SciPy function approach
scipy1rep = np.array([0, 0.028022899758070707, 0.14200800005346537, 0.2153349998407066, 0.30247109988704324,
                      0.671157700009644, 1.117040100041777, 1.6577463001012802, 2.302802700083703])
scipy100rep = np.array([0, 2.7755446997471154, 14.94257490010932, 22.33078940026462, 31.62791100004688, 48.3898672000505,
                        73.46399520011619, 117.49322899989784, 167.00498059997335, 228.91165419993922])

# Rolling Sample Kurtosis
rsk1rep = np.array([0, 0.0001984001137316227, 0.000773200299590826, 0.001308700069785118, 0.0015855003148317337, 0.0026422999799251556,
                    0.0031574000604450703, 0.004659799858927727, 0.006097500212490559, 0.007816399913281202])
rsk100reps = np.array([0, 0.015446899924427271, 0.08323229989036918, 0.1389234997332096, 0.18066020030528307, 0.2848979998379946,
                       0.33999310014769435, 0.5233772001229227, 0.6700568003579974, 0.7777939001098275])
rsk500reps = np.array([0, 0.07724030036479235, 0.40092469984665513, 0.7443058001808822, 0.7823079000227153, 1.5036132000386715,
                       1.6706104995682836, 2.368354299571365, 3.2126414999365807, 4.163174000103027])

# Generate predictions for a range of list sizes
list_sizes_range = np.linspace(min(list_sizes), max(list_sizes), 100)

# Fit a polynomial regression model
degree = 2

# Fit the standard approach points
standard_coefficients = np.polyfit(list_sizes, entire100rep, degree)
standard_poly_function = np.poly1d(standard_coefficients) # Create a polynomial function based on the coefficients
standard_pred_exec_times = standard_poly_function(list_sizes_range)
print(standard_poly_function)

# Fit the scipy points
scipy_coefficients = np.polyfit(list_sizes, scipy100rep, degree)
scipy_poly_function = np.poly1d(scipy_coefficients)
scipy_pred_exec_times = scipy_poly_function(list_sizes_range)
print(scipy_poly_function)

# Fit the RSK points
rsk_coefficients = np.polyfit(list_sizes, rsk100reps, degree)
rsk_poly_function = np.poly1d(rsk_coefficients)
rsk_pred_exec_times = rsk_poly_function(list_sizes_range)
print(rsk_poly_function)


fig, ax1 = plt.subplots(figsize=(10,7))
plt.title('Execution time (x100 repetitions)', fontsize=20)
ax1.plot(list_sizes, entire100rep, color='tab:blue', linestyle = '-', linewidth = 4,label='EntireBuffer Data')
ax1.plot(list_sizes, scipy100rep, color='tab:orange', linestyle = '-', linewidth = 4,label='SciPy Data')
ax1.plot(list_sizes, rsk100reps, color='tab:red', linestyle = '-', linewidth = 4, label='RSK Data')
ax1.tick_params(axis='x', labelsize=20)
ax1.tick_params(axis='y', labelsize=20)
ax1.set_xlabel('Buffer Size (samples)', fontsize=20)
ax1.set_ylabel('time (s)', fontsize=20)
ax1.grid(True, which='both')

# Creating custom legend entries with lines of specific colors
# Here, setting legend line colors different from the plot line colors
legend_line1 = mlines.Line2D([], [], color='blue', linestyle='-', label='Standard Kurtosis formula')
legend_line2 = mlines.Line2D([], [], color='orange', linestyle='-', label='SciPy Kurtosis function')
legend_line3 = mlines.Line2D([], [], color='red', linestyle='-', label='Rolling Sample Kurtosis')
plt.legend(handles=[legend_line1, legend_line2, legend_line3], fontsize=20) # Creating the legend with custom entries

# Add the fits
'''
ax1.plot(list_sizes_range, standard_pred_exec_times, color='tab:blue', linestyle = '--', linewidth = 3,label='EntireBuffer fit')
ax1.plot(list_sizes_range, scipy_pred_exec_times, color='tab:orange', linestyle = '--', linewidth = 3,label='SciPy fit')
ax1.plot(list_sizes_range, rsk_pred_exec_times, color='tab:red', linestyle = '--', linewidth = 3, label='RSK fit')
'''

# Logaritmic plot
fig, ax2 = plt.subplots(figsize=(10,7))
plt.title('LOG Execution time (x100 repetitions)', fontsize=20)
ax2.plot(list_sizes, np.log10(rsk100reps), color='tab:red', linestyle = '-', linewidth = 4,label='Log RSK Data')
ax2.plot(list_sizes, np.log10(entire100rep), color='tab:blue', linestyle = '-', linewidth = 4,label='Log EntireBuffer Data')
ax2.plot(list_sizes, np.log10(scipy100rep), color='tab:orange', linestyle = '-', linewidth = 4,label='Log SciPy Data')
ax2.tick_params(axis='x', labelsize=20)
ax2.tick_params(axis='y', labelsize=20)
ax2.set_xlabel('Buffer Size (samples)', fontsize=20)
ax2.set_ylabel('log10(time)', fontsize=20)  # we already handled the x-label with ax1
ax2.grid(True, which='both')
plt.legend(handles=[legend_line1, legend_line2, legend_line3], fontsize=20)


## Plot The ratio
fig, ax3 = plt.subplots(figsize=(10,7))
plt.title('Ratios of execution time',fontsize=20)
ax3.plot(list_sizes, entire100rep/rsk100reps, color='tab:green', linestyle = '-', linewidth = 4, label='Rolling Sample Kurtosis vs Standard')
ax3.plot(list_sizes, scipy100rep/rsk100reps, color='tab:orange', linestyle = '-', linewidth = 4,label='Rolling Sample Kurtosis vs SciPy')
ax3.tick_params(axis='x', labelsize=20)
ax3.tick_params(axis='y', labelsize=20)
ax3.grid(True)
ax3.set_xlabel('Buffer Size (samples)', fontsize=20)
ax3.set_ylabel('Execution time ratio', fontsize=20)
plt.legend(fontsize=20)
plt.show()