# -----------------------------------------------------------------------------
# Title: Real-time sensing of upper extremity movement diversity using kurtosis implemented on a smartwatch
# Author: Guillem Cornella i Barba
# Affiliation: Department of Mechanical and Aerospace Engineering, University of California Irvine
# Email: cornellg@uci.edu
# Date: 20th June 2024
#
# Description: This code implements the validation of the Rolling Sample Kurtosis algorithm in Python
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
import timeit
import json

def calculateMean(data, _iter):
    sum = 0.0
    for value in data:
        if value is not None:
            sum += value
    return sum/_iter


def calculateStandardDeviation(data, mean, _iter):
    sumSquaredDeviations = 0.0
    for value in data:
        if value is not None:
            sumSquaredDeviations += (value-mean)**2
    variance = sumSquaredDeviations/_iter
    return variance


def calculateFourthMoment(data, mean, _iter):
    sumFourthPowerDeviations = 0.0
    for value in data:
        if value is not None:
            sumFourthPowerDeviations += (value-mean)**4
    return sumFourthPowerDeviations/_iter


def calculate_kurtosis(_circularBuffer, _iter):
    data = _circularBuffer

    mean = calculateMean(data, _iter)

    ''' # This section is one order of magnitude slower
    start_time2 = timeit.default_timer()
    mean2 = np.nanmean(_circularBuffer)
    stop_time2 = timeit.default_timer()
    print('easy mean: ', (stop_time2 - start_time2)*1000000)
    '''

    variance = calculateStandardDeviation(data, mean, _iter)
    fourthMoment = calculateFourthMoment(data, mean, _iter)
    # To avoid division by 0:
    if variance != 0:
        kurt = fourthMoment / (variance**2) # same as standardDeviation^4
    else:
        kurt = 0
    return kurt

#################################################################################################################
def rolling_kurtosis(_newValue, _poppedValue, _iter, _circularBuffer, _currentIndex, _mean, _M2, _M3, _M4, _kurtosis):

    # Add first value
    if _iter == 1 and _poppedValue == -1:
        newMean =_newValue
        newM2 = 0
        newM3 = 0
        newM4 = 0
        _kurtosis = 0

    # Buffer is not full, apply incremental approach
    elif _poppedValue == -1:

        delta = _newValue-_mean
        delta_n = delta/_iter
        delta_n2 = delta_n*delta_n
        term1 = delta * delta_n * (_iter-1)

        # Cumulative Sample Mean (CSM)
        newMean = _mean + delta_n

        # Mid variables
        dif1 = _newValue - newMean
        dif2 = newMean - _mean

        # Cumulative Sample Variance (CSV)
        newM2 = _M2 + dif1*(_newValue - _mean)

        # Cumulative Sample Skewness (CSS)
        newM3 = _M3 - 3*dif2*_M2 + \
                (newM2 - _M2)*(dif1 - dif2)

        # Cumulative Sample Kurtosis (CSK)
        newM4 = _M4 + term1 * delta_n2 *(_iter*_iter - 3*_iter + 3) + 6*delta_n2*_M2 - 4*delta_n*_M3

        # Kurtosis
        _kurtosis = _iter * newM4 / (newM2 * newM2)

    # Buffer is full. Apply Rolling Approach
    else:
        dif3 = _newValue - _poppedValue

        # Rolling Sample Mean (RSM)
        newMean = _mean + dif3/MAX_SIZE

        dif4 = _poppedValue - newMean
        dif5 = _newValue - newMean
        dif6 = newMean - _mean
        dif7 = _poppedValue - _mean
        sum1 = dif4 + dif5

        # Rolling Sample Variance (RSV)
        newM2 = _M2 + dif3*(dif5+dif7)

        # Rolling Sample Skewness (RSS)
        newM3 = _M3 - 3*dif6*\
                _M2 + dif3*\
                (dif7*(dif4-dif6)+ dif5* sum1 )

        # Rolling Sample Kurtosis (RSK)
        dif6_2 = dif6*dif6
        newM4 = _M4 -4*dif6*_M3 + 6*dif6_2*_M2 +dif3*\
                 (dif6*dif6_2+sum1*
                  (dif5*dif5 + dif4*dif4))

        # Kurtosis
        _kurtosis = MAX_SIZE * newM4 / (newM2 * newM2)

    _mean = newMean
    _M2 = newM2
    _M3 = newM3
    _M4 = newM4

    return _mean, _M2, _M3, _M4, _kurtosis

def simulate_onSensorChanged(_newValue, _poppedValue, _iter, _circularBuffer, _currentIndex):
    # Simulate that a new value is received by the sensor
    if _iter < MAX_SIZE:
        _iter += 1
    else:
        _poppedValue = _circularBuffer[_currentIndex]

    #print('changing old {} for new {}'.format(_poppedValue, _newValue))
    #print('iter : ',_iter, ' poppedValue: ',_poppedValue)
    # Add value to the circular buffer
    _circularBuffer[_currentIndex] = _newValue

    # Update the current index to th next value
    _currentIndex = (_currentIndex+1) % MAX_SIZE

    return _poppedValue, _iter, _circularBuffer, _currentIndex

def main(_poppedValue, _iter, _circularBuffer, _currentIndex, _mean, _M2, _M3, _M4, _kurtosis):

    for i in range(MAX_SIZE, MAX_SIZE*2): # Fill the circular buffer and add values until doubling its size
        #print('-----------------------------------------------------------------------------')

        # Get sensor value
        _newValue = SIM_SENSOR_VALUES[i]
        _poppedValue, _iter, _circularBuffer, _currentIndex = simulate_onSensorChanged(_newValue, _poppedValue, _iter, _circularBuffer, _currentIndex)

        # ROLLING SAMPLE KURTOSIS
        _mean, _M2, _M3, _M4, _kurtosis = rolling_kurtosis(_newValue, _poppedValue, _iter, _circularBuffer, _currentIndex, _mean, _M2, _M3, _M4, _kurtosis)

        # ENTIRE BUFFER EVERY TIME
        #kurt = calculate_kurtosis(_circularBuffer, _iter)

        # USING PYTHON LIBRARY SCIPY
        #kurt_py = kurtosis(_circularBuffer, fisher=False)


    print('Using RSK: ', _kurtosis)
    # print('Using entire buffer: ', kurt)
    # print('Using scipy.stats: ', kurt_py)

# Press the green button in the gutter to run the script.
def fill_buffer():
    _newValue, _poppedValue = 0, -1
    _iter, _currentIndex = 0, 0
    _circularBuffer = [None] * MAX_SIZE

    _mean, _M2, _M3, _M4, _kurtosis = 0, 0, 0, 0, 0

    for i in range(0, MAX_SIZE):  # Fill the circular buffer and add values until doubling its size
        # print('-----------------------------------------------------------------------------')

        # Get sensor value
        _newValue = SIM_SENSOR_VALUES[i]
        _poppedValue, _iter, _circularBuffer, _currentIndex = simulate_onSensorChanged(_newValue, _poppedValue, _iter,
                                                                                       _circularBuffer, _currentIndex)

        # ROLLING KURTOSIS
        _mean, _M2, _M3, _M4, _kurtosis = rolling_kurtosis(_newValue, _poppedValue, _iter, _circularBuffer,_currentIndex, _mean, _M2, _M3, _M4, _kurtosis)

        # ENTIRE BUFFER EVERY TIME
        #kurt = calculate_kurtosis(_circularBuffer, _iter)

        # USING PYTHON LIBRARY SCIPY
        # If False, Pearson’s definition is used (normal ==> 3.0).
        # If omit, NaNs will be omitted when performing the calculation
        # my_list = [np.nan if x is None else x for x in _circularBuffer]  # Convert None to nan
        # kurt_py = kurtosis(my_list, fisher=False, nan_policy='omit')

    # print('Buffer full: ', _circularBuffer)
    return _poppedValue, _iter, _circularBuffer, _currentIndex, _mean, _M2, _M3, _M4, _kurtosis

if __name__ == '__main__':
    # Define global CONSTANTS that will not change during the execution
    MAX_SIZE = 5000 # This value should be at least half of the size from "data_10000.json"

    # Open the JSON file (list with different lengths)
    with open("data_10000.json", "r") as file:
        # Load the JSON data as a list
        SIM_SENSOR_VALUES = json.load(file)
        # Convert to array for increased efficiency
        SIM_SENSOR_VALUES = np.array(SIM_SENSOR_VALUES)

    print('Data list loaded')
    print(SIM_SENSOR_VALUES)

    _poppedValue, _iter, _circularBuffer, _currentIndex, _mean, _M2, _M3, _M4, _kurtosis = fill_buffer()

    # setup function just runs once, then the main function keeps repeating
    elapsed_time = timeit.timeit(lambda: main(_poppedValue, _iter, _circularBuffer, _currentIndex, _mean, _M2, _M3, _M4, _kurtosis),
                                 setup='from __main__ import main, fill_buffer; fill_buffer()', number=100)  # Repeat 'number' times
    print('Total execution time: ', elapsed_time, ' seconds')


