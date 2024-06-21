# -----------------------------------------------------------------------------
# Title: Real-time sensing of upper extremity movement diversity using kurtosis implemented on a smartwatch
# Author: Guillem Cornella i Barba
# Affiliation: Department of Mechanical and Aerospace Engineering, University of California Irvine
# Email: cornellg@uci.edu
# Date: 20th June 2024
#
# Description: This code generates a random list of data to test the kurtosis algorithm.
# See that we are generating a uniform distribution of values, so the kurtosis should be 1.8
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

import json
import random

# Generate some list with a specified length, filled with floats of 4 decimals
maxSize = 10000
data = [round(random.uniform(0, 1), 4) for _ in range(maxSize)]

# Define the filename for the JSON file
json_file = "data_"+str(maxSize)+".json"

# Write the data to the JSON file and save it
with open(json_file, "w") as file:
    json.dump(data, file, indent=4)

print("Data has been saved to", json_file)
