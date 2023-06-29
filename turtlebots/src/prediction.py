"""
    MIT License

    Copyright (c) 2023 Mohd Alqama Shaikh

    Permission is hereby granted, free of charge, to any person obtaining a copy
    of this software and associated documentation files (the "Software"), to deal
    in the Software without restriction, including without limitation the rights
    to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
    copies of the Software, and to permit persons to whom the Software is
    furnished to do so, subject to the following conditions:

    The above copyright notice and this permission notice shall be included in all
    copies or substantial portions of the Software.

    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
    IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
    FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
    AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
    LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
    OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
    SOFTWARE.
"""

#! usr/bin/python3
# This is a polynomial predictor which will get the future state of the person
# using the simple position and velocity state variables
"""
Function : predictor_polynomial curve
Inputs : current position of target and a position second before of the target
"""
import numpy as np
from math import sqrt


# This is only about a forward direction
def predictor_polynomial(x0, y0, x1, y1):
    global x2, y2

    x2 = sqrt(((x0 + x1 + 1) / 2) ** 2) * abs(x1) / x1
    y2 = sqrt(((y0 + y1 + 1) / 2) ** 2) * abs(y1) / y1

    pose_est = np.array([x2, y2])
    return pose_est


"""
Function : predictor_kalman
Inputs : current position of target and a position second before of the target
"""


def predictor_kalman():
    x2 = 0
    y2 = 0
