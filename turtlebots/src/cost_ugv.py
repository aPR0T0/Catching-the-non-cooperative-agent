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
# This code is to get the value for the cost i.e. distance between
# ugv and the person based on the location of the ugv and the person

from cmath import sqrt, cos, sin
import numpy as np
from candidates import candidates

# x1 and y1 here are the current positons of the UGV
x1, y1 = 0, 0
# Speed of UGV
v1 = 1  # 1 m/s

"""
Information cost calculation:
We need the most reliable field of view possible from a given point to the tracked object

So, for the same we propose a cost function that focuses on predicting motion of the object over the larger FOV
"""


def dist(x1, y1, x2, y2):
    return np.real(sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2))


def minimize_cost(x1, y1, x_ugv, y_ugv):
    min_cost = np.inf

    list = candidates(x_ugv, y_ugv)
    min_cost_coordinates = np.array([list[0][0], list[0][1]])

    for i in range(len(list)):
        if min_cost > dist(x1, y1, list[i][0], list[i][1]):
            # print("Minimizing the cost\n")
            min_cost = min(dist(x1, y1, list[i][0], list[i][1]), min_cost)
            min_cost_coordinates = [list[i][0], list[i][1]]
            # print("coord", min_cost_coordinates)
            # print("min_cost:\t", min_cost, "\n")
    return min_cost_coordinates
