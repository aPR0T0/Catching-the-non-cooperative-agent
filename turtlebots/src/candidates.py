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
import numpy as np
from cmath import sin, cos, pi
from prediction import predictor_polynomial

"""
Now, we need 3 different sets of points
1. Get the location of the target and then choose four points in 1m North, South, East, and west of the target
2. Now, take the radius equal to the time_horizon and choose six equidistant points on the circle
3. Take the radius of another circle equal to twice the time horizon and choose 8 equidistant point from the circle and add them to the candidate locations
"""

# i.e. we can predict just the next step through the polynomial predictor
time_horizon = 1


def candidates(x_current, y_current):
    candidate_locations = np.empty((2, np.inf))

    candidate_locations.append(x_current, y_current + 1)
    candidate_locations.append(x_current, y_current - 1)
    candidate_locations.append(x_current + 1, y_current)
    candidate_locations.append(x_current - 1, y_current)

    for i in range(0, 360, 60):
        candidate_locations.append(
            (time_horizon * cos(i * pi / 180)) + x_current,
            (time_horizon * sin(i * pi / 180)) + y_current,
        )

    for i in range(0, 360, 45):
        candidate_locations.append(
            (2 * time_horizon * cos(i * pi / 180)) + x_current,
            (2 * time_horizon * sin(i * pi / 180)) + y_current,
        )

    return candidate_locations
