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

from prediction import predictor_polynomial

"""
We first need the current location
Then we need to get all the desired setpoints for UGV and UAV
Then we need to go for the next point that is Update step and do the previous step for the same
Above procedure is repeated until distance between UGV and target is <= 1
"""
x_previous, y_previous = 0, 0

# velocity of target = 1m/s ... Initially i.e. just assumed
# This velocity should be observed by the UAV
vel_target = 1

# i.e. we can predict just the next step through the polynomial predictor
time_horizon = 1

x_current, y_current = predictor_polynomial(
    x_previous, y_previous, x_previous + vel_target, y_previous + vel_target
)

"""
Now, we need 3 different sets of points
1. Get the location of the target and then choose four points in 1m North, South, East, and west of the target
2. Now, take the radius equal to the time_horizon and choose six equidistant points on the circle
3. Take the radius of another circle equal to twice the time horizon and choose 8 equidistant point from the circle and add them to the candidate locations
"""

def candidates():
    


if __name__ == "__main__":
    candidates()