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

# This will give the reward based on the reachable states and the
# candidate states provided

from cmath import sqrt, cos, sin
import numpy as np

# x0 and y0 here are the previous postions of the UAV
x0, y0 = -1, -1
# x1 and y1 here are the current positons of the UAV
x1, y1 = 0, 0
# Angle of vision in degrees
theta = 60
# radius of vision is 5m from the UAV
rov = 5


def dist(x1, y1, x2, y2):
    return sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)


"""
Information reward calculation:
We need the most reliable field of view possible from a given point to the tracked object

So, for the same we propose a reward function that focuses on predicting motion of the object over the larger FOV
"""


def reward(x0, y0, x1, y1, x_target, y_target):
    vec1 = np.array([(x1 - x0), (y1 - y0)], [(y0 - y1), (x1 - x0)])
    vec2 = np.array(
        [(x0 - x1) * x_target + (y1 - y0) * y_target], [(x0 - x1) * y1 + (y1 - y0) * x1]
    )
    x2, y2 = np.linalg.solve(vec1, vec2)[0], np.linalg.solve(vec1, vec2)[1]
    pose = np.array([x2], [y2], [0])

    z = dist(x1, y1, x2, y2)
    pose_left, pose_right = np.matmul(
        np.array(
            [cos(theta / 2), -sin(theta / 2), 0],
            [sin(theta / 2), cos(theta / 2), 0],
            [0, 0, 1],
        ),
        pose,
    ), np.matmul(
        np.array(
            [cos(-theta / 2), -sin(-theta / 2), 0],
            [sin(-theta / 2), cos(-theta / 2), 0],
            [0, 0, 1],
        ),
        pose,
    )

    xt, yt = dist(pose_left[0], pose_left[1], x2, y2), dist(
        pose_right[0], pose_right[1], x2, y2
    )

    reward_value = xt * yt * (rov - z) * z

    return reward_value


def maximize_reward(list, x0, y0):
    max_reward = 0
    max_reward_coordinates = np.array(list[0], list[1])
    for i in range(len(list)):
        max_reward = max(reward(x0, y0, x1, y1, list[i][0], list[i][1]), max_reward)