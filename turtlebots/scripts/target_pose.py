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
# This will give the path that should be followed based on the shortlisted
# reachable sites by the uav

"""k
We will get a set of candidate goal pose from the candidates.py
and then according to the heuristics for the UAV we will shortlist the reachable
set of candidates for just UAV
"""

import rospy
from nav_msgs.msg import Odometry
from std_msgs.msg import Int16MultiArray
import numpy as np

import candidates, cost_ugv, prediction, reward_uav

# Subscribe to the position of the first model ( Position of the moving target ), second model ( Position of the UAV ),  and the third model ( Position of the UGV ).

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
path_uav = np.array([[3,1]])
path_ugv = np.array([[3,-1]])
x_uav, y_uav = 3, 3
x_ugv, y_ugv = 1, 1

def odo_sub(odom_data):
    x = odom_data.pose.pose.position.x
    y = odom_data.pose.pose.position.y

    return x, y

def vel_sub(odom_data):
    vel = odom_data.twist.twist.linear.x

    return vel

def catcher():
    rospy.init_node('Path_renderer', anonymous=False)

    while not rospy.is_shutdown():
        # creating subscribers and publishers for 3 turtlebots
        x_previous, y_previous = rospy.Subscriber("/pose_sub_for_agent", Odometry, odo_sub)
        x_ugv, y_ugv = rospy.Subscriber("/pose_sub_for_ugv", Odometry, odo_sub)
        x_uav, y_uav = rospy.Subscriber("/pose_sub_for_uav", Odometry, odo_sub)

        vel_target = rospy.Subscriber("/vel_sub_for_agent", Odometry, vel_sub)

        pose_one = np.array([x_previous, y_previous])

        curr_pose = prediction.predictor_polynomial(
            pose_one[0], pose_one[1], pose_one[0] + vel_target, pose_one[1] + vel_target
        )

        candidates_locations = candidates.candidates(curr_pose[0], curr_pose[1])

        reward_coordinates = reward_uav.maximize_reward(candidates_locations, x_uav, y_uav)
        cost_coordinates = cost_ugv.minimize_cost(candidates_locations, x_ugv, y_ugv)

        # publish the next pose of the UAV's and UGV's until UGV is less than 1m away from the target
        new_x_uav, new_y_uav = reward_coordinates[0], reward_coordinates[1]
        new_x_ugv, new_y_ugv = cost_coordinates[0], cost_coordinates[1]

        if reward_uav.dist(new_x_ugv, new_y_uav, curr_pose[0], curr_pose[1]) <= 1:
            rospy.signal_shutdown("Target was caught successfully\n")

if __name__ == "__main__":
    try:
        catcher()
    except rospy.ROSInterruptException:
        pass