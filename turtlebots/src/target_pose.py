#! /home/proto/mambaforge/envs/ros_env/bin/python3
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

"""
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
x_previous, y_previous = 0, -1

# velocity of target = 1m/s ... Initially i.e. just assumed
# This velocity should be observed by the UAV
vel_target = 1
path_uav = [[3, 1]]
path_ugv = [[3, -1]]
x_uav_prev, y_uav_prev = 3, 0
x_uav, y_uav = 3, 1
x_ugv, y_ugv = 3, -1
odo_tb1_count, odo_tb2_count, odo_tb3_count, vel_target_count = 0, 0, 0, 0


def odo_sub_uav(odom_data):
    global x_uav, y_uav, odo_tb1_count
    global vel_target, vel_target_count
    # odo_tb1_count, x_uav, y_uav = 0, 3, 1
    x_uav = odom_data.pose.pose.position.x
    y_uav = odom_data.pose.pose.position.y
    odo_tb1_count = odo_tb1_count + 1
    vel_target = odom_data.twist.twist.linear.x
    vel_target_count = vel_target_count + 1
    # rospy.loginfo(" odo count " + str(odo_tb2_count))


def odo_sub_ugv(odom_data):
    global x_ugv, y_ugv, odo_tb2_count
    # odo_tb2_count, x_ugv, y_ugv = 0, 3, -1
    x_ugv = odom_data.pose.pose.position.x
    y_ugv = odom_data.pose.pose.position.y
    odo_tb2_count = odo_tb2_count + 1


def odo_sub(odom_data):
    global x_previous, y_previous, odo_tb3_count
    # x_previous , y_previous , odo_tb3_count = 0 , -1 , 0
    x_previous = odom_data.pose.pose.position.x
    y_previous = odom_data.pose.pose.position.y
    odo_tb3_count = odo_tb3_count + 1


# rospy.loginfo(
#     "odo counts "
#     + str(odo_tb1_count)
#     + " "
#     + str(odo_tb2_count)
#     + " "
#     + str(odo_tb3_count)
#     + "\n"
# )

count = 0


def catcher():
    global x_previous, y_previous, x_uav, y_uav, x_ugv, y_ugv, odo_tb1_count, odo_tb2_count, odo_tb3_count, x_uav_prev, y_uav_prev, path_uav, path_ugv

    rospy.init_node("Path_renderer", anonymous=False)

    r = rospy.Rate(100)

    while not rospy.is_shutdown():
        # creating subscribers and publishers for 3 turtlebots
        rospy.Subscriber("/odom", Odometry, odo_sub)
        rospy.Subscriber("/odom_tb1", Odometry, odo_sub_ugv)
        rospy.Subscriber("/odom_tb2", Odometry, odo_sub_uav)

        # At least we have some idea about the position of the bots
        if odo_tb1_count > 2 and odo_tb2_count > 2 and odo_tb3_count > 2:
            # print(
            #     "Odo Counts : ",
            #     odo_tb1_count,
            #     " ",
            #     odo_tb2_count,
            #     " ",
            #     odo_tb3_count,
            #     "  \n",
            # )
            # rospy.loginfo("This is triggered\n")
            pose_one = [x_previous, y_previous]

            pose_pred = prediction.predictor_polynomial(
                pose_one[0],
                pose_one[1],
                pose_one[0] + vel_target,
                pose_one[1] + vel_target,
            )

            while reward_uav.dist(x_ugv, y_ugv, pose_pred[0], pose_pred[1]) > 1:
                reward_coordinates = reward_uav.maximize_reward(
                    pose_pred[0], pose_pred[1], x_uav_prev, y_uav_prev, x_uav, y_uav
                )
                # print("Here are reward Coordinates:\n", reward_coordinates, "\n")
                cost_coordinates = cost_ugv.minimize_cost(
                    pose_pred[0], pose_pred[1], x_ugv, y_ugv
                )
                # print("Here are cost coordinates:\n", cost_coordinates, "\n")
                # publish the next pose of the UAV's and UGV's until UGV is less than 1m away from the target

                x_uav, y_uav = reward_coordinates[0], reward_coordinates[1]
                x_ugv, y_ugv = cost_coordinates[0], cost_coordinates[1]

                x_uav_prev = x_uav
                y_uav_prev = y_uav

                path_uav.append([x_uav, y_uav])
                path_ugv.append([x_ugv, y_ugv])

                # print(path_uav, "\n", path_ugv, "\n")

            rospy.loginfo(
                "Path of UAV until target is caught: \n" + str(path_uav) + "\n"
            )
            rospy.loginfo(
                "Path of UGV until target is caught: \n" + str(path_ugv) + "\n"
            )
            rospy.signal_shutdown("Target was caught successfully\n")

        r.sleep()


if __name__ == "__main__":
    try:
        catcher()
    except rospy.ROSInterruptException:
        pass
