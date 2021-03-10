#!/usr/bin/env python3

import rospy
import sensor_msgs
import argparse
import gazebo_gym.utils.utils

def callback(msg):
    npImg = gazebo_gym.utils.utils.image_to_numpy(msg)
    cy = npImg.shape[0]//2
    cx = npImg.shape[1]//2
    print(npImg[cy,cx])

if __name__ == "__main__":
    rospy.init_node('center_distance', anonymous=True, log_level=rospy.INFO)
    # ap = argparse.ArgumentParser()
    # ap.add_argument("--traj_controller_name", required=False, default="panda_arm_effort_trajectory_controller", type=str, help="Topic namespace name for the trajectory contorller to use")
    # ap.add_argument("--robot_name", required=False, default="panda", type=str, help="Topic namespace name for the trajectory contorller to use")
    # ap.add_argument('--joint_pose', nargs='+', type=float)
    # args = vars(ap.parse_known_args()[0])

    sub = rospy.Subscriber("input", sensor_msgs.msg.Image, callback)

    rospy.spin()
